/**
 ******************************************************************************
 * @addtogroup Modules Modules
 * @{
 * @addtogroup Control Control Module
 * @{
 *
 * @file       transmitter_control.c
 * @author     dRonin, http://dRonin.org/, Copyright (C) 2015-2016
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2017
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Handles R/C link and driving mode.
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>
 *
 * Additional note on redistribution: The copyright and license notices above
 * must be maintained in each individual source file that is a derivative work
 * of this source file; otherwise redistribution is prohibited.
 */

#include "openpilot.h"
#include "control.h"
#include "transmitter_control.h"
#include "pios_thread.h"
// #include "uavcan_interface.h"

#include "altitudeholddesired.h"
#include "altitudeholdsettings.h"
#include "baroaltitude.h"
#include "flighttelemetrystats.h"
#include "drivingstatus.h"
#include "loitercommand.h"
#include "pathdesired.h"
#include "positionactual.h"
#include "receiveractivity.h"
#include "systemsettings.h"
#include "shareddefs.h"
#include "caractuatordesired.h"
#include "carnavigationdesired.h"

#include "misc_math.h"
#include "auto_car.h"

#include "jlink_rtt.h"

#if defined(PIOS_INCLUDE_OPENLRS_RCVR)
#include "pios_openlrs.h"
#endif /* PIOS_INCLUDE_OPENLRS_RCVR */

#if defined(PIOS_INCLUDE_FRSKY_RSSI)
#include "pios_frsky_rssi.h"
#endif /* PIOS_INCLUDE_FRSKY_RSSI */

// This is how far "left" you have to deflect for "yaw left" arming, etc.
#define ARMED_THRESHOLD    0.50f

//safe band to allow a bit of calibration error or trim offset (in microseconds)
//these specify how far outside the "permitted range" throttle and other channels
//can go.  e.g. if range is 1000..2000us, values under 750us or over 2250us
//result in failsafe.
#define CONNECTION_OFFSET          250

#define RCVR_ACTIVITY_MONITOR_CHANNELS_PER_GROUP 12
#define RCVR_ACTIVITY_MONITOR_MIN_RANGE 20

/* All channels must have at least this many counts on each side of neutral.
 * (Except throttle which must have this many on the positive side).  This is
 * to prevent situations where a partial calibration results in spurious
 * arming, etc. */
#define MIN_MEANINGFUL_RANGE 40

struct rcvr_activity_fsm {
	ManualControlSettingsChannelGroupsOptions group;
	uint16_t prev[RCVR_ACTIVITY_MONITOR_CHANNELS_PER_GROUP];
	uint8_t check_count;
};

extern uintptr_t pios_rcvr_group_map[];

// Private variables
static CarManualControlCommandData   cmd;
static CarManualControlSettingsData  settings;

static uint8_t                    disconnected_count = 0;
static uint8_t                    connected_count = 0;
static struct rcvr_activity_fsm   activity_fsm;
static uint32_t                   lastActivityTime;
static uint32_t                   lastSysTime;
static float                      driving_mode_value;
static enum control_status        control_status;
static bool                       settings_updated;

static struct pios_can_cmd_data cmd_from_can;

// Private functions
static void update_manual_desired(CarManualControlCommandData * cmd);
static void update_navigation_desired(CarManualControlCommandData * cmd);
static void update_emergency_desired(CarManualControlCommandData * cmd);

static void set_driving_mode();
static void process_transmitter_events(CarManualControlCommandData * cmd, CarManualControlSettingsData * settings, bool valid);

static float scaleChannel(int n, int16_t value);
static bool validInputRange(int n, uint16_t value, uint16_t offset);
static uint32_t timeDifferenceMs(uint32_t start_time, uint32_t end_time);
static void applyDeadband(float *value, float deadband);

static void resetRcvrActivity(struct rcvr_activity_fsm * fsm);
static bool updateRcvrActivity(struct rcvr_activity_fsm * fsm);

//! Convert a rssi type to the associated channel group.
int rssitype_to_channelgroup() {
	switch (settings.RssiType) {
		case MANUALCONTROLSETTINGS_RSSITYPE_PWM:
			return MANUALCONTROLSETTINGS_CHANNELGROUPS_PWM;
		case MANUALCONTROLSETTINGS_RSSITYPE_PPM:
			return MANUALCONTROLSETTINGS_CHANNELGROUPS_PPM;
		case MANUALCONTROLSETTINGS_RSSITYPE_SBUS:
			return MANUALCONTROLSETTINGS_CHANNELGROUPS_SBUS;
		case MANUALCONTROLSETTINGS_RSSITYPE_HOTTSUM:
			return MANUALCONTROLSETTINGS_CHANNELGROUPS_HOTTSUM;
		case MANUALCONTROLSETTINGS_RSSITYPE_TBSCROSSFIRE:
			return MANUALCONTROLSETTINGS_CHANNELGROUPS_TBSCROSSFIRE;
		default:
			return -1;
	}
}

//! Initialize the transmitter control mode
int32_t transmitter_control_initialize()
{
	if (CarManualControlCommandInitialize() == -1
			|| CarManualControlSettingsInitialize() == -1
			|| DrivingStatusInitialize() == -1 
			|| ReceiverActivityInitialize() == -1) {
		return -1;
	}

	control_status = STATUS_ERROR;

	/* Initialize the RcvrActivty FSM */
	lastActivityTime = PIOS_Thread_Systime();
	resetRcvrActivity(&activity_fsm);

	// Use callback to update the settings when they change
	CarManualControlSettingsConnectCallbackCtx(UAVObjCbSetFlag, &settings_updated);

	settings_updated = true;

	// Main task loop
	lastSysTime = PIOS_Thread_Systime();
	return 0;
}

/**
  * @brief Process inputs and arming
  *
  * This will always process the arming signals as for now the transmitter
  * is always in charge.  When a transmitter is not detected control will
  * fall back to the failsafe module.  If the driving mode is in tablet
  * control position then control will be ceeded to that module.
  */
int32_t transmitter_control_update()
{
	lastSysTime = PIOS_Thread_Systime();

	float scaledChannel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_NUMELEM];
	bool validChannel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_NUMELEM];

	if (settings_updated) {
		settings_updated = false;
		CarManualControlSettingsGet(&settings);
	}

	/* Update channel activity monitor */
	uint8_t arm_status;
	DrivingStatusArmedGet(&arm_status);

	if (arm_status == DRIVINGSTATUS_ARMED_DISARMED) {
		if (updateRcvrActivity(&activity_fsm)) {
			/* Reset the aging timer because activity was detected */
			lastActivityTime = lastSysTime;
		}
	}

	if (timeDifferenceMs(lastActivityTime, lastSysTime) > 5000) {
		resetRcvrActivity(&activity_fsm);
		lastActivityTime = lastSysTime;
	}

	if (settings.RssiType != CARMANUALCONTROLSETTINGS_RSSITYPE_NONE) {
		int32_t value = 0;

		switch (settings.RssiType) {
		case CARMANUALCONTROLSETTINGS_RSSITYPE_ADC:
#if defined(PIOS_INCLUDE_ADC)
			if (settings.RssiChannelNumber > 0) {
				value = PIOS_ADC_GetChannelRaw(settings.RssiChannelNumber - 1);
			} else {
				value = 0;
			}
#endif
			break;
		case CARMANUALCONTROLSETTINGS_RSSITYPE_OPENLRS:
#if defined(PIOS_INCLUDE_OPENLRS_RCVR)
			value = PIOS_OpenLRS_RSSI_Get();
#endif /* PIOS_INCLUDE_OPENLRS_RCVR */
			break;
		case CARMANUALCONTROLSETTINGS_RSSITYPE_FRSKYPWM:
#if defined(PIOS_INCLUDE_FRSKY_RSSI)
			value = PIOS_FrSkyRssi_Get();
#endif /* PIOS_INCLUDE_FRSKY_RSSI */
			break;
		case CARMANUALCONTROLSETTINGS_RSSITYPE_RFM22B:
#if defined(PIOS_INCLUDE_RFM22B)
			value = PIOS_RFM22B_RSSI_Get();
#endif /* PIOS_INCLUDE_RFM22B */
			break;
		default:
			(void) 0 ;
			int mapped = rssitype_to_channelgroup();

			if (mapped >= 0) {
				value = PIOS_RCVR_Read(
						pios_rcvr_group_map[mapped],
						settings.RssiChannelNumber);
			}

			break;
		}

		if (value < settings.RssiMin) {
			cmd.Rssi = 0;
		} else if (settings.RssiMax == settings.RssiMin) {
			cmd.Rssi = 0;
		} else if (value > settings.RssiMax) {
			if (value > (settings.RssiMax + settings.RssiMax/4)) {
				cmd.Rssi = 0;
			} else {
				cmd.Rssi = 100;
			}
		} else {
			cmd.Rssi = ((float)(value - settings.RssiMin)/((float)settings.RssiMax-settings.RssiMin)) * 100;
		}

		cmd.RawRssi = value;
	}

	volatile bool valid_input_detected = true;

	// for(uint8_t n = 0; n < CARMANUALCONTROLCOMMAND_CHANNEL_NUMELEM; ++n)
	// 	settings.ChannelNumber[n] = n + 1;
	
	settings.ChannelNumber[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_THROTTLE] = CARMANUALCONTROLSETTINGS_CHANNELGROUPS_THROTTLE + 1;
	settings.ChannelNumber[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_ROLL] = CARMANUALCONTROLSETTINGS_CHANNELGROUPS_ROLL + 1;
	settings.ChannelNumber[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_PITCH] = CARMANUALCONTROLSETTINGS_CHANNELGROUPS_PITCH + 1;
	settings.ChannelNumber[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_YAW] = CARMANUALCONTROLSETTINGS_CHANNELGROUPS_YAW + 1;
	settings.ChannelNumber[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_DRIVINGMODE] = CARMANUALCONTROLSETTINGS_CHANNELGROUPS_DRIVINGMODE + 1;	
	settings.ChannelNumber[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_COLLECTIVE] = CARMANUALCONTROLSETTINGS_CHANNELGROUPS_COLLECTIVE + 1;
	settings.ChannelNumber[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_ACCESSORY0] = CARMANUALCONTROLSETTINGS_CHANNELGROUPS_ACCESSORY0 + 1;
	settings.ChannelNumber[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_ARMING] = CARMANUALCONTROLSETTINGS_CHANNELGROUPS_ARMING + 1;

	// Read channel values in us
	for (volatile uint8_t n = 0;
	     n < CARMANUALCONTROLSETTINGS_CHANNELGROUPS_NUMELEM && n < CARMANUALCONTROLCOMMAND_CHANNEL_NUMELEM;
	     ++n) {

		if (settings.ChannelGroups[n] >= CARMANUALCONTROLSETTINGS_CHANNELGROUPS_NONE) {
			cmd.Channel[n] = PIOS_RCVR_INVALID;
			validChannel[n] = false;
		} else {
			cmd.Channel[n] = PIOS_RCVR_Read(pios_rcvr_group_map[settings.ChannelGroups[n]],
							settings.ChannelNumber[n]);
		}

		// If a channel has timed out this is not valid data and we shouldn't update anything
		// until we decide to go to failsafe
		if(cmd.Channel[n] == (uint16_t) PIOS_RCVR_TIMEOUT) {
			valid_input_detected = false;
			validChannel[n] = false;
		} 
		else {
			scaledChannel[n] = scaleChannel(n, cmd.Channel[n]);
			validChannel[n] = validInputRange(n, cmd.Channel[n], CONNECTION_OFFSET);
		}
	}

	// JLinkRTTPrintf(0, "Received PPM signal [THR]-[ROL]-[PIT]-[YAW]-[MOD]-[ARM]: %ld, %ld, %ld, %ld, %ld, %ld\n", 
	// cmd.Channel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_THROTTLE], 
	// cmd.Channel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_ROLL],
	// cmd.Channel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_PITCH],
	// cmd.Channel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_YAW],
	// cmd.Channel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_DRIVINGMODE],
	// cmd.Channel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_ARMING]);
	// JLinkRTTPrintf(0, "Received PPM signal [1]-[8]: %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld\n", cmd.Channel[0], cmd.Channel[1],
	// 	cmd.Channel[2],cmd.Channel[3],cmd.Channel[4],cmd.Channel[5],cmd.Channel[6], cmd.Channel[7]);
	// JLinkRTTPrintf(0, "Scaled PPM signal [1]-[8]: %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld\n", scaledChannel[0], scaledChannel[1],
	// 	scaledChannel[2],scaledChannel[3],scaledChannel[4],scaledChannel[5],scaledChannel[6], scaledChannel[7]);

	// Check settings, if error raise alarm
	if (settings.ChannelGroups[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_ROLL] >= CARMANUALCONTROLSETTINGS_CHANNELGROUPS_NONE ||
		settings.ChannelGroups[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_PITCH] >= CARMANUALCONTROLSETTINGS_CHANNELGROUPS_NONE ||
		settings.ChannelGroups[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_YAW] >= CARMANUALCONTROLSETTINGS_CHANNELGROUPS_NONE ||
		settings.ChannelGroups[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_THROTTLE] >= CARMANUALCONTROLSETTINGS_CHANNELGROUPS_NONE ||
		// Check all channel mappings are valid
		cmd.Channel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_ROLL] == (uint16_t) PIOS_RCVR_INVALID ||
		cmd.Channel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_PITCH] == (uint16_t) PIOS_RCVR_INVALID ||
		cmd.Channel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_YAW] == (uint16_t) PIOS_RCVR_INVALID ||
		cmd.Channel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_THROTTLE] == (uint16_t) PIOS_RCVR_INVALID ||
		// Check the driver exists
		cmd.Channel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_ROLL] == (uint16_t) PIOS_RCVR_NODRIVER ||
		cmd.Channel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_PITCH] == (uint16_t) PIOS_RCVR_NODRIVER ||
		cmd.Channel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_YAW] == (uint16_t) PIOS_RCVR_NODRIVER ||
		cmd.Channel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_THROTTLE] == (uint16_t) PIOS_RCVR_NODRIVER ||
		// Check the DrivingModeNumber is valid
		settings.DrivingModeNumber < 1 || settings.DrivingModeNumber > CARMANUALCONTROLSETTINGS_DRIVINGMODEPOSITION_NUMELEM ||
		// If we've got more than one possible valid DrivingMode, we require a configured DrivingMode channel
		((settings.DrivingModeNumber > 1) && (settings.ChannelGroups[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_DRIVINGMODE] >= CARMANUALCONTROLSETTINGS_CHANNELGROUPS_NONE)) ||
		// Whenever DrivingMode channel is configured, it needs to be valid regardless of DrivingModeNumber settings
		((settings.ChannelGroups[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_DRIVINGMODE] < CARMANUALCONTROLSETTINGS_CHANNELGROUPS_NONE) && (
			cmd.Channel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_DRIVINGMODE] == (uint16_t) PIOS_RCVR_INVALID ||
			cmd.Channel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_DRIVINGMODE] == (uint16_t) PIOS_RCVR_NODRIVER ))) {

		// set_manual_control_error(SYSTEMALARMS_MANUALCONTROL_SETTINGS);

		cmd.Connected = CARMANUALCONTROLCOMMAND_CONNECTED_FALSE;
		CarManualControlCommandSet(&cmd);

		control_status = STATUS_ERROR;

		return -1;
	}

	// the block above validates the input configuration. this simply checks that the range is valid if driving mode is configured.
	bool drivingmode_valid_input = settings.ChannelGroups[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_DRIVINGMODE] >= CARMANUALCONTROLSETTINGS_CHANNELGROUPS_NONE ||
	    validChannel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_DRIVINGMODE];

	// because arming is optional we must determine if it is needed before checking it is valid
	bool arming_valid_input = (settings.Arming < CARMANUALCONTROLSETTINGS_ARMING_SWITCH) ||
		validChannel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_ARMING];

	// decide if we have valid manual input or not
	valid_input_detected &= validChannel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_THROTTLE] &&
	    validChannel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_ROLL] &&
	    validChannel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_YAW] &&
	    validChannel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_PITCH] &&
	    drivingmode_valid_input &&
		arming_valid_input;
		
	// JLinkRTTPrintf(0, "valid_input_detected: %ld ; drivingmode_valid_input: %ld ; arming_valid_input: %ld\n", valid_input_detected, drivingmode_valid_input, arming_valid_input);
	// JLinkRTTPrintf(0, "Valid channel [1]-[8]: %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld\n", validChannel[0], validChannel[1],
	// validChannel[2],validChannel[3],validChannel[4],validChannel[5],validChannel[6], validChannel[7]);

	// Implement hysteresis loop on connection status
	if (valid_input_detected && (++connected_count > 10)) {
		cmd.Connected = CARMANUALCONTROLCOMMAND_CONNECTED_TRUE;
		connected_count = 0;
		disconnected_count = 0;
	} else if (!valid_input_detected && (++disconnected_count > 10)) {
		cmd.Connected = CARMANUALCONTROLCOMMAND_CONNECTED_FALSE;
		connected_count = 0;
		disconnected_count = 0;
	}

	if (cmd.Connected == CARMANUALCONTROLCOMMAND_CONNECTED_FALSE) {
		// These values are not used but just put ManualControlCommand in a sane state.  When
		// Connected is false, then the failsafe submodule will be in control.

		cmd.Throttle = 0;
		cmd.Roll = 0;
		cmd.Yaw = 0;
		cmd.Pitch = 0;
		cmd.Collective = 0;

		/* If all relevant channel groups aren't in error mode, then the receiver seems to be working fine.
		   So if we still reach this point, the channel min/max/neutral values on the input page are faulty. */
		bool rx_signal_detected = cmd.Channel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_THROTTLE] < (uint16_t)PIOS_RCVR_NODRIVER &&
			cmd.Channel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_ROLL] < (uint16_t)PIOS_RCVR_NODRIVER &&
			cmd.Channel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_PITCH] < (uint16_t)PIOS_RCVR_NODRIVER &&
			cmd.Channel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_YAW] < (uint16_t)PIOS_RCVR_NODRIVER &&
			cmd.Channel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_ARMING] < (uint16_t)PIOS_RCVR_NODRIVER &&
			cmd.Channel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_DRIVINGMODE] < (uint16_t)PIOS_RCVR_NODRIVER;

		(void)rx_signal_detected;

		// if (rx_signal_detected)
		// 	set_manual_control_error(SYSTEMALARMS_MANUALCONTROL_CHANNELCONFIGURATION);
		// else
		// 	set_manual_control_error(SYSTEMALARMS_MANUALCONTROL_NORX);

	} else if (valid_input_detected) {
		// set_manual_control_error(SYSTEMALARMS_MANUALCONTROL_NONE);

		// Scale channels to -1 -> +1 range
		cmd.Roll           = scaledChannel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_ROLL];
		cmd.Pitch          = scaledChannel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_PITCH];
		cmd.Yaw            = scaledChannel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_YAW];
		cmd.Throttle       = scaledChannel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_THROTTLE];
		cmd.ArmSwitch      = scaledChannel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_ARMING] > 0 ?
							 CARMANUALCONTROLCOMMAND_ARMSWITCH_ARMED : CARMANUALCONTROLCOMMAND_ARMSWITCH_DISARMED;

		driving_mode_value  = scaledChannel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_DRIVINGMODE];

		// Apply deadband for Roll/Pitch/Yaw stick inputs
		if (settings.Deadband) {
			applyDeadband(&cmd.Roll, settings.Deadband);
			applyDeadband(&cmd.Pitch, settings.Deadband);
			applyDeadband(&cmd.Yaw, settings.Deadband);
		}

		if(cmd.Channel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_COLLECTIVE] != (uint16_t) PIOS_RCVR_INVALID &&
		   cmd.Channel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_COLLECTIVE] != (uint16_t) PIOS_RCVR_NODRIVER &&
		   cmd.Channel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_COLLECTIVE] != (uint16_t) PIOS_RCVR_TIMEOUT) {
			cmd.Collective = scaledChannel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_COLLECTIVE];
		}

		// Set Accessory 0
		if (settings.ChannelGroups[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_ACCESSORY0] !=
			CARMANUALCONTROLSETTINGS_CHANNELGROUPS_NONE) {
			cmd.Accessory[0] = scaledChannel[CARMANUALCONTROLSETTINGS_CHANNELGROUPS_ACCESSORY0];
		}
	}

	// Process arming outside conditional so system will disarm when disconnected.  Notice this
	// is processed in the _update method instead of _select method so the state system is always
	// evalulated, even if not detected.
	process_transmitter_events(&cmd, &settings, valid_input_detected);

	// Update cmd object
	CarManualControlCommandSet(&cmd);

	return 0;
}

/**
 * Select and use transmitter control
 * @param [in] reset_controller True if previously another controller was used
 */
int32_t transmitter_control_select(bool reset_controller)
{
	// Activate the driving mode corresponding to the switch position
	set_driving_mode();

	CarManualControlCommandGet(&cmd);

	uint8_t drivingMode;
	DrivingStatusDrivingModeGet(&drivingMode);

	// Depending on the mode update the Navigation object
	static uint8_t lastDrivingMode = DRIVINGSTATUS_DRIVINGMODE_MANUAL;

	switch(drivingMode) {
	case DRIVINGSTATUS_DRIVINGMODE_MANUAL:
		AutoCarResetNavigationDesired();
		update_manual_desired(&cmd);
		break;
	case DRIVINGSTATUS_DRIVINGMODE_NAVIGATION:
		update_navigation_desired(&cmd);
		break;
	case DRIVINGSTATUS_DRIVINGMODE_EMERGENCY:
		AutoCarResetNavigationDesired();
		update_emergency_desired(&cmd);
		break;
	default:
		// set_manual_control_error(SYSTEMALARMS_MANUALCONTROL_UNDEFINED);
		return -1;
	}
	lastDrivingMode = drivingMode;
	(void)lastDrivingMode;

	return 0;
}

//! Get the control & arming status
enum control_status transmitter_control_get_status()
{
	return control_status;
}

//! Determine which of N positions the driving mode switch is in but do not set it
uint8_t transmitter_control_get_driving_mode()
{
	// Convert drivingMode value into the switch position in the range [0..N-1]
	uint8_t pos = ((int16_t)(driving_mode_value * 256.0f) + 256) * settings.DrivingModeNumber >> 9;
	if (pos >= settings.DrivingModeNumber)
		pos = settings.DrivingModeNumber - 1;

	return settings.DrivingModePosition[pos];
}

/**
 * Check sticks to determine if they are in the arming position
 */
static bool arming_position(CarManualControlCommandData * cmd, CarManualControlSettingsData * settings) {

	// If system is not appropriate to arm, do not even attempt
	// if (!ok_to_arm())
	// 	return false;

	bool lowThrottle = cmd->Throttle <= 0;

	switch(settings->Arming) {
	case CARMANUALCONTROLSETTINGS_ARMING_ALWAYSDISARMED:
		return false;
	case CARMANUALCONTROLSETTINGS_ARMING_ROLLLEFTTHROTTLE:
		return lowThrottle && cmd->Roll < -ARMED_THRESHOLD;
	case CARMANUALCONTROLSETTINGS_ARMING_ROLLRIGHTTHROTTLE:
		return lowThrottle && cmd->Roll > ARMED_THRESHOLD;
	case CARMANUALCONTROLSETTINGS_ARMING_YAWLEFTTHROTTLE:
		return lowThrottle && cmd->Yaw < -ARMED_THRESHOLD;
	case CARMANUALCONTROLSETTINGS_ARMING_YAWRIGHTTHROTTLE:
		return lowThrottle && cmd->Yaw > ARMED_THRESHOLD;
	case CARMANUALCONTROLSETTINGS_ARMING_CORNERSTHROTTLE:
		return lowThrottle && (
			(cmd->Yaw > ARMED_THRESHOLD || cmd->Yaw < -ARMED_THRESHOLD) &&
			(cmd->Roll > ARMED_THRESHOLD || cmd->Roll < -ARMED_THRESHOLD) ) &&
			(cmd->Pitch > ARMED_THRESHOLD);
			// Note that pulling pitch stick down corresponds to positive values
	case CARMANUALCONTROLSETTINGS_ARMING_SWITCHTHROTTLE:
		if (!lowThrottle) return false;
	case CARMANUALCONTROLSETTINGS_ARMING_SWITCH:
		return cmd->ArmSwitch == CARMANUALCONTROLCOMMAND_ARMSWITCH_ARMED;
	default:
		return false;
	}
}

/**
 * Check sticks to determine if they are in the disarmed position
 */
static bool disarming_position(CarManualControlCommandData * cmd, CarManualControlSettingsData * settings)
{
	bool lowThrottle = cmd->Throttle <= 0;

	switch(settings->Arming) {
	case CARMANUALCONTROLSETTINGS_ARMING_ALWAYSDISARMED:
		return true;
	case CARMANUALCONTROLSETTINGS_ARMING_ROLLLEFTTHROTTLE:
		return lowThrottle && cmd->Roll > ARMED_THRESHOLD;
	case CARMANUALCONTROLSETTINGS_ARMING_ROLLRIGHTTHROTTLE:
		return lowThrottle && cmd->Roll < -ARMED_THRESHOLD;
	case CARMANUALCONTROLSETTINGS_ARMING_YAWLEFTTHROTTLE:
		return lowThrottle && cmd->Yaw > ARMED_THRESHOLD;
	case CARMANUALCONTROLSETTINGS_ARMING_YAWRIGHTTHROTTLE:
		return lowThrottle && cmd->Yaw < -ARMED_THRESHOLD;
	case CARMANUALCONTROLSETTINGS_ARMING_CORNERSTHROTTLE:
		return lowThrottle && (
			(cmd->Yaw > ARMED_THRESHOLD || cmd->Yaw < -ARMED_THRESHOLD) &&
			(cmd->Roll > ARMED_THRESHOLD || cmd->Roll < -ARMED_THRESHOLD) );
	case CARMANUALCONTROLSETTINGS_ARMING_SWITCH:
	case CARMANUALCONTROLSETTINGS_ARMING_SWITCHTHROTTLE:
		return cmd->ArmSwitch != CARMANUALCONTROLCOMMAND_ARMSWITCH_ARMED;
	default:
		return false;
	}
}

static bool disarm_commanded(CarManualControlCommandData *cmd,
		CarManualControlSettingsData *settings)
{
	static uint32_t start_time;

	if (!disarming_position(cmd, settings)) {
		start_time = 0;
		return false;
	}

	uint32_t now = PIOS_Thread_Systime();

	if (!start_time) {
		start_time = now;
		return false;
	}

	uint16_t disarm_time = 100;

	if ((settings->Arming != MANUALCONTROLSETTINGS_ARMING_SWITCH) &&
			(settings->Arming != MANUALCONTROLSETTINGS_ARMING_SWITCHTHROTTLE)) {
		disarm_time = (settings->DisarmTime == MANUALCONTROLSETTINGS_DISARMTIME_250) ? 250 :
			(settings->DisarmTime == MANUALCONTROLSETTINGS_DISARMTIME_500) ? 500 :
			(settings->DisarmTime == MANUALCONTROLSETTINGS_DISARMTIME_1000) ? 1000 :
			(settings->DisarmTime == MANUALCONTROLSETTINGS_DISARMTIME_2000) ? 2000 : 1000;
	}

	return (now - start_time) >= disarm_time;
}

#define RECEIVER_TIMER_FIRED 0xffffffff

static uint32_t receiver_timer = RECEIVER_TIMER_FIRED;

static void reset_receiver_timer() {
	receiver_timer = 0;
}

static bool check_receiver_timer(uint32_t threshold) {
	if (!threshold) {
		/* 0 threshold means timer disabled */

		reset_receiver_timer();

		return false;
	}

	if (receiver_timer == RECEIVER_TIMER_FIRED) {
		/* It already went off!  We just want a strobe. */
		return false;
	}

	uint32_t now = PIOS_Thread_Systime();

	if (receiver_timer) {
		if ((now - receiver_timer) >= threshold) {
			receiver_timer = RECEIVER_TIMER_FIRED;
			return true;
		}

		return false;
	}

	receiver_timer = now;

	return false;
}

/**
 * @brief Process the inputs and determine whether to arm or not
 * @param[in] cmd The manual control inputs
 * @param[in] settings Settings indicating the necessary position
 * @param[in] scaled The scaled channels, used for checking arming
 * @param[in] valid If the input data is valid (i.e. transmitter is transmitting)
 */
static void process_transmitter_events(CarManualControlCommandData * cmd, CarManualControlSettingsData * settings, bool valid)
{
	valid &= cmd->Connected == CARMANUALCONTROLCOMMAND_CONNECTED_TRUE;

	if (!valid || (cmd->Connected != CARMANUALCONTROLCOMMAND_CONNECTED_TRUE)) {
		/* disarm timeout check-- use least favorable timeout.
		 * Note that if things were low throttle, and are now
		 * disconnected, we keep the old timer running.
		 *
		 * (The same is true for vice-versa).
		 */
		uint32_t timeout = 0;

		if (settings->InvalidRXArmedTimeout && settings->ArmedTimeout) {
			timeout = MIN(settings->InvalidRXArmedTimeout,
				settings->ArmedTimeout);
		} else if (settings->InvalidRXArmedTimeout) {
			timeout = settings->InvalidRXArmedTimeout;
		} else {
			timeout = settings->ArmedTimeout;
		}

		if (check_receiver_timer(timeout)) {
			control_status = STATUS_SAFETYTIMEOUT;
		} else {
			control_status = STATUS_DISCONNECTED;
		}

		return;
	}

	// Never disarm due to low throttle
	bool low_throt = false;
	if (low_throt) {
		// Currently this condition will never be met
		// It could be recovered in the future
		if (check_receiver_timer(settings->ArmedTimeout)) {
			control_status = STATUS_DISARM;
			return;
		}
	} else {
		/* Not low throt, not disconnected. */
		reset_receiver_timer();
	}

	if (disarm_commanded(cmd, settings)) {
		/* Separate timer from the above */
		control_status = STATUS_DISARM;
		return;
	} else if (arming_position(cmd, settings)) {
		/* Not disarming pos, not low throt timeout, not disconnected. */

		if (control_status == STATUS_DISARM) {
			/* Edge detector says it's time to reset the receiver
			 * timer
			 */
			reset_receiver_timer();
		}

		if (settings->Arming == CARMANUALCONTROLSETTINGS_ARMING_SWITCH ||
				settings->Arming == CARMANUALCONTROLSETTINGS_ARMING_SWITCHTHROTTLE) {
			control_status = STATUS_ARM_VALID;
		} else {
			control_status = STATUS_ARM_INVALID;
		}

		return;
	} else {
		/* If the arming switch is flipped on, but we don't otherwise
		 * qualify for arming, let the upper layer know.
		 */
		if (settings->Arming == CARMANUALCONTROLSETTINGS_ARMING_SWITCHTHROTTLE) {
			if (cmd->ArmSwitch == CARMANUALCONTROLCOMMAND_ARMSWITCH_ARMED) {
				control_status = STATUS_INVALID_FOR_DISARMED;
				return;
			}
		}
	}

	control_status = STATUS_NORMAL;

	return;

}


//! Determine which of N positions the driving mode switch is in and set driving mode accordingly
static void set_driving_mode()
{
	uint8_t new_mode = transmitter_control_get_driving_mode();

	DrivingStatusDrivingModeOptions cur_mode;

	DrivingStatusDrivingModeGet(&cur_mode);

	if (cur_mode != new_mode) {
		DrivingStatusDrivingModeSet(&new_mode);
	}
}


static void resetRcvrActivity(struct rcvr_activity_fsm * fsm)
{
	ReceiverActivityData data;
	bool updated = false;

	/* Clear all channel activity flags */
	ReceiverActivityGet(&data);
	if (data.ActiveGroup != RECEIVERACTIVITY_ACTIVEGROUP_NONE &&
		data.ActiveChannel != 255) {
		data.ActiveGroup = RECEIVERACTIVITY_ACTIVEGROUP_NONE;
		data.ActiveChannel = 255;
		updated = true;
	}
	if (updated) {
		ReceiverActivitySet(&data);
	}

	/* Reset the FSM state */
	fsm->group        = 0;
	fsm->check_count = 0;
}

static void updateRcvrActivitySample(uintptr_t rcvr_id, uint16_t samples[], uint8_t max_channels) {
	for (uint8_t channel = 1; channel <= max_channels; channel++) {
		// Subtract 1 because channels are 1 indexed
		samples[channel - 1] = PIOS_RCVR_Read(rcvr_id, channel);
	}
}

static bool updateRcvrActivityCompare(uintptr_t rcvr_id, struct rcvr_activity_fsm * fsm)
{
	uint8_t active_channel = 0;

	int threshold = RCVR_ACTIVITY_MONITOR_MIN_RANGE;

	int rssi_channel = 0;

	/* Start off figuring out if there's a receive-rssi-injection channel */
	int rssi_group = rssitype_to_channelgroup();

	/* If so, see if it's what we're processing */
	if ((rssi_group >= 0) && (pios_rcvr_group_map[rssi_group] == rcvr_id)) {
		// Yup.  So let's skip the configured RSSI channel
		rssi_channel = settings.RssiChannelNumber;
	}

	/* Compare the current value to the previous sampled value */
	for (uint8_t channel = 1;
	     channel <= RCVR_ACTIVITY_MONITOR_CHANNELS_PER_GROUP;
	     channel++) {
		if (rssi_channel == channel) {
			// Don't spot activity on the configured RSSI channel
			continue;
		}

		uint16_t delta;
		uint16_t prev = fsm->prev[channel - 1];   // Subtract 1 because channels are 1 indexed
		uint16_t curr = PIOS_RCVR_Read(rcvr_id, channel);
		if (curr > prev) {
			delta = curr - prev;
		} else {
			delta = prev - curr;
		}

		if (delta > threshold) {
			active_channel = channel;

			// This is the best channel so far.  Only look for
			// "more active" channels now.
			threshold = delta;
		}
	}

	if (active_channel) {
		/* Mark this channel as active */
		ReceiverActivityActiveGroupSet((uint8_t *)&fsm->group);
		ReceiverActivityActiveChannelSet(&active_channel);

		return true;
	}

	return false;
}

static bool updateRcvrActivity(struct rcvr_activity_fsm * fsm)
{
	bool activity_updated = false;

	if (fsm->group >= MANUALCONTROLSETTINGS_CHANNELGROUPS_NONE) {
		/* We're out of range, reset things */
		resetRcvrActivity(fsm);
	}

	if (fsm->check_count && (pios_rcvr_group_map[fsm->group])) {
		/* Compare with previous sample */
		activity_updated = updateRcvrActivityCompare(pios_rcvr_group_map[fsm->group], fsm);
	}

	if (!activity_updated && fsm->check_count < 3) {
		// First time through this is 1, so basically be willing
		// to check twice.
		fsm->check_count++;

		return false;
	}

	/* Reset the sample counter */
	fsm->check_count = 0;

	/* Find the next active group, but limit search so we can't loop forever here */
	for (uint8_t i = 0; i <= MANUALCONTROLSETTINGS_CHANNELGROUPS_NONE; i++) {
		/* Move to the next group */
		fsm->group++;
		if (fsm->group >= MANUALCONTROLSETTINGS_CHANNELGROUPS_NONE) {
			/* Wrap back to the first group */
			fsm->group = 0;
		}
		if (pios_rcvr_group_map[fsm->group]) {
			/*
			 * Found an active group, take a sample here to avoid an
			 * extra 20ms delay in the main thread so we can speed up
			 * this algorithm.
			 */
			updateRcvrActivitySample(pios_rcvr_group_map[fsm->group],
						fsm->prev,
						NELEMENTS(fsm->prev));

			fsm->check_count = 1;

			break;
		}
	}

	return (activity_updated);
}

//! In manual mode directly set actuator desired
static void update_manual_desired(CarManualControlCommandData * cmd)
{
	CarActuatorDesiredData actuator;
	CarActuatorDesiredGet(&actuator);
	actuator.Roll = cmd->Roll;
	actuator.Pitch = cmd->Pitch;
	actuator.Yaw = cmd->Yaw;
	actuator.Steering = cmd->Roll;
	actuator.Throttle = cmd->Pitch;

	JLinkRTTPrintf(0, "Updating manual desired: %d, %d\n",(int32_t)(actuator.Steering*100), (int32_t)(actuator.Throttle*100));

	CarActuatorDesiredSet(&actuator);
}

//! In navigation mode, set navigation desired
static void update_navigation_desired(CarManualControlCommandData * cmd)
{
	AutoCarGetNavigationDesired(&cmd_from_can);

	CarActuatorDesiredData actuator;
	CarActuatorDesiredGet(&actuator);
	actuator.Roll = cmd->Roll;
	actuator.Pitch = cmd->Pitch;
	actuator.Yaw = cmd->Yaw;
	actuator.Steering = cmd_from_can.steering;
	actuator.Throttle = cmd_from_can.throttle;

	if(actuator.Steering != 0 && actuator.Throttle != 0)
		JLinkRTTPrintf(0, "Updating navigation desired: %d, %d\n",(int32_t)(actuator.Steering*100), (int32_t)(actuator.Throttle*100));

	CarActuatorDesiredSet(&actuator);
}

static void update_emergency_desired(CarManualControlCommandData * cmd)
{
	CarActuatorDesiredData actuator;
	CarActuatorDesiredGet(&actuator);
	actuator.Roll = cmd->Roll;
	actuator.Pitch = cmd->Pitch;
	actuator.Yaw = cmd->Yaw;
	actuator.Steering = 0;
	actuator.Throttle = 0;

	CarActuatorDesiredSet(&actuator);
}

/**
 * Convert channel from servo pulse duration (microseconds) to scaled -1/+1 range.
 */
static float scaleChannel(int n, int16_t value)
{
	int16_t min = settings.ChannelMin[n];
	int16_t max = settings.ChannelMax[n];
	int16_t neutral = settings.ChannelNeutral[n];

	float valueScaled;

	// Scale
	if ((max > min && value >= neutral) || (min > max && value <= neutral))
	{
		if (max != neutral)
			valueScaled = (float)(value - neutral) / (float)(max - neutral);
		else
			valueScaled = 0;
	}
	else
	{
		if (min != neutral)
			valueScaled = (float)(value - neutral) / (float)(neutral - min);
		else
			valueScaled = 0;
	}

	// Bound
	if (valueScaled >  1.0f) valueScaled =  1.0f;
	else
	if (valueScaled < -1.0f) valueScaled = -1.0f;

	return valueScaled;
}

static uint32_t timeDifferenceMs(uint32_t start_time, uint32_t end_time) {
	if (end_time >= start_time)
		return end_time - start_time;
	else
		return (uint32_t)PIOS_THREAD_TIMEOUT_MAX - start_time + end_time + 1;
}

/**
 * @brief Determine if the manual input value is within acceptable limits
 * @returns return TRUE if so, otherwise return FALSE
 */
bool validInputRange(int n, uint16_t value, uint16_t offset)
{
	int16_t min = settings.ChannelMin[n];
	int16_t max = settings.ChannelMax[n];
	int16_t neutral = settings.ChannelNeutral[n];
	int16_t range;
	bool inverted = false;

	if (min > max) {
		int16_t tmp = min;
		min = max;
		max = tmp;

		inverted = true;
	}

	if ((neutral > max) || (neutral < min)) {
		return false;
	}

	if (n == MANUALCONTROLSETTINGS_CHANNELGROUPS_THROTTLE) {
		/* Pick just the one that results in the "positive" side of
		 * the throttle scale */
		if (inverted) {
			range = neutral - min;
		} else {
			range = max - neutral;
		}
	} else {
		range = MIN(max - neutral, neutral - min);
	}

	if (range < MIN_MEANINGFUL_RANGE) {
		return false;
	}

	return (value >= (min - offset) && value <= (max + offset));
}

/**
 * @brief Apply deadband to Roll/Pitch/Yaw channels
 */
static void applyDeadband(float *value, float deadband)
{
	if (deadband < 0.0001f) return; /* ignore tiny deadband value */
	if (deadband >= 0.85f) return;	/* reject nonsensical db values */

	if (fabsf(*value) < deadband) {
		*value = 0.0f;
	} else {
		if (*value > 0.0f) {
			*value -= deadband;
		} else {
			*value += deadband;
		}

		*value /= (1 - deadband);
	}
}

/**
  * @}
  * @}
  */

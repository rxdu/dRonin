/**
 ******************************************************************************
 * @addtogroup Modules Modules
 * @{
 * @addtogroup ActuatorModule Actuator Module
 * @{
 *
 * @file       actuator.c
 * @author     dRonin, http://dRonin.org/, Copyright (C) 2015-2016
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013-2016
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Actuator module. Drives the actuators (servos, motors etc).
 * @brief      Take the values in @ref ActuatorDesired and mix to set the outputs
 *
 * This module ultimately controls the outputs.  The values from @ref ActuatorDesired
 * are combined based on the values in @ref MixerSettings and then scaled by the
 * values in @ref ActuatorSettings to create the output PWM times.
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

#include <math.h>

#include "openpilot.h"
#include "pios_thread.h"
#include "pios_queue.h"
#include "misc_math.h"

#include "systemsettings.h"
#include "drivingstatus.h"
#include "cameradesired.h"
#include "caractuatorsettings.h"
#include "caractuatordesired.h"
#include "caractuatorcommand.h"
#include "carmixersettings.h"
#include "carmanualcontrolcommand.h"

#include "jlink_rtt.h"

// Private constants
#define MAX_QUEUE_SIZE 2

#if defined(PIOS_ACTUATOR_STACK_SIZE)
#define STACK_SIZE_BYTES PIOS_ACTUATOR_STACK_SIZE
#else
#define STACK_SIZE_BYTES 1336
#endif

#define TASK_PRIORITY PIOS_THREAD_PRIO_HIGHEST
#define FAILSAFE_TIMEOUT_MS 100

#define MIXER_SCALE 128

#define MAX_MIX_ACTUATORS 2

#ifndef MAX_MIX_ACTUATORS
#define MAX_MIX_ACTUATORS CARACTUATORCOMMAND_CHANNEL_NUMELEM
#endif

#define SERVO_NEUTRAL_POS 1460

DONT_BUILD_IF(CARACTUATORSETTINGS_TIMERUPDATEFREQ_NUMELEM > PIOS_SERVO_MAX_BANKS, TooManyServoBanks);
DONT_BUILD_IF(MAX_MIX_ACTUATORS > CARACTUATORCOMMAND_CHANNEL_NUMELEM, TooManyMixers);

// Private types

// Private variables
static struct pios_queue *queue;
static struct pios_thread *taskHandle;

// used to inform the actuator thread that actuator / mixer settings are updated
// set true to ensure they're fetched on first run
static volatile bool driving_status_updated = true;
static volatile bool manual_control_cmd_updated = true;
static volatile bool actuator_settings_updated = true;
static volatile bool mixer_settings_updated = true;

/* These are various settings objects used throughout the actuator code */
static CarActuatorSettingsData actuatorSettings;

// The actual mixer settings data, pulled at the top of the actuator thread
static CarMixerSettingsData mixerSettings;

// Private functions
static void actuator_task(void* parameters);

static void config_steering_driving_mixer();
static float scale_channel(float value, int idx);
static void set_failsafe();

static CarMixerSettingsMixer1TypeOptions get_mixer_type(int idx);

volatile enum actuator_interlock actuator_interlock = ACTUATOR_INTERLOCK_OK;

/**
 * @brief Module initialization
 * @return 0
 */
int32_t ActuatorStart()
{
	// Watchdog must be registered before starting task
	// PIOS_WDG_RegisterFlag(PIOS_WDG_ACTUATOR);

	// Start main task
	taskHandle = PIOS_Thread_Create(actuator_task, "Actuator", STACK_SIZE_BYTES, NULL, TASK_PRIORITY);
	TaskMonitorAdd(TASKINFO_RUNNING_ACTUATOR, taskHandle);

	return 0;
}

/**
 * @brief Module initialization
 * @return 0
 */
int32_t ActuatorInitialize()
{
	// Register for notification of changes to ActuatorSettings
	if (CarActuatorSettingsInitialize()  == -1) {
		return -1;
	}
	CarActuatorSettingsConnectCallbackCtx(UAVObjCbSetFlag, &actuator_settings_updated);

	// Register for notification of changes to MixerSettings
	if (CarMixerSettingsInitialize()  == -1) {
		return -1;
	}
	CarMixerSettingsConnectCallbackCtx(UAVObjCbSetFlag, &mixer_settings_updated);

	// Listen for ActuatorDesired updates (Primary input to this module)
	if (CarActuatorDesiredInitialize()  == -1) {
		return -1;
	}

	queue = PIOS_Queue_Create(MAX_QUEUE_SIZE, sizeof(UAVObjEvent));
	CarActuatorDesiredConnectQueue(queue);

	// Primary output of this module
	if (CarActuatorCommandInitialize() == -1) {
		return -1;
	}

	config_steering_driving_mixer();

	return 0;
}

MODULE_HIPRI_INITCALL(ActuatorInitialize, ActuatorStart);

static void config_steering_driving_mixer()
{
	// reserved actuators
	// - actuator mixer 1: (steering) servo
	// - actuator mixer 2: (driving) motor 
	mixerSettings.Mixer1Type = 	CARMIXERSETTINGS_MIXER1TYPE_SERVO;
	mixerSettings.Mixer2Type = 	CARMIXERSETTINGS_MIXER1TYPE_MOTOR; 
}

static void post_process_scale_and_commit(float *actuator_vect,
		float *desired_vect, float dT,
		bool armed, bool stabilize_now,
		float *maxpoweradd_bucket)
{
	CarActuatorCommandData command;

	float gain = 1.0f;
	float offset = 0.0f;

	for (int ct = 0; ct < MAX_MIX_ACTUATORS; ct++) {
		// Motors have additional protection for when to be on
		if (get_mixer_type(ct) == CARMIXERSETTINGS_MIXER1TYPE_MOTOR) {
			// if (!armed) {
			// 	actuator_vect[ct] = 0;  //force min throttle
			// } 
			// else {
				actuator_vect[ct] = actuator_vect[ct] * gain + offset;
			// }
		}

		command.Channel[ct] = scale_channel(actuator_vect[ct], ct);
	}	

	// Store update time
	command.UpdateTime = 1000.0f*dT;

	CarActuatorCommandMaxUpdateTimeGet(&command.MaxUpdateTime);

	if (command.UpdateTime > command.MaxUpdateTime)
		command.MaxUpdateTime = 1000.0f*dT;

	// Update output object
	if (!CarActuatorCommandReadOnly()) {
		CarActuatorCommandSet(&command);
	} else {
		// it's read only during servo configuration--
		// so GCS takes precedence.
		CarActuatorCommandGet(&command);
	}

	// JLinkRTTPrintf(0, "Armed: %d, Actuator command servo-motor: %ld, %ld\n", armed, (uint32_t)command.Channel[0], (uint32_t)command.Channel[1]);

	for (int n = 0; n < MAX_MIX_ACTUATORS; ++n) {
		PIOS_Servo_Set(n, command.Channel[n]);
	}

	PIOS_Servo_Update();
}

static void process_input_data(uint32_t this_systime,
		float actuator_vect[MAX_MIX_ACTUATORS],
		bool *armed, bool *stabilize_now)
{
	static DrivingStatusData drivingStatus;
	if (driving_status_updated) {
		DrivingStatusGet(&drivingStatus);
		driving_status_updated = false;
	}

	float steering_val = 0;
	float throttle_val = 0;	
	
	CarActuatorDesiredData desired;
	CarActuatorDesiredGet(&desired);

	steering_val = desired.Steering;
	throttle_val = desired.Throttle;

	*armed = drivingStatus.Armed == DRIVINGSTATUS_ARMED_ARMED;

	// if (!*armed) {
	// 	throttle_val = 0;
	// }

	*stabilize_now = throttle_val > 0.0f;

	actuator_vect[0] = steering_val;
	actuator_vect[1] = throttle_val;
}

/**
 * @brief Main Actuator module task
 *
 * Universal matrix based mixer for VTOL, helis and fixed wing.
 * Converts desired roll,pitch,yaw and throttle to servo/ESC outputs.
 *
 * Because of how the Throttle ranges from 0 to 1, the motors should too!
 *
 * Note this code depends on the UAVObjects for the mixers being all being the same
 * and in sequence. If you change the object definition, make sure you check the code!
 *
 * @return -1 if error, 0 if success
 */
static void actuator_task(void* parameters)
{
	// Connect update callbacks
	DrivingStatusConnectCallbackCtx(UAVObjCbSetFlag, &driving_status_updated);
	CarManualControlCommandConnectCallbackCtx(UAVObjCbSetFlag, &manual_control_cmd_updated);

	// Ensure the initial state of actuators is safe.
	actuator_settings_updated = false;
	CarActuatorSettingsGet(&actuatorSettings);

	PIOS_Servo_SetMode(actuatorSettings.TimerUpdateFreq,
			CARACTUATORSETTINGS_TIMERUPDATEFREQ_NUMELEM,
			actuatorSettings.ChannelMax,
			actuatorSettings.ChannelMin);
	set_failsafe();

	/* This is out here because not everything may change each time */
	uint32_t last_systime = PIOS_Thread_Systime();
	float desired_vect[CARMIXERSETTINGS_MIXER1VECTOR_NUMELEM] = { 0 };
	float dT = 0.0f;
	
	float maxpoweradd_bucket = 0.0f;

	// Main task loop
	while (1) {
		/* If settings objects have changed, update our internal
		 * state appropriately.
		 */
		if (actuator_settings_updated) {
			actuator_settings_updated = false;
			CarActuatorSettingsGet(&actuatorSettings);

			PIOS_Servo_SetMode(actuatorSettings.TimerUpdateFreq,
					CARACTUATORSETTINGS_TIMERUPDATEFREQ_NUMELEM,
					actuatorSettings.ChannelMax,
					actuatorSettings.ChannelMin);
		}

		if (mixer_settings_updated) {
			mixer_settings_updated = false;

			// compute_mixer();
			// // XXX compute_inverse_mixer();

			// CarMixerSettingsThrottleCurve1Get(curve1);
			// CarMixerSettingsThrottleCurve2Get(curve2);
			// CarMixerSettingsCurve2SourceGet(&curve2_src);
			config_steering_driving_mixer();
		}

		// PIOS_WDG_UpdateFlag(PIOS_WDG_ACTUATOR);

		UAVObjEvent ev;

		// Wait until the ActuatorDesired object is updated
		if (!PIOS_Queue_Receive(queue, &ev, FAILSAFE_TIMEOUT_MS)) {
			// If we hit a timeout, set the actuator failsafe and
			// try again.
			set_failsafe();
			continue;
		}

		uint32_t this_systime = PIOS_Thread_Systime();

		/* Check how long since last update; this is stored into the
		 * UAVO to allow analysis of actuation jitter.
		 */
		if (this_systime > last_systime) {
			dT = (this_systime - last_systime) / 1000.0f;
			/* (Otherwise, the timer has wrapped [rare] and we should
			 * just reuse dT)
			 */
		}

		last_systime = this_systime;

		if (actuator_interlock != ACTUATOR_INTERLOCK_OK) {
			/* Chosen because: 50Hz does 4-6 updates in 100ms */
			uint32_t exp_time = this_systime + 100;

			while (actuator_interlock != ACTUATOR_INTERLOCK_OK) {
				/* Simple state machine.  If someone has asked us to
				 * stop, set actuator failsafe for a short while.
				 * Then, set the flag to STOPPED.
				 *
				 * Setting to STOPPED isn't atomic, so we rely on
				 * anyone who has stopped us to waitfor STOPPED
				 * before putting us back to OK.
				 */
				if (actuator_interlock == ACTUATOR_INTERLOCK_STOPREQUEST) {
					set_failsafe();

					this_systime = PIOS_Thread_Systime();

					if ((exp_time - this_systime) > 100) {
						actuator_interlock = ACTUATOR_INTERLOCK_STOPPED;
					}
				}

				PIOS_Thread_Sleep(3);
				PIOS_WDG_UpdateFlag(PIOS_WDG_ACTUATOR);
			}

			PIOS_Servo_SetMode(actuatorSettings.TimerUpdateFreq,
					CARACTUATORSETTINGS_TIMERUPDATEFREQ_NUMELEM,
					actuatorSettings.ChannelMax,
					actuatorSettings.ChannelMin);
			continue;
		}


		float actuator_vect[MAX_MIX_ACTUATORS];
		bool armed, stabilize_now;

		/* Receive manual control and desired UAV objects.  Perform
		 * arming / hangtime checks; form a vector with desired
		 * axis actions.
		 */
		process_input_data(this_systime, actuator_vect, &armed,
				&stabilize_now);

		/* Perform clipping adjustments on the outputs, along with
		 * state-related corrections (spin while armed, disarmed, etc).
		 *
		 * Program the actual values to the timer subsystem.
		 */
		post_process_scale_and_commit(actuator_vect, desired_vect,
				dT, armed, stabilize_now,
				&maxpoweradd_bucket);

		/* If we got this far, everything is OK. */
		AlarmsClear(SYSTEMALARMS_ALARM_ACTUATOR);
	}
}

/**
 * Interpolate a collective curve
 *
 * we need to accept input in [-1,1] so that the neutral point may be set arbitrarily within the typical channel input range, which is [-1,1]
 *
 * @param input The input value, in [-1,1]
 * @param curve Array of points in the curve
 * @param num_points Number of points in the curve
 * @return the output value, in [-1,1]
 */
// static float collective_curve(float const input, float const * curve, uint8_t num_points)
// {
// 	return linear_interpolate(input, curve, num_points, -1.0f, 1.0f);
// }

/**
 * Convert channel from -1/+1 to servo pulse duration in microseconds
 */
static float scale_channel(float value, int idx)
{
	float max = actuatorSettings.ChannelMax[idx];
	float min = actuatorSettings.ChannelMin[idx];
	float neutral = actuatorSettings.ChannelNeutral[idx];

	float valueScaled;
	// Scale
	if (value >= 0.0f) {
		valueScaled = value*(max-neutral) + neutral;
	} else {
		valueScaled = value*(neutral-min) + neutral;
	}

	if (max>min) {
		if (valueScaled > max) valueScaled = max;
		if (valueScaled < min) valueScaled = min;
	} else {
		if (valueScaled < max) valueScaled = max;
		if (valueScaled > min) valueScaled = min;
	}

	return valueScaled;
}

static CarMixerSettingsMixer1TypeOptions get_mixer_type(int idx)
{
	switch (idx) {
	case 0:
		return mixerSettings.Mixer1Type;
		break;
	case 1:
		return mixerSettings.Mixer2Type;
		break;
	case 2:
		return mixerSettings.Mixer3Type;
		break;
	case 3:
		return mixerSettings.Mixer4Type;
		break;
	case 4:
		return mixerSettings.Mixer5Type;
		break;
	case 5:
		return mixerSettings.Mixer6Type;
		break;
	default:
		// We can never get here unless there are mixer channels not handled in the above. Fail out.
		PIOS_Assert(0);
	}

	/* Can't get here. Just make compiler happy */
	return mixerSettings.Mixer1Type;
}

static float channel_failsafe_value(int idx)
{
	switch (get_mixer_type(idx)) {
	case CARMIXERSETTINGS_MIXER1TYPE_MOTOR:
		return actuatorSettings.ChannelNeutral[idx];
		// if no reverse is allowed, return min value
		//return actuatorSettings.ChannelMin[idx];
	case CARMIXERSETTINGS_MIXER1TYPE_SERVO:
		// return actuatorSettings.ChannelNeutral[idx];
		return SERVO_NEUTRAL_POS;
	case CARMIXERSETTINGS_MIXER1TYPE_DISABLED:
		return 0;
	default:
		// TODO: is this actually right/safe?
		return 0;
	}

	/* Can't get here. */
	return -1;
}

/**
 * Set actuator output to the neutral values (failsafe)
 */
static void set_failsafe()
{
	float Channel[CARACTUATORCOMMAND_CHANNEL_NUMELEM] = {0};

	// Set alarm
	AlarmsSet(SYSTEMALARMS_ALARM_ACTUATOR, SYSTEMALARMS_ALARM_CRITICAL);

	// Update servo outputs
	for (int n = 0; n < MAX_MIX_ACTUATORS; ++n) {
		float fs_val = channel_failsafe_value(n);

		Channel[n] = fs_val;

		PIOS_Servo_Set(n, fs_val);
	}

	// JLinkRTTPrintf(0, "Failsafe command servo-motor: %ld, %ld\n", (uint32_t)Channel[0], (uint32_t)Channel[1]);

	PIOS_Servo_Update();

	// Update output object's parts that we changed
	CarActuatorCommandChannelSet(Channel);
}

/**
 * @}
 * @}
 */

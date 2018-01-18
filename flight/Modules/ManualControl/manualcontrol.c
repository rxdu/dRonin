/**
 ******************************************************************************
 * @addtogroup Modules Modules
 * @{
 * @addtogroup Control Control Module
 * @{
 * @brief      Highest level control module which decides the control state
 *
 * This module users the values from the transmitter and its settings to
 * to determine if the system is currently controlled by failsafe, transmitter,
 * or a tablet.  The transmitter values are read out and stored in @ref
 * ManualControlCommand.  The tablet sends values via @ref TabletInfo which
 * may be used if the driving mode switch is in the appropriate position. The
 * transmitter settings come from @ref ManualControlSettings.
 *
 * @file       manualcontrol.c
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013-2016
 * @brief      ManualControl module. Handles safety R/C link and driving mode.
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
 */

#include "openpilot.h"

#include "pios_thread.h"
#include "pios_rcvr.h"

#include "control.h"
#include "failsafe_control.h"
#include "transmitter_control.h"

#include "systemalarms.h"
#include "drivingstatus.h"

#include "carmanualcontrolcommand.h"
#include "carmanualcontrolsettings.h"
#include "carnavigationdesired.h"

#include "jlink_rtt.h"

// Private constants
#if defined(PIOS_MANUAL_STACK_SIZE)
#define STACK_SIZE_BYTES PIOS_MANUAL_STACK_SIZE
#else
#define STACK_SIZE_BYTES 1200
#endif

#define TASK_PRIORITY PIOS_THREAD_PRIO_HIGHEST
#define UPDATE_PERIOD_MS 20
#define CTRL_HEARTBEAT_MULTIPLIER	25	/* period = CTRL_HEARTBEAT_MULTIPLIER * UPDATE_PERIOD_MS */

#define GO_STATE(x) \
				do { \
					arm_state = (x); \
					arm_state_time = (now); \
				} while (0)

// Private variables
static struct pios_thread *taskHandle;

// Private functions
static void manualControlTask(void *parameters);
static DrivingStatusControlSourceOptions control_source_select();

bool vehicle_is_armed = false;
static uint32_t control_status_led = 0;

/**
 * Module starting
 */
int32_t ManualControlStart()
{
	// Watchdog must be registered before starting task
	// PIOS_WDG_RegisterFlag(PIOS_WDG_MANUAL);

	// Start main task
	taskHandle = PIOS_Thread_Create(manualControlTask, "Control", STACK_SIZE_BYTES, NULL, TASK_PRIORITY);
	TaskMonitorAdd(TASKINFO_RUNNING_MANUALCONTROL, taskHandle);

	return 0;
}

/**
 * Module initialization
 */
int32_t ManualControlInitialize()
{
	if (transmitter_control_initialize() == -1) {
	
		return -1;
	}

	return 0;
}

MODULE_HIPRI_INITCALL(ManualControlInitialize, ManualControlStart);

/**
 * Module task
 */
static void manualControlTask(void *parameters)
{
	/* Make sure disarmed on power up */
	DrivingStatusData drivingStatus;
	DrivingStatusGet(&drivingStatus);
	drivingStatus.Armed = DRIVINGSTATUS_ARMED_DISARMED;
	DrivingStatusSet(&drivingStatus);

	// TODO: Stop car before run
	
	uint32_t arm_time = 1000;

	enum arm_state {
		ARM_STATE_SAFETY,
		ARM_STATE_DISARMED,
		ARM_STATE_ARMING,
		ARM_STATE_HOLDING,
		ARM_STATE_ARMED,
	} arm_state = ARM_STATE_SAFETY;

	uint32_t arm_state_time = 0;

	while (1) {

		// Process periodic data for each of the controllers, including reading
		// all available inputs
		transmitter_control_update();
		failsafe_control_update();
		
		// Initialize to invalid value to ensure first update sets DrivingStatus
		static DrivingStatusControlSourceOptions last_control_selection = -1;

		// Control logic to select the valid controller
		DrivingStatusControlSourceOptions control_selection =
			control_source_select();
		bool reset_controller = control_selection != last_control_selection;

		enum control_status status = transmitter_control_get_status();

		switch(control_selection) {
		case DRIVINGSTATUS_CONTROLSOURCE_TRANSMITTER:
			transmitter_control_select(reset_controller);
			// JLinkRTTPrintf(0, "Control source: transmitter\n", 0);
			break;		
		default:
			/* Fall through into failsafe */
			failsafe_control_select(last_control_selection !=
					DRIVINGSTATUS_CONTROLSOURCE_FAILSAFE);
			control_selection = DRIVINGSTATUS_CONTROLSOURCE_FAILSAFE;
			// JLinkRTTPrintf(0, "Control source: failsafe\n", 0);
			break;
		}

		if (control_selection != last_control_selection) {
			DrivingStatusControlSourceSet(&control_selection);
			last_control_selection = control_selection;
		}

		uint32_t now = PIOS_Thread_Systime();

		/* Global state transitions: weird stuff means disarmed +
		 * safety hold-down.
		 */
		if ((status == STATUS_ERROR) ||
				(status == STATUS_SAFETYTIMEOUT)) {
			GO_STATE(ARM_STATE_SAFETY);
		}

		/* If there's an alarm that prevents arming, enter a 0.2s
		 * no-arming-holddown.  This also stretches transient alarms
		 * and makes them more visible.
		 */
		// if (!ok_to_arm() && (arm_state != ARM_STATE_ARMED)) {
		if ((arm_state != ARM_STATE_ARMED)) {
			GO_STATE(ARM_STATE_SAFETY);
		}

		switch (arm_state) {
			case ARM_STATE_SAFETY:
				/* state_time > 0.2s + "NONE" or "DISARM" -> DISARMED
				 *
				 * !"NONE" and !"DISARM" -> SAFETY, reset state_time
				 */
				if ((status == STATUS_NORMAL) ||
						(status == STATUS_DISARM)) {
					if (PIOS_Thread_Period_Elapsed(arm_state_time, 200)) {
						GO_STATE(ARM_STATE_DISARMED);
					}
				} else {
					GO_STATE(ARM_STATE_SAFETY);
				}

				break;
			case ARM_STATE_DISARMED:
				/* "ARM_*" -> ARMING
				 * "DISCONNECTED" -> SAFETY
				 * "INVALID_FOR_DISARMED" -> SAFETY
				 */

				if ((status == STATUS_ARM_INVALID) ||
						(status == STATUS_ARM_VALID)) {
					GO_STATE(ARM_STATE_ARMING);
				} else if ((status == STATUS_DISCONNECTED) ||
						(status == STATUS_INVALID_FOR_DISARMED)) {
					GO_STATE(ARM_STATE_SAFETY);
				}

				CarManualControlSettingsArmTimeOptions arm_enum;
				CarManualControlSettingsArmTimeGet(&arm_enum);

				arm_time = (arm_enum == CARMANUALCONTROLSETTINGS_ARMTIME_250) ? 250 :
					(arm_enum == CARMANUALCONTROLSETTINGS_ARMTIME_500) ? 500 :
					(arm_enum == CARMANUALCONTROLSETTINGS_ARMTIME_1000) ? 1000 :
					(arm_enum == CARMANUALCONTROLSETTINGS_ARMTIME_2000) ? 2000 : 1000;

				break;
			case ARM_STATE_ARMING:
				/* Anything !"ARM_*" -> SAFETY
				 *
				 * state_time > ArmTime + ARM_INVALID -> HOLDING
				 * state_time > ArmTime + ARM_VALID -> ARMED
				 */

				if ((status != STATUS_ARM_INVALID) &&
						(status != STATUS_ARM_VALID)) {
					GO_STATE(ARM_STATE_SAFETY);
				} else if (PIOS_Thread_Period_Elapsed(arm_state_time, arm_time)) {
					if (status == STATUS_ARM_VALID) {
						GO_STATE(ARM_STATE_ARMED);
					} else {
						/* Must be STATUS_ARM_INVALID */
						GO_STATE(ARM_STATE_HOLDING);
					}
				}

				break;
			case ARM_STATE_HOLDING:
				/* ARM_VALID -> ARMED
				 * NONE -> ARMED
				 *
				 * ARM_INVALID -> Stay here
				 *
				 * Anything else -> SAFETY
				 */
				if ((status == STATUS_ARM_VALID) ||
						(status == STATUS_NORMAL)) {
					GO_STATE(ARM_STATE_ARMED);
				} else if (status == STATUS_ARM_INVALID) {
					/* TODO: could consider having a maximum
					 * time before we go to safety */
				} else {
					GO_STATE(ARM_STATE_SAFETY);
				}

				break;
			case ARM_STATE_ARMED:
				/* "DISARM" -> SAFETY (lower layer's job to check
				 * 	DisarmTime)
				 */
				if (status == STATUS_DISARM) {
					GO_STATE(ARM_STATE_SAFETY);
				}

				break;
		}

		DrivingStatusArmedOptions armed, prev_armed;

		DrivingStatusArmedGet(&prev_armed);

		switch (arm_state) {
			default:
			case ARM_STATE_SAFETY:
			case ARM_STATE_DISARMED:
				armed = DRIVINGSTATUS_ARMED_DISARMED;
				vehicle_is_armed = false;
				break;
			case ARM_STATE_ARMING:
				armed = DRIVINGSTATUS_ARMED_ARMING;
				break;
			case ARM_STATE_HOLDING:
				/* For now consider "HOLDING" an armed state,
				 * like old code does.  (Necessary to get the
				 * "initial spin while armed" that causes user
				 * to release arming position).
				 *
				 * TODO: do something different because
				 * control position invalid.
				 */
			case ARM_STATE_ARMED:
				armed = DRIVINGSTATUS_ARMED_ARMED;
				vehicle_is_armed = true;
				break;
		}

		if (armed != prev_armed) {
			DrivingStatusArmedSet(&armed);
		}

		// Wait until next update
		PIOS_RCVR_WaitActivity(UPDATE_PERIOD_MS);
		// PIOS_WDG_UpdateFlag(PIOS_WDG_MANUAL);

		if((control_status_led++)%CTRL_HEARTBEAT_MULTIPLIER == 0)
			PIOS_ANNUNC_Toggle(PIOS_LED_HEARTBEAT);
	}
}


/**
 * @brief control_source_select Determine which sub-module to use
 * for the main control source of the driving controller.
 * @returns @ref DrivingStatusControlSourceOptions indicating the selected
 * mode
 *
 * This function is the ultimate one that controls what happens and
 * selects modes such as failsafe, transmitter control, geofencing
 * and potentially other high level modes in the future
 * 
 * RC Cars will be in either Failsafe mode or Transmitter mode.
 * 
 * Failsafe mode: set all actuator desired values to be 0
 * 		- Triggered when no transmiter signal is received
 * Transmitter mode: actuator desired values are set according to selected driving mode
 * 		- Manual: control values from remote controller sticks
 * 		- Navigation: control values from CAN bus
 * 		- Emergency: stop the car as commanded by user from remote controller
 */
static DrivingStatusControlSourceOptions control_source_select()
{
	CarManualControlCommandData cmd;
	CarManualControlCommandGet(&cmd);
	if (cmd.Connected != CARMANUALCONTROLCOMMAND_CONNECTED_TRUE) {
		return DRIVINGSTATUS_CONTROLSOURCE_FAILSAFE;
	} 
	else {
		return DRIVINGSTATUS_CONTROLSOURCE_TRANSMITTER;
	}
}

/**
  * @}
  * @}
  */

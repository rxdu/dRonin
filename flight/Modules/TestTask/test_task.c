/**
 ******************************************************************************
 * @addtogroup TauLabsModules Tau Labs Modules
 * @{
 * @addtogroup Control Control Module
 * @{
 * @brief      Highest level control module which decides the control state
 *
 * This module users the values from the transmitter and its settings to
 * to determine if the system is currently controlled by failsafe, transmitter,
 * or a tablet.  The transmitter values are read out and stored in @ref
 * ManualControlCommand.  The tablet sends values via @ref TabletInfo which
 * may be used if the flight mode switch is in the appropriate position. The
 * transmitter settings come from @ref ManualControlSettings.
 *
 * @file       manualcontrol.c
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013-2016
 * @brief      ManualControl module. Handles safety R/C link and flight mode.
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

#include "pios_queue.h"
#include "pios_thread.h"
#include "pios_rcvr.h"

#include "flightstatus.h"
#include "systemalarms.h"

#include "pios_can.h"
#include "jlink_rtt.h"

extern uintptr_t pios_can_id;

// Private constants
#if defined(PIOS_MANUAL_STACK_SIZE)
#define STACK_SIZE_BYTES PIOS_MANUAL_STACK_SIZE
#else
#define STACK_SIZE_BYTES 1200
#endif

#define TASK_PRIORITY PIOS_THREAD_PRIO_NORMAL
#define UPDATE_PERIOD_MS 500

#define CAN_RX_TIMEOUT_MS 500

// Private variables
static struct pios_queue *queue;
static struct pios_thread *taskHandle;

// Private functions
static void testTask(void *parameters);

/**
 * Module starting
 */
int32_t TestTaskStart()
{
	// Watchdog must be registered before starting task
	//PIOS_WDG_RegisterFlag(PIOS_WDG_MANUAL);

	/* Delay system */
	PIOS_DELAY_Init();

	// Start main task
	taskHandle = PIOS_Thread_Create(testTask, "TestTask", STACK_SIZE_BYTES, NULL, TASK_PRIORITY);
	//TaskMonitorAdd(TASKINFO_RUNNING_MANUALCONTROL, taskHandle);

	return 0;
}

/**
 * Module initialization
 */
int32_t TestTaskInitialize()
{

	queue = PIOS_CAN_RegisterMessageQueue(pios_can_id, PIOS_CAN_GIMBAL);

	return 0;
}

MODULE_HIPRI_INITCALL(TestTaskInitialize, TestTaskStart);

/**
 * Module task
 */
static void testTask(void *parameters)
{
	while (1)
	{
		PIOS_ANNUNC_Toggle(PIOS_LED_HEARTBEAT);
		//SEGGER_RTT_WriteString(0, "segger rtt test\n");

		struct pios_can_gimbal_message bgc_message = {
			.fc_roll = 1,
			.fc_pitch = 2,
			.fc_yaw = 3,
			.setpoint_roll = 2,
			.setpoint_pitch = 16,
			.setpoint_yaw = 32};

		PIOS_CAN_TxData(pios_can_id, PIOS_CAN_GIMBAL, (uint8_t *)&bgc_message);

		UAVObjEvent ev;
		if (PIOS_Queue_Receive(queue, &ev, CAN_RX_TIMEOUT_MS))
		{
			JLinkWriteString(0, "CAN msg received\n");
		}
		else
		{
			JLinkWriteString(0, "---");
		}

		// Wait until next update
		//PIOS_RCVR_WaitActivity(UPDATE_PERIOD_MS);
		//PIOS_WDG_UpdateFlag(PIOS_WDG_MANUAL);
		PIOS_DELAY_WaitmS(UPDATE_PERIOD_MS);
	}
}

/**
  * @}
  * @}
  */

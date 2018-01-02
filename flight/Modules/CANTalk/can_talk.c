/* 
 * can_talk.c
 * 
 * Created on: Dec 28, 2017 15:32
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include "openpilot.h"
#include "pios_queue.h"
#include "pios_thread.h"

#include "pixcar_can.h"

// UAVOs
#include "gyros.h"
#include "accels.h"
#include "magnetometer.h"
#include "hallsensor.h"

#include "jlink_rtt.h"

// Private constants
#define STACK_SIZE_BYTES 1024 * 8

#define TASK_PRIORITY PIOS_THREAD_PRIO_HIGH

#define GYRO_TIMEOUT_MS 1

#define UPDATE_PERIOD_MS 5
#define UPDATE_PERIOD_US 2000

// Private variables
static struct pios_thread *taskHandle;

// sensor queues
static struct pios_queue *speedQueue;

// Private functions
static void canTalkTask(void *parameters);

/**
 * Module starting
 */
int32_t CANTalkStart()
{
	// Create the queues for the sensors
	speedQueue = PIOS_Queue_Create(1, sizeof(UAVObjEvent));
	HallSensorConnectQueue(speedQueue);

	// Watchdog must be registered before starting task
	// PIOS_WDG_RegisterFlag(PIOS_WDG_MANUAL);

	/* Delay system */
	PIOS_DELAY_Init();

	// Start main task
	taskHandle = PIOS_Thread_Create(canTalkTask, "CANTalk", STACK_SIZE_BYTES, NULL, TASK_PRIORITY);
	//TaskMonitorAdd(TASKINFO_RUNNING_MANUALCONTROL, taskHandle);

	return 0;
}

/**
 * Module initialization
 */
int32_t CANTalkInitialize()
{
	return 0;
}

MODULE_HIPRI_INITCALL(CANTalkInitialize, CANTalkStart);

/**
 * Module task
 */
static void canTalkTask(void *parameters)
{
	UAVObjEvent ev;
	HallSensorData hallData;

	hallData.count = 0;

	uint32_t sys_time = PIOS_Thread_Systime();
	// static uint32_t loop_count = 0;
	while (1)
	{
		// static uint32_t time_label = 0;
		// uint32_t prev_time_label = time_label;
		// time_label = PIOS_DELAY_GetuS();
		// if(time_label > prev_time_label)
		// 	JLinkRTTPrintf(0, "%ld\n", time_label - prev_time_label);
		// else
		// 	JLinkRTTPrintf(0, "%ld\n", 0xffffffff - prev_time_label + time_label);

		uint32_t time_stamp = PIOS_Thread_Systime();
		
		if (PIOS_Queue_Receive(speedQueue, &ev, 0))
		{
			struct CANSpeedRawData spd_raw;

			HallSensorGet(&hallData);

			// Send latest speed measurement
			// float speed = calcCarSpeed();
			spd_raw.time_stamp = time_stamp;
			spd_raw.hallsensor_count = hallData.count;
			spd_raw.speed_estimate = hallData.speed_estimate;

			(void)spd_raw;
			// UAVCANNode_PublishSpeedData(&spd_raw);

			// JLinkRTTPrintf(0, "Speed: %ld\n", spd_raw.speed);
		}
		// else
		// 	JLinkRTTPrintf(0, "No speed data: %ld\n",0);

		// if (loop_count++ % 200 == 0)
		// 	updateCANNodeStatus(false);

		PIOS_Thread_Sleep_Until(&sys_time, UPDATE_PERIOD_MS);
	}
}

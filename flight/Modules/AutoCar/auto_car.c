/* 
 * auto_car.c
 * 
 * Created on: Nov 04, 2017
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include "openpilot.h"
#include "pios_thread.h"
#include "pios_queue.h"
#include "pios_semaphore.h"

#include "carnavigationdesired.h"
#include "hallsensor.h"
#include "drivingstatus.h"
#include "caractuatordesired.h"
#include "carmanualcontrolcommand.h"

#include "auto_car.h"
#include "can_talk.h"
#include "jlink_rtt.h"

extern uintptr_t pios_can_id;

// Private constants
#if defined(PIOS_MANUAL_STACK_SIZE)
#define STACK_SIZE_BYTES PIOS_MANUAL_STACK_SIZE
#else
#define STACK_SIZE_BYTES 1200
#endif

#define TASK_PRIORITY PIOS_THREAD_PRIO_HIGH

#define UPDATE_PERIOD_MS 10
#define UPDATE_PERIOD_US 2000

#define HALL_SENSOR_QUEUE_SIZE 2
#define CAN_CMD_QUEUE_SIZE 2

// Private variables
static uint32_t idleCounter;
static uint32_t idleCounterClear;

static struct pios_can_cmd_data prev_can_cmd_data;

static struct pios_queue *hallsensor_queue;
static struct pios_queue *navigation_desired_queue;

static volatile bool driving_status_update_flag = true;
static bool is_in_navigation_mode = false;
static volatile bool transmitter_status_update_flag = true;
static bool transmitter_connected = false;

static struct pios_thread *taskHandle;

// Private functions
static void autoCarTask(void *parameters);
static void AutoCarResetNavigationDesired();

/**
 * Module starting
 */
int32_t AutoCarTaskStart()
{
	// Watchdog must be registered before starting task
	//PIOS_WDG_RegisterFlag(PIOS_WDG_MANUAL);

	// Start main task
	taskHandle = PIOS_Thread_Create(autoCarTask, "autoCarTask", STACK_SIZE_BYTES, NULL, TASK_PRIORITY);
	//TaskMonitorAdd(TASKINFO_RUNNING_MANUALCONTROL, taskHandle);

	return 0;
}

/**
 * Module initialization
 */
int32_t AutoCarTaskInitialize()
{
	if (HallSensorInitialize() == -1 || CarNavigationDesiredInitialize() == -1)
		return -1;

	// Create queues
	navigation_desired_queue = PIOS_Queue_Create(CAN_CMD_QUEUE_SIZE, sizeof(UAVObjEvent));
	CarNavigationDesiredConnectQueue(navigation_desired_queue);

	hallsensor_queue = PIOS_Queue_Create(HALL_SENSOR_QUEUE_SIZE, sizeof(UAVObjEvent));
	HallSensorConnectQueue(hallsensor_queue);

	prev_can_cmd_data.steering = 0.0;
	prev_can_cmd_data.throttle = 0.0;

	return 0;
}

MODULE_HIPRI_INITCALL(AutoCarTaskInitialize, AutoCarTaskStart);

/**
 * Module task
 */
static void autoCarTask(void *parameters)
{
	UAVObjEvent ev;

	HallSensorData hallData;
	hallData.count = 0;

	struct CANSpeedRawData spd_raw;

	uint32_t sys_time = PIOS_Thread_Systime();
	uint32_t last_update_time = sys_time;
	static uint32_t loop_count = 0;

	DrivingStatusConnectCallbackCtx(UAVObjCbSetFlag, &driving_status_update_flag);
	CarManualControlCommandConnectCallbackCtx(UAVObjCbSetFlag, &transmitter_status_update_flag);

	while (1)
	{
		// static uint32_t time_label = 0;
		// uint32_t prev_time_label = time_label;
		// time_label = PIOS_DELAY_GetuS();
		// if(time_label > prev_time_label)
		// 	JLinkRTTPrintf(0, "%ld\n", time_label - prev_time_label);
		// else
		// 	JLinkRTTPrintf(0, "%ld\n", 0xffffffff - prev_time_label + time_label);

		if (transmitter_status_update_flag)
		{
			transmitter_status_update_flag = false;

			CarManualControlCommandData cmd;
			CarManualControlCommandGet(&cmd);

			if (cmd.Connected == CARMANUALCONTROLCOMMAND_CONNECTED_TRUE)
			{
				transmitter_connected = true;
				// JLinkRTTPrintf(0, "Transmitter connected\n", 0);
			}
			else
			{
				transmitter_connected = false;
				JLinkRTTPrintf(0, "Transmitter lost connection\n", 0);
			}
		}

		if (driving_status_update_flag)
		{
			driving_status_update_flag = false;

			DrivingStatusDrivingModeOptions current_mode;
			DrivingStatusDrivingModeGet(&current_mode);

			if (current_mode == DRIVINGSTATUS_DRIVINGMODE_NAVIGATION && transmitter_connected == true)
			{
				JLinkRTTPrintf(0, "In navigation mode\n", 0);
				is_in_navigation_mode = true;
			}
			else
			{
				JLinkRTTPrintf(0, "Left navigation mode\n", 0);
				is_in_navigation_mode = false;
				AutoCarResetNavigationDesired();
			}
		}

		// get current system time
		sys_time = PIOS_Thread_Systime();

		if (PIOS_Queue_Receive(hallsensor_queue, &ev, 0) != false)
		{
			last_update_time = sys_time;

			HallSensorGet(&hallData);

			// Send latest speed measurement
			spd_raw.time_stamp = sys_time;
			// spd_raw.hallsensor_count = hallData.count;
			spd_raw.speed_estimate = hallData.speed;

			(void)spd_raw;
			AutoCarPublishSpeedData(&spd_raw);

			// JLinkRTTPrintf(0, "Raw: %d , Speed: %ld\n", hallData.count, (int32_t)(hallData.speed * 100));
		}
		else
		{
			uint32_t out_dated = sys_time - last_update_time;
			(void)out_dated;
			// the car is running at very low speed, smaller than 0.07892467125 m/s
			// JLinkRTTPrintf(0, "No speed data for: %d ms\n", out_dated);
			// Send latest speed measurement
			spd_raw.time_stamp = sys_time;
			// spd_raw.hallsensor_count = hallData.count;
			spd_raw.speed_estimate = 0;

			(void)spd_raw;
			AutoCarPublishSpeedData(&spd_raw);
		}

		if (loop_count++ % 200 == 0)
			AutoCarPublishHeartbeat();

		PIOS_Thread_Sleep_Until(&sys_time, UPDATE_PERIOD_MS);
	}
}

struct pios_queue *AutoCarGetHallSensorQueue(void)
{
	if (hallsensor_queue == NULL)
		hallsensor_queue = PIOS_Queue_Create(HALL_SENSOR_QUEUE_SIZE, sizeof(struct pios_sensor_hallsensor_data));

	return hallsensor_queue;
}

void AutoCarUpdateHallsensorData(uint16_t hall_count)
{
	static bool first_call = true;
	if (first_call)
	{
		HallSensorInitialize();
		first_call = false;
	}

	HallSensorData hallsensorData;
	HallSensorGet(&hallsensorData);

	hallsensorData.count = hall_count;
	hallsensorData.speed = AutoCarUpdateCarSpeed(hallsensorData.count);

	// JLinkRTTPrintf(0, "Raw: %d , Speed: %ld\n", hallsensorData.count, (int32_t)(hallsensorData.speed*100));

	HallSensorSet(&hallsensorData);
}

float AutoCarUpdateCarSpeed(uint16_t hall_count)
{
	// 1.0e6 units = 1 second
	// hall_count = number of time units between two pulses
	// 1.0e6/(hall_count * TRIGGERS_PER_ROUND) = rounds per second (RPS)
	// RPS * (M_PI * WHEEL_DIAMETER) = m/s
	float latest_speed = 1.0e6 / (hall_count * TRIGGERS_PER_ROUND) * (M_PI * WHEEL_DIAMETER) / GEAR_RATIO;
	float speed_est = latest_speed;

	return speed_est;
}

void AutoCarResetNavigationDesired()
{
	prev_can_cmd_data.steering = 0;
	prev_can_cmd_data.throttle = 0;
}

void AutoCarSetNavigationDesired(struct pios_can_cmd_data *cmd)
{
	if (cmd->update_flags & 0x01)
		prev_can_cmd_data.steering = cmd->steering;
	if (cmd->update_flags & 0x02)
		prev_can_cmd_data.throttle = cmd->throttle;

	if (is_in_navigation_mode)
	{
		CarActuatorDesiredData actuator;
		actuator.Roll = 0;  //cmd->Roll;
		actuator.Pitch = 0; //cmd->Pitch;
		actuator.Yaw = 0;   //cmd->Yaw;
		actuator.Steering = prev_can_cmd_data.steering;
		actuator.Throttle = prev_can_cmd_data.throttle;
		CarActuatorDesiredSet(&actuator);
		// JLinkRTTPrintf(0, "Set navigation desired from CAN: %d, %d\n", (int32_t)(actuator.Steering * 100), (int32_t)(actuator.Throttle * 100));
	}
}

/**
 * Called by the RTOS when the CPU is idle, used to measure the CPU idle time.
 */
void vApplicationIdleHook(void)
{
	// Called when the scheduler has no tasks to run
	if (idleCounterClear == 0)
	{
		++idleCounter;
	}
	else
	{
		idleCounter = 0;
		idleCounterClear = 0;
	}
}

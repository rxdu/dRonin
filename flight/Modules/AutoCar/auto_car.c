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

#define UPDATE_PERIOD_MS 5
#define UPDATE_PERIOD_US 2000

#define CAN_RX_TIMEOUT_MS 10

#define HALL_SENSOR_QUEUE_SIZE 1
#define CAN_CMD_QUEUE_SIZE 1 

// Private variables
static uint32_t idleCounter;
static uint32_t idleCounterClear;

static struct pios_can_cmd_data prev_can_cmd_data;

static HallSensorData hallsensorData;
static struct pios_sensor_hallsensor_data hallsensor_data;

// sensor queues
static struct pios_queue *speedQueue;

static struct pios_queue *hallsensor_queue;
static struct pios_queue *navigation_desired_queue;

static struct pios_thread *taskHandle;

// Private functions
static void autoCarTask(void *parameters);
static void update_hallsensor_data(struct pios_sensor_hallsensor_data *hall);

/**
 * Module starting
 */
int32_t AutoCarTaskStart()
{
	/* Delay system */
	PIOS_DELAY_Init();

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
	if(HallSensorInitialize() == -1 \
		|| CarNavigationDesiredInitialize() == -1)
		return -1;

	prev_can_cmd_data.steering = 0.0;
	prev_can_cmd_data.throttle = 0.0;		

	// Create the queues
	navigation_desired_queue = PIOS_Queue_Create(CAN_CMD_QUEUE_SIZE, sizeof(UAVObjEvent));
	CarNavigationDesiredConnectQueue(navigation_desired_queue);

	hallsensor_queue = PIOS_Queue_Create(HALL_SENSOR_QUEUE_SIZE, sizeof(struct pios_sensor_hallsensor_data));

	speedQueue = PIOS_Queue_Create(1, sizeof(UAVObjEvent));
	HallSensorConnectQueue(speedQueue);

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

		if (PIOS_Queue_Receive(hallsensor_queue, &hallsensor_data, 0) != false) 
		{			
			update_hallsensor_data(&hallsensor_data);
		}
		
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
			// AutoCarPublishSpeedData(&spd_raw);

			// JLinkRTTPrintf(0, "Speed: %ld\n", spd_raw.speed);
		}

		// struct CANSpeedRawData spd_raw;
		// spd_raw.time_stamp = time_stamp;
		// spd_raw.hallsensor_count = 0;//hallData.count;
		// spd_raw.speed_estimate = 25;//hallData.speed_estimate;
		// AutoCarPublishSpeedData(&spd_raw);
		// else
		// 	JLinkRTTPrintf(0, "No speed data: %ld\n",0);

		// if (loop_count++ % 200 == 0)
		// 	updateCANNodeStatus(false);

		PIOS_Thread_Sleep_Until(&sys_time, UPDATE_PERIOD_MS);
	}
}

struct pios_queue *AutoCarGetHallSensorQueue(void)
{
	if(hallsensor_queue == NULL)
		hallsensor_queue = PIOS_Queue_Create(HALL_SENSOR_QUEUE_SIZE, sizeof(struct pios_sensor_hallsensor_data));

	return hallsensor_queue;
}

/**
 * Update the hall sensor uavo from the data from the hall sensor queue
 * @param [in] baro raw hall sensor data
 */
static void update_hallsensor_data(struct pios_sensor_hallsensor_data *hall)
{
	hallsensorData.count = hall->count;
	hallsensorData.speed_estimate = AutoCarUpdateCarSpeed(hallsensorData.count);
	HallSensorSet(&hallsensorData);
}

float AutoCarUpdateCarSpeed(uint16_t hall_count)
{
	float latest_speed = 1.0e6/(hall_count * 6.0)/GEAR_RATIO*(M_PI*WHEEL_DIAMETER);
	float speed_est = latest_speed;

	return speed_est;
}

void AutoCarResetNavigationDesired()
{
	CarNavigationDesiredData nav_desired;
	CarNavigationDesiredGet(&nav_desired);

	nav_desired.Steering = 0;
	nav_desired.Throttle = 0;

	CarNavigationDesiredSet(&nav_desired);
}

void AutoCarSetNavigationDesired(struct pios_can_cmd_data * cmd)
{
	CarNavigationDesiredData nav_desired;
	CarNavigationDesiredGet(&nav_desired);

	nav_desired.Steering = cmd->steering;
	nav_desired.Throttle = cmd->throttle;

	CarNavigationDesiredSet(&nav_desired);

	// JLinkRTTPrintf(0, "Set navigation desired from CAN: %d, %d\n",(int32_t)(nav_desired.Steering*100), (int32_t)(nav_desired.Throttle*100));
}

void AutoCarGetNavigationDesired(struct pios_can_cmd_data * cmd)
{
	UAVObjEvent ev;
	if(PIOS_Queue_Receive(navigation_desired_queue, &ev, 0))
	{			 		
		CarNavigationDesiredData nav_desired;
		CarNavigationDesiredGet(&nav_desired);

		prev_can_cmd_data.steering = nav_desired.Steering;
		prev_can_cmd_data.throttle = nav_desired.Throttle;		
	}

	// update CAN cmd if there exists a new one in queue
	//	otherwise use previous values
	cmd->steering = prev_can_cmd_data.steering;
	cmd->throttle = prev_can_cmd_data.throttle;
}

/**
 * Called by the RTOS when the CPU is idle, used to measure the CPU idle time.
 */
void vApplicationIdleHook(void)
{
	// Called when the scheduler has no tasks to run
	if (idleCounterClear == 0) {
		++idleCounter;
	} else {
		idleCounter = 0;
		idleCounterClear = 0;
	}
}

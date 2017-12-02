/* 
 * can_bridge.c
 * 
 * Created on: Nov 04, 2017 17:12
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#include "openpilot.h"
#include "pios_queue.h"
#include "pios_thread.h"

#include "uavcan_interface.h"

// UAVOs
#include "gyros.h"
#include "accels.h"
#include "magnetometer.h"
#include "hallsensor.h"

#include "jlink_rtt.h"

// Private constants
#if defined(PIOS_MANUAL_STACK_SIZE)
#define STACK_SIZE_BYTES PIOS_MANUAL_STACK_SIZE
#else
#define STACK_SIZE_BYTES 2048
#endif

#define TASK_PRIORITY PIOS_THREAD_PRIO_HIGH

#define GYRO_TIMEOUT_MS 1

#define UPDATE_PERIOD_MS 5
#define UPDATE_PERIOD_US 2000

// Parameters for speed estimation
#define WHEEL_DIAMETER 0.065
#define GEAR_RATIO	6.58	// with 20T pinion gear

// Private variables
static struct pios_thread *taskHandle;

// sensor queues
static struct pios_queue *gyroQueue;
static struct pios_queue *accelQueue;
static struct pios_queue *magQueue;
static struct pios_queue *speedQueue;

// Private functions
static void canBridgeTask(void *parameters);
// static float calcCarSpeed(void);

/**
 * Module starting
 */
int32_t CANBridgeStart()
{
	// Create the queues for the sensors
	gyroQueue = PIOS_Queue_Create(1, sizeof(UAVObjEvent));
	accelQueue = PIOS_Queue_Create(1, sizeof(UAVObjEvent));
	magQueue = PIOS_Queue_Create(1, sizeof(UAVObjEvent));
	speedQueue = PIOS_Queue_Create(1, sizeof(UAVObjEvent));

	GyrosConnectQueue(gyroQueue);
	AccelsConnectQueue(accelQueue);
	if (MagnetometerHandle())
		MagnetometerConnectQueue(magQueue);
	HallSensorConnectQueue(speedQueue);

	// Watchdog must be registered before starting task
	// PIOS_WDG_RegisterFlag(PIOS_WDG_MANUAL);

	/* Delay system */
	PIOS_DELAY_Init();

	// Start main task
	taskHandle = PIOS_Thread_Create(canBridgeTask, "CANBridge", STACK_SIZE_BYTES, NULL, TASK_PRIORITY);
	//TaskMonitorAdd(TASKINFO_RUNNING_MANUALCONTROL, taskHandle);

	return 0;
}

/**
 * Module initialization
 */
int32_t CANBridgeInitialize()
{
	return 0;
}

MODULE_HIPRI_INITCALL(CANBridgeInitialize, CANBridgeStart);

/**
 * Module task
 */
static void canBridgeTask(void *parameters)
{
	UAVObjEvent ev;
	GyrosData gyrosData;
	AccelsData accelsData;
	MagnetometerData magData;
	HallSensorData hallData;

	gyrosData.x = 0;
	gyrosData.y = 0;
	gyrosData.z = 0;

	accelsData.x = 0;
	accelsData.y = 0;
	accelsData.z = 9.8;

	magData.x = 100;
	magData.y = 0;
	magData.z = 0;

	hallData.count = 0;

    while (1)
	{
		static uint32_t time_label = 0;
		uint32_t prev_time_label = time_label;
		time_label = PIOS_DELAY_GetuS();
		uint32_t error = time_label - prev_time_label;
		//if(error > 5200)
		JLinkRTTPrintf(0, "can bridge update period: %ld\n",error);

		uint32_t start_time = PIOS_DELAY_GetuS();

		bool gyro_accel_updated = true;
		uint32_t time_stamp = PIOS_Thread_Systime();		

		bool gyroTimeout  = PIOS_Queue_Receive(gyroQueue, &ev, GYRO_TIMEOUT_MS) != true;
		bool accelTimeout = PIOS_Queue_Receive(accelQueue, &ev, 0) != true;

		// When one of these is updated so should the other.
		if (gyroTimeout || accelTimeout) {
			// failed to get new sensor data, do not send to CAN bus
			gyro_accel_updated = false;
		}
		
		// Send IMU sensor data to CAN bus if updated		
		if(gyro_accel_updated)
		{
			struct CANIMURawData imu_raw;

			GyrosGet(&gyrosData);
			AccelsGet(&accelsData);			
			
			imu_raw.time_stamp = time_stamp;

			imu_raw.gyro.x = gyrosData.x;
			imu_raw.gyro.y = gyrosData.y;
			imu_raw.gyro.z = gyrosData.z;

			imu_raw.accel.x = accelsData.x;
			imu_raw.accel.y = accelsData.y;
			imu_raw.accel.z = accelsData.z;

			UAVCANNode_PublishIMUData(&imu_raw);

			if(PIOS_Queue_Receive(magQueue, &ev, 0))
			{
				struct CANMagRawData mag_raw;
	
				MagnetometerGet(&magData);
	
				mag_raw.time_stamp = time_stamp;
	
				mag_raw.mag.x = magData.x;
				mag_raw.mag.y = magData.y;
				mag_raw.mag.z = magData.z;
	
				// (void)mag_raw;
				UAVCANNode_PublishMagData(&mag_raw);
			}
			// else
			// 	JLinkRTTPrintf(0, "No Mag data: %ld\n",0);

			if(PIOS_Queue_Receive(speedQueue, &ev, 0))
			{			 		
				struct CANSpeedRawData spd_raw;

				HallSensorGet(&hallData);

				// Send latest speed measurement
				// float speed = calcCarSpeed();				
				spd_raw.time_stamp = time_stamp;
				spd_raw.speed = hallData.count;
				(void)spd_raw;
				UAVCANNode_PublishSpeedData(&spd_raw);

				JLinkRTTPrintf(0, "Speed: %ld\n",spd_raw.speed);
			}
		}
		// else
		// 	JLinkRTTPrintf(0, "No speed data: %ld\n",0);
		
		// PIOS_WDG_UpdateFlag(PIOS_WDG_MANUAL);
		// PIOS_DELAY_WaitmS(UPDATE_PERIOD_MS);
		
		uint32_t spent_time = PIOS_DELAY_GetuSSince(start_time);
		// (void)spent_time;
		if(spent_time < UPDATE_PERIOD_US)
			PIOS_DELAY_WaituS(UPDATE_PERIOD_US - spent_time);
    }
}

// static float calcCarSpeed()
// {
// 	uint16_t hall_reading = PIOS_TIM_GetHallSensorReading();
// 	// JLinkRTTPrintf(0, "hall reading: %ld\n",hall_reading);
// 	float speed = 1.0e6/(hall_reading * 6.0)/GEAR_RATIO*(M_PI*WHEEL_DIAMETER);
// 	return speed;
// }
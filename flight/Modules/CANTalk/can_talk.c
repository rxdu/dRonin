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

#include "canard.h"
#include "pios_canard.h"
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
static struct pios_queue *gyroQueue;
static struct pios_queue *accelQueue;
static struct pios_queue *magQueue;
static struct pios_queue *speedQueue;

// libcanard
static CanardInstance *canard;
static uint8_t canard_memory_pool[1024];
static uint8_t node_health = UAVCAN_NODE_HEALTH_OK;
static uint8_t node_mode = UAVCAN_NODE_MODE_INITIALIZATION;

// Private functions
static void canTalkTask(void *parameters);
static void updateCANNodeStatus(bool print_mem_stat);
void makeNodeStatusMessage(uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE]);
void processTxRxOnce();

/**
 * Module starting
 */
int32_t CANTalkStart()
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

	// Initialize libcanard
	canard = getCanardInstance();
	canardInit(canard, canard_memory_pool, sizeof(canard_memory_pool), onTransferReceived, shouldAcceptTransfer, NULL);
	canardSetLocalNodeID(canard, 16);

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
	// UAVObjEvent ev;
	// GyrosData gyrosData;
	// AccelsData accelsData;
	// MagnetometerData magData;
	// HallSensorData hallData;

	// gyrosData.x = 0;
	// gyrosData.y = 0;
	// gyrosData.z = 0;

	// accelsData.x = 0;
	// accelsData.y = 0;
	// accelsData.z = 9.8;

	// magData.x = 100;
	// magData.y = 0;
	// magData.z = 0;

	// hallData.count = 0;

	uint32_t sys_time = PIOS_Thread_Systime();
	static uint32_t loop_count = 0;
	while (1)
	{
		// static uint32_t time_label = 0;
		// uint32_t prev_time_label = time_label;
		// time_label = PIOS_DELAY_GetuS();
		// if(time_label > prev_time_label)
		// 	JLinkRTTPrintf(0, "%ld\n", time_label - prev_time_label);
		// else
		// 	JLinkRTTPrintf(0, "%ld\n", 0xffffffff - prev_time_label + time_label);

		// uint32_t time_stamp = PIOS_Thread_Systime();

		// Send IMU sensor data to CAN bus if updated
		// if (PIOS_Queue_Receive(gyroQueue, &ev, GYRO_TIMEOUT_MS) && PIOS_Queue_Receive(accelQueue, &ev, 0))
		// {
		// 	struct CANIMURawData imu_raw;

		// 	GyrosGet(&gyrosData);
		// 	AccelsGet(&accelsData);

		// 	imu_raw.time_stamp = time_stamp;

		// 	imu_raw.gyro.x = gyrosData.x;
		// 	imu_raw.gyro.y = gyrosData.y;
		// 	imu_raw.gyro.z = gyrosData.z;

		// 	imu_raw.accel.x = accelsData.x;
		// 	imu_raw.accel.y = accelsData.y;
		// 	imu_raw.accel.z = accelsData.z;

		//     (void)imu_raw;
		// 	// Pixcar_PublishIMUData(&imu_raw);
		// }
		// else
		// 	JLinkRTTPrintf(0, "No IMU data: %ld\n", 0);

		// if (PIOS_Queue_Receive(magQueue, &ev, 0))
		// {
		// 	struct CANMagRawData mag_raw;

		// 	MagnetometerGet(&magData);

		// 	mag_raw.time_stamp = time_stamp;

		// 	mag_raw.mag.x = magData.x;
		// 	mag_raw.mag.y = magData.y;
		// 	mag_raw.mag.z = magData.z;

		// 	(void)mag_raw;
		// 	// Pixcar_PublishMagData(&mag_raw);
		// }
		// else
		// 	JLinkRTTPrintf(0, "No Mag data: %ld\n",0);

		// if (PIOS_Queue_Receive(speedQueue, &ev, 0))
		// {
		// 	struct CANSpeedRawData spd_raw;

		// 	HallSensorGet(&hallData);

		// 	// Send latest speed measurement
		// 	// float speed = calcCarSpeed();
		// 	spd_raw.time_stamp = time_stamp;
		// 	spd_raw.hallsensor_count = hallData.count;
		// 	spd_raw.speed_estimate = hallData.speed_estimate;

		// 	(void)spd_raw;
		// 	// UAVCANNode_PublishSpeedData(&spd_raw);

		// 	// JLinkRTTPrintf(0, "Speed: %ld\n", spd_raw.speed);
		// }
		// else
		// 	JLinkRTTPrintf(0, "No speed data: %ld\n",0);

		// PIOS_WDG_UpdateFlag(PIOS_WDG_MANUAL);
		// PIOS_DELAY_WaitmS(UPDATE_PERIOD_MS);
		// UAVCANNode_SpinNode(UAVCAN_SPIN_TIMEOUT_MS);

		processTxRxOnce();

		if (loop_count++ % 200 == 0)
			updateCANNodeStatus(false);

		PIOS_Thread_Sleep_Until(&sys_time, UPDATE_PERIOD_MS);
	}
}

void updateCANNodeStatus(bool print_mem_stat)
{
	/*
     * Purging transfers that are no longer transmitted. This will occasionally free up some memory.
     */
	canardCleanupStaleTransfers(canard, getMonotonicTimestampUSec());

	/*
	 * Printing the memory usage statistics.
	 */
	if(print_mem_stat)
	{
	    const CanardPoolAllocatorStatistics stats = canardGetPoolAllocatorStatistics(canard);
	    const unsigned peak_percent = 100U * stats.peak_usage_blocks / stats.capacity_blocks;

	    JLinkRTTPrintf(0, "Memory pool stats: capacity %u blocks, usage %u blocks, peak usage %u blocks (%u%%)\n",
	           stats.capacity_blocks, stats.current_usage_blocks, stats.peak_usage_blocks, peak_percent);

	    /*
	     * The recommended way to establish the minimal size of the memory pool is to stress-test the application and
	     * record the worst case memory usage.
	     */
	    if (peak_percent > 70)
	        JLinkRTTPrintf(0, "WARNING: ENLARGE MEMORY POOL", 0);
	}

	/*
     * Transmitting the node status message periodically.
     */
	{
		uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE];
		makeNodeStatusMessage(buffer);

		static uint8_t transfer_id;
		const int bc_res = PIOS_canardBroadcast(canard, UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE,
										   UAVCAN_NODE_STATUS_DATA_TYPE_ID, &transfer_id, CANARD_TRANSFER_PRIORITY_LOW,
										   buffer, UAVCAN_NODE_STATUS_MESSAGE_SIZE);
		if (bc_res <= 0)
			JLinkRTTPrintf(0, "Could not broadcast node status; error %d\n", bc_res);
	}

	/*
	 * Set node mode to be operational.
	 */
	node_mode = UAVCAN_NODE_MODE_OPERATIONAL;
}

void makeNodeStatusMessage(uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE])
{
	memset(buffer, 0, UAVCAN_NODE_STATUS_MESSAGE_SIZE);

	static uint32_t started_at_sec = 0;
	if (started_at_sec == 0)
	{
		started_at_sec = (uint32_t)(getMonotonicTimestampUSec() / 1000000U);
	}

	const uint32_t uptime_sec = (uint32_t)((getMonotonicTimestampUSec() / 1000000U) - started_at_sec);

	/*
     * Here we're using the helper for demonstrational purposes; in this simple case it could be preferred to
     * encode the values manually.
     */
	canardEncodeScalar(buffer, 0, 32, &uptime_sec);
	canardEncodeScalar(buffer, 32, 2, &node_health);
	canardEncodeScalar(buffer, 34, 3, &node_mode);
}

void processTxRxOnce()
{
	// Transmitting
	for (const CanardCANFrame *txf = NULL; (txf = canardPeekTxQueue(canard)) != NULL;)
	{
		const int tx_res = PIOS_canardTransmit(txf);
		if (tx_res < 0) // Failure - drop the frame and report
		{
			canardPopTxQueue(canard);
			JLinkRTTPrintf(0, "Transmit error %d, frame dropped\n", tx_res);
		}
		else if (tx_res > 0) // Success - just drop the frame
		{
			canardPopTxQueue(canard);
		}
		else // Timeout - just exit and try again later
		{
			break;
		}
	}

	// Receiving handled from CAN RX IRQ
}
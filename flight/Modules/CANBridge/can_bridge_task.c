#include "openpilot.h"
#include "pios_queue.h"
#include "pios_thread.h"
#include "can_bridge_task.h"

// UAVOs
#include "gyros.h"
#include "accels.h"
    
#include "jlink_rtt.h"

bool CANBridge_InitComm();
void CANBridge_UpdateComm(bool sensor_updated, struct CANIMURawData *gyro, struct CANIMURawData *accel);

// Private constants
#if defined(PIOS_MANUAL_STACK_SIZE)
#define STACK_SIZE_BYTES PIOS_MANUAL_STACK_SIZE
#else
#define STACK_SIZE_BYTES 9600
#endif

#define TASK_PRIORITY PIOS_THREAD_PRIO_HIGH
#define UPDATE_PERIOD_MS 500
#define FAILSAFE_TIMEOUT_MS 10

#define CAN_RX_TIMEOUT_MS 10

// Private variables
static struct pios_thread *taskHandle;

// sensor queues
static struct pios_queue *gyroQueue;
static struct pios_queue *accelQueue;

// Private functions
static void canBridgeTask(void *parameters);

/**
 * Module starting
 */
int32_t CANBridgeStart()
{
	// Create the queues for the sensors
	gyroQueue = PIOS_Queue_Create(1, sizeof(UAVObjEvent));
	accelQueue = PIOS_Queue_Create(1, sizeof(UAVObjEvent));

	GyrosConnectQueue(gyroQueue);
	AccelsConnectQueue(accelQueue);

	// Watchdog must be registered before starting task
	// PIOS_WDG_RegisterFlag(PIOS_WDG_MANUAL);

	/* Delay system */
	PIOS_DELAY_Init();
	CANBridge_InitComm();

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

	// queue = PIOS_CAN_RegisterMessageQueue(pios_can_id, PIOS_CAN_GIMBAL);

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

    while (1)
	{
		bool sensor_updated = true;
		bool gyroTimeout  = PIOS_Queue_Receive(gyroQueue, &ev, FAILSAFE_TIMEOUT_MS) != true;
		bool accelTimeout = PIOS_Queue_Receive(accelQueue, &ev, 1) != true;

		// When one of these is updated so should the other.
		if (gyroTimeout || accelTimeout) {
			// failed to get new sensor data, do not send to CAN bus
			sensor_updated = false;
		}

		// Send sensor data to CAN bus if updated
		struct CANIMURawData gyro_can, accel_can;
		if(sensor_updated)
		{
			GyrosGet(&gyrosData);
			AccelsGet(&accelsData);			
			
			gyro_can.x = gyrosData.x;
			gyro_can.y = gyrosData.y;
			gyro_can.z = gyrosData.z;

			accel_can.x = accelsData.x;
			accel_can.y = accelsData.y;
			accel_can.z = accelsData.z;
		}

		CANBridge_UpdateComm(sensor_updated, &gyro_can, &accel_can);
		// PIOS_WDG_UpdateFlag(PIOS_WDG_MANUAL);
		PIOS_DELAY_WaitmS(UPDATE_PERIOD_MS);
    }
}

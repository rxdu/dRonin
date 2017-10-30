#include "openpilot.h"
#include "pios_queue.h"
#include "pios_thread.h"
#include "can_bridge_task.h"

// UAVOs
#include "gyros.h"
#include "accels.h"
    
#include "jlink_rtt.h"

bool CANBridge_InitComm();
void CANBridge_UpdateComm(bool sensor_updated, struct CANIMURawData *gyro, struct CANIMURawData *accel, float * speed);

// Private constants
#if defined(PIOS_MANUAL_STACK_SIZE)
#define STACK_SIZE_BYTES PIOS_MANUAL_STACK_SIZE
#else
#define STACK_SIZE_BYTES 9600
#endif

#define TASK_PRIORITY PIOS_THREAD_PRIO_HIGH
#define UPDATE_PERIOD_MS 5
#define FAILSAFE_TIMEOUT_MS 10

#define CAN_RX_TIMEOUT_MS 10
#define CAN_CMD_QUEUE_LEN 2

// Parameters for speed estimation
#define WHEEL_DIAMETER 0.065
#define GEAR_RATIO	6.58	// with 20T pinion gear

// Private variables
static struct pios_thread *taskHandle;

// cmd queue (from CAN bus)
// static struct pios_queue *canCmdQueue;
static struct CANCmdData cmd_from_can;

// sensor queues
static struct pios_queue *gyroQueue;
static struct pios_queue *accelQueue;

// Private functions
static void canBridgeTask(void *parameters);
static float calcCarSpeed(void);

void updateCmdFromCAN(float servo_cmd, float motor_cmd)
{
	// struct CANCmdData cmd_data;
	// cmd_data.servo = servo_cmd;
	// cmd_data.motor = motor_cmd;
	// if(PIOS_Queue_Send(canCmdQueue, &cmd_data, 0))
	// {
	// 	JLinkRTTPrintf(0, "CMD queue sent: %d, %d\n",(int32_t)(cmd_data.servo*100), (int32_t)(cmd_data.motor*100));
	// }
	cmd_from_can.servo = servo_cmd;
	cmd_from_can.motor = motor_cmd;
}

void getCmdFromCAN(float* servo_cmd, float* motor_cmd)
{
	*servo_cmd = cmd_from_can.servo;
	*motor_cmd = cmd_from_can.motor;
}

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

	// Make sure CAN commands initialized with safe value
	cmd_from_can.servo = 0;
	cmd_from_can.motor = 0;

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
	// canCmdQueue = PIOS_Queue_Create(CAN_CMD_QUEUE_LEN, sizeof(struct CANCmdData));
	// if (canCmdQueue == NULL)
	// 	return -1;

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

		// Send IMU sensor data to CAN bus if updated
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

		// Send latest speed measurement
		float speed = calcCarSpeed();

		CANBridge_UpdateComm(sensor_updated, &gyro_can, &accel_can, &speed);
		// PIOS_WDG_UpdateFlag(PIOS_WDG_MANUAL);
		PIOS_DELAY_WaitmS(UPDATE_PERIOD_MS);
    }
}

static float calcCarSpeed()
{
	uint16_t hall_reading = PIOS_TIM_GetHallSensorReading();
	// JLinkRTTPrintf(0, "hall reading: %ld\n",hall_reading);
	float speed = 1.0e6/(hall_reading * 6.0)/GEAR_RATIO*(M_PI*WHEEL_DIAMETER);
	return speed;
}
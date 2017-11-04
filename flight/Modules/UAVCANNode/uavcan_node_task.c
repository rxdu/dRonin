#include "openpilot.h"
#include "pios_queue.h"
#include "pios_thread.h"
#include "can_bridge_task.h"

// UAVOs
#include "gyros.h"
#include "accels.h"
#include "magnetometer.h"
    
#include "jlink_rtt.h"

bool CANBridge_InitComm();
void CANBridge_UpdateComm(struct CANIMURawData *imu_data, float * speed, int32_t spin_timeout);

// Private constants
#if defined(PIOS_MANUAL_STACK_SIZE)
#define STACK_SIZE_BYTES PIOS_MANUAL_STACK_SIZE
#else
#define STACK_SIZE_BYTES 9600
#endif

#define TASK_PRIORITY PIOS_THREAD_PRIO_HIGH

#define FAILSAFE_TIMEOUT_MS 1
#define UAVCAN_SPIN_TIME 2
#define UPDATE_PERIOD_MS (5-UAVCAN_SPIN_TIME)
#define UPDATE_PERIOD_US 5000

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
static struct pios_queue *magQueue;

// Private functions
static void uavcanNodeTask(void *parameters);
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

void resetCmdFromCAN(void)
{
	cmd_from_can.servo = 0;
	cmd_from_can.motor = 0;
}

/**
 * Module starting
 */
int32_t UAVCANNodeStart()
{
	// Create the queues for the sensors
	gyroQueue = PIOS_Queue_Create(1, sizeof(UAVObjEvent));
	accelQueue = PIOS_Queue_Create(1, sizeof(UAVObjEvent));
	magQueue = PIOS_Queue_Create(1, sizeof(UAVObjEvent));

	GyrosConnectQueue(gyroQueue);
	AccelsConnectQueue(accelQueue);
	if (MagnetometerHandle())
		MagnetometerConnectQueue(magQueue);

	// Watchdog must be registered before starting task
	// PIOS_WDG_RegisterFlag(PIOS_WDG_MANUAL);

	/* Delay system */
	PIOS_DELAY_Init();
	UAVCANNode_InitComm();

	// Make sure CAN commands initialized with safe value
	cmd_from_can.servo = 0;
	cmd_from_can.motor = 0;

	// Start main task
	taskHandle = PIOS_Thread_Create(uavcanNodeTask, "UAVCANNode", STACK_SIZE_BYTES, NULL, TASK_PRIORITY);
	//TaskMonitorAdd(TASKINFO_RUNNING_MANUALCONTROL, taskHandle);

	return 0;
}

/**
 * Module initialization
 */
int32_t UAVCANNodeInitialize()
{
	// canCmdQueue = PIOS_Queue_Create(CAN_CMD_QUEUE_LEN, sizeof(struct CANCmdData));
	// if (canCmdQueue == NULL)
	// 	return -1;

	return 0;
}

MODULE_HIPRI_INITCALL(UAVCANNodeInitialize, UAVCANNodeStart);

/**
 * Module task
 */
static void uavcanNodeTask(void *parameters)
{
	UAVObjEvent ev;
	GyrosData gyrosData;
	AccelsData accelsData;
	MagnetometerData magData;

	gyrosData.x = 0;
	gyrosData.y = 0;
	gyrosData.z = 0;

	accelsData.x = 0;
	accelsData.y = 0;
	accelsData.z = 9.8;

	magData.x = 100;
	magData.y = 0;
	magData.z = 0;

    while (1)
	{
		uint32_t start_time = PIOS_DELAY_GetuS();

		bool gyro_accel_updated = true;
		bool gyroTimeout  = PIOS_Queue_Receive(gyroQueue, &ev, FAILSAFE_TIMEOUT_MS) != true;
		bool accelTimeout = PIOS_Queue_Receive(accelQueue, &ev, 1) != true;

		// When one of these is updated so should the other.
		if (gyroTimeout || accelTimeout) {
			// failed to get new sensor data, do not send to CAN bus
			gyro_accel_updated = false;
		}

		// Send IMU sensor data to CAN bus if updated
		struct CANIMURawData imu_raw;
		if(gyro_accel_updated)
		{
			GyrosGet(&gyrosData);
			AccelsGet(&accelsData);			
			
			imu_raw.gyro_accel_updated = true;

			imu_raw.gyro.x = gyrosData.x;
			imu_raw.gyro.y = gyrosData.y;
			imu_raw.gyro.z = gyrosData.z;

			imu_raw.accel.x = accelsData.x;
			imu_raw.accel.y = accelsData.y;
			imu_raw.accel.z = accelsData.z;
		}
		else
		{	
			imu_raw.gyro_accel_updated = false;
		}
		
		if(PIOS_Queue_Receive(magQueue, &ev, 1))
		{
			MagnetometerGet(&magData);

			imu_raw.mag_updated = true;

			imu_raw.mag.x = magData.x;
			imu_raw.mag.y = magData.y;
			imu_raw.mag.z = magData.z;
		}
		else 
		{
			imu_raw.mag_updated = false;
		}

		// Send latest speed measurement
		float speed = calcCarSpeed();

		CANBridge_UpdateComm(&imu_raw, &speed, UAVCAN_SPIN_TIME);
		// PIOS_WDG_UpdateFlag(PIOS_WDG_MANUAL);
		// PIOS_DELAY_WaitmS(UPDATE_PERIOD_MS);
		
		uint32_t spent_time = PIOS_DELAY_GetuSSince(start_time);
		if(spent_time < UPDATE_PERIOD_US)
			PIOS_DELAY_WaituS(UPDATE_PERIOD_US - spent_time);
    }
}

static float calcCarSpeed()
{
	uint16_t hall_reading = PIOS_TIM_GetHallSensorReading();
	// JLinkRTTPrintf(0, "hall reading: %ld\n",hall_reading);
	float speed = 1.0e6/(hall_reading * 6.0)/GEAR_RATIO*(M_PI*WHEEL_DIAMETER);
	return speed;
}
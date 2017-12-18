#include "openpilot.h"
#include "pios_thread.h"
#include "pios_queue.h"
#include "pios_semaphore.h"

#include "carnavigationdesired.h"
#include "hallsensor.h"

#include "pixcar.h"
#include "jlink_rtt.h"

extern uintptr_t pios_can_id;

// Private constants
#if defined(PIOS_MANUAL_STACK_SIZE)
#define STACK_SIZE_BYTES PIOS_MANUAL_STACK_SIZE
#else
#define STACK_SIZE_BYTES 1200
#endif

#define TASK_PRIORITY PIOS_THREAD_PRIO_LOW
#define UPDATE_PERIOD_MS 500

#define CAN_RX_TIMEOUT_MS 10

#define HALL_SENSOR_QUEUE_SIZE 1
#define CAN_CMD_QUEUE_SIZE 1 

// Private variables
static uint32_t idleCounter;
static uint32_t idleCounterClear;

static struct pios_can_cmd_data prev_can_cmd_data;

static struct pios_queue *hallsensor_queue;
static struct pios_queue *navigation_desired_queue;

static struct pios_thread *taskHandle;

// Private functions
static void pixCarTask(void *parameters);

/**
 * Module starting
 */
int32_t PixCarTaskStart()
{
	// Watchdog must be registered before starting task
	//PIOS_WDG_RegisterFlag(PIOS_WDG_MANUAL);

	// Start main task
	taskHandle = PIOS_Thread_Create(pixCarTask, "pixCarTask", STACK_SIZE_BYTES, NULL, TASK_PRIORITY);
	//TaskMonitorAdd(TASKINFO_RUNNING_MANUALCONTROL, taskHandle);

	return 0;
}

/**
 * Module initialization
 */
int32_t PixCarTaskInitialize()
{
	prev_can_cmd_data.steering = 0.0;
	prev_can_cmd_data.throttle = 0.0;		

	navigation_desired_queue = PIOS_Queue_Create(CAN_CMD_QUEUE_SIZE, sizeof(UAVObjEvent));
	CarNavigationDesiredConnectQueue(navigation_desired_queue);

	return 0;
}

MODULE_HIPRI_INITCALL(PixCarTaskInitialize, PixCarTaskStart);

/**
 * Module task
 */
static void pixCarTask(void *parameters)
{
	while (1)
	{
		PIOS_DELAY_WaitmS(UPDATE_PERIOD_MS);
	}
}


struct pios_queue *PIXCAR_GetHallSensorQueue(void)
{
	if(hallsensor_queue == NULL)
		hallsensor_queue = PIOS_Queue_Create(HALL_SENSOR_QUEUE_SIZE, sizeof(struct pios_sensor_hallsensor_data));

	return hallsensor_queue;
}

void PIXCAR_ResetNavigationDesired()
{
	CarNavigationDesiredData nav_desired;
	CarNavigationDesiredGet(&nav_desired);

	nav_desired.Steering = 0;
	nav_desired.Throttle = 0;

	CarNavigationDesiredSet(&nav_desired);
}

void PIXCAR_SetNavigationDesired(struct pios_can_cmd_data * cmd)
{
	CarNavigationDesiredData nav_desired;
	CarNavigationDesiredGet(&nav_desired);

	nav_desired.Steering = cmd->steering;
	nav_desired.Throttle = cmd->throttle;

	CarNavigationDesiredSet(&nav_desired);

	// JLinkRTTPrintf(0, "Set navigation desired from CAN: %d, %d\n",(int32_t)(nav_desired.Steering*100), (int32_t)(nav_desired.Throttle*100));
}

void PIXCAR_GetNavigationDesired(struct pios_can_cmd_data * cmd)
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

float PIXCAR_UpdateCarSpeed(uint16_t hall_count)
{
	float latest_speed = 1.0e6/(hall_count * 6.0)/GEAR_RATIO*(M_PI*WHEEL_DIAMETER);
	float speed_est = latest_speed;

	return speed_est;
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

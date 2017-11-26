#include "openpilot.h"
#include "pios_thread.h"
#include "pios_queue.h"
#include "pios_semaphore.h"

#include "carnavigationdesired.h"
#include "hallsensor.h"

#include "pixcar.h"
#include "jlink_rtt.h"

#define HALL_SENSOR_QUEUE_SIZE 1
#define CAN_CMD_QUEUE_SIZE 2 

static uint32_t idleCounter;
static uint32_t idleCounterClear;

static struct pios_queue *hallsensor_queue;

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
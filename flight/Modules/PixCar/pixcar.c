#include "openpilot.h"
#include "pios_thread.h"
#include "pios_queue.h"
#include "pios_semaphore.h"

#include "hallsensor.h"

#include "pixcar.h"

#define HALL_SENSOR_QUEUE_SIZE 1

static struct pios_queue *hallsensor_queue;

struct pios_queue *PIXCAR_GetHallSensorQueue(void)
{
	if(hallsensor_queue == NULL)
		hallsensor_queue = PIOS_Queue_Create(HALL_SENSOR_QUEUE_SIZE, sizeof(struct pios_sensor_hallsensor_data));

	return hallsensor_queue;
}

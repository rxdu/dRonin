#include "openpilot.h"
#include "pios_thread.h"
#include "pios_queue.h"
#include "pios_semaphore.h"

#include "hallsensor.h"

#include "pixcar.h"

void updateHallSensorData(uint16_t count)
{
    HallSensorData hallsensorData;
	hallsensorData.count = count;
	HallSensorSet(&hallsensorData);
}


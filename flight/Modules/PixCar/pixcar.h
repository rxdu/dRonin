/* 
 * pixcar.h
 * 
 * Created on: Nov 04, 2017 21:09
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef PIXCAR_H
#define PIXCAR_H

#include "pios.h"
#include "stdint.h"
#include "pios_queue.h"

// Parameters for speed estimation
#define WHEEL_DIAMETER 0.065
#define GEAR_RATIO 6.58 // with 20T pinion gear

struct pios_sensor_hallsensor_data {
	uint16_t count;
};

struct pios_can_cmd_data {
	float steering;
	float throttle;
};

// Hall sensor data is updated from timer IRQ handler using queue
struct pios_queue *PIXCAR_GetHallSensorQueue(void);
float PIXCAR_UpdateCarSpeed(uint16_t hall_count);

// The set and reset functions are called by the CAN node or transmitter control node
void PIXCAR_ResetNavigationDesired();
void PIXCAR_SetNavigationDesired(struct pios_can_cmd_data * cmd);
// This function is used the get latest command from CAN bus
void PIXCAR_GetNavigationDesired(struct pios_can_cmd_data * cmd);

#endif /* PIXCAR_H */

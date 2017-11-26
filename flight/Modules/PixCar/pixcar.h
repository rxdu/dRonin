/* 
 * pixcar.h
 * 
 * Created on: Nov 04, 2017 21:09
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef PIXCAR_H
#define PIXCAR_H

#include "pios.h"
#include "stdint.h"
#include "pios_queue.h"

struct pios_sensor_hallsensor_data {
	uint16_t count;
};

struct pios_can_cmd_data {
	float steering;
	float throttle;
};

struct pios_queue *PIXCAR_GetHallSensorQueue(void);

// The set and reset functions are called by the CAN node or transmitter control node
void PIXCAR_ResetNavigationDesired();
void PIXCAR_SetNavigationDesired(struct pios_can_cmd_data * cmd);
// This function is used the get latest command from CAN bus
void PIXCAR_GetNavigationDesired(struct pios_can_cmd_data * cmd);

#endif /* PIXCAR_H */

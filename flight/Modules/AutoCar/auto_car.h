/* 
 * auto_car.h
 * 
 * Created on: Nov 04, 2017 21:09
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef AUTO_CAR_H
#define AUTO_CAR_H

#include "pios.h"
#include "stdint.h"
#include "pios_queue.h"

// Parameters for speed estimation
#define WHEEL_DIAMETER 		0.065
#define GEAR_RATIO 			6.58 // with 20T pinion gear
#define TRIGGERS_PER_ROUND 	6.0

struct pios_sensor_hallsensor_data {
	uint16_t count;
};

struct pios_can_cmd_data {
	float steering;
	float throttle;
};

// Hall sensor data is updated from timer IRQ handler using queue
struct pios_queue *AutoCarGetHallSensorQueue(void);
float AutoCarUpdateCarSpeed(uint16_t hall_count);
void AutoCarUpdateHallsensorData(uint16_t hall_count);

// The set and reset functions are called by the CAN node or transmitter control node
void AutoCarResetNavigationDesired();
void AutoCarSetNavigationDesired(struct pios_can_cmd_data * cmd);
// This function is used the get latest command from CAN bus
void AutoCarGetNavigationDesired(struct pios_can_cmd_data * cmd);

#endif /* AUTO_CAR_H */

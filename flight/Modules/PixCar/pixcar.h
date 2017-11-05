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

// struct pios_queue *PIXCAR_GetCANCmdQueue();

void updateHallSensorData(uint16_t count);

#endif /* PIXCAR_H */

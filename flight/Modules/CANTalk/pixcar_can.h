/* 
 * pixcar_can.h
 * 
 * Created on: Jan 01, 2018 23:34
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef PIXCAR_CAN_H
#define PIXCAR_CAN_H

#include "stdint.h"

// Single-frame data from UAVCAN
#define CANTALK_PIXCAR_CARSPEED_DATA_TYPE_ID        22026242
#define CANTALK_PIXCAR_CARCOMMAND_DATA_TYPE_ID      22025474
#define CANTALK_PIXCAR_SBCHEARTBEAT_DATA_TYPE_ID    268522754

struct CANCmdData
{
    float servo;
    float motor;
};

struct CANSpeedRawData
{
    uint32_t time_stamp;
    uint32_t hallsensor_count;
    float speed_estimate;
};

void Pixcar_PublishSpeedData(struct CANSpeedRawData *spd_data);
void Pixcar_CanTalkReceive(uint32_t id, const uint8_t *data, uint8_t data_len);

#endif /* PIXCAR_CAN_H */

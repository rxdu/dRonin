/* 
 * can_talk.h
 * 
 * Created on: Jan 01, 2018 23:34
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef CAN_TALK_H
#define CAN_TALK_H

#include "stdint.h"

// Smaller ID -> Higher priority
#define CANTALK_AUTOCAR_MCUHEARTBEAT_DATA_TYPE_ID    3301
#define CANTALK_AUTOCAR_SBCHEARTBEAT_DATA_TYPE_ID    3302

#define CANTALK_AUTOCAR_CARCOMMAND_DATA_TYPE_ID      3311
#define CANTALK_AUTOCAR_CARSPEED_DATA_TYPE_ID        3312

struct CANCmdData
{
    float servo;
    float motor;
};

struct CANSpeedRawData
{
    uint32_t time_stamp;
    float speed_estimate;
};

void AutoCarPublishHeartbeat();
void AutoCarPublishSpeedData(struct CANSpeedRawData *spd_data);
void AutoCarReceiveCANMessage(uint32_t id, const uint8_t *data, uint8_t data_len);

#endif /* CAN_TALK_H */

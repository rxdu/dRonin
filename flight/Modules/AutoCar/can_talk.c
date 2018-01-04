/* 
 * can_talk.c
 * 
 * Created on: Jan 01, 2018 23:35
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include "can_talk.h"
#include "jlink_rtt.h"
#include "auto_car.h"

void AutoCarPublishSpeedData(struct CANSpeedRawData *spd_data)
{
    // make can message binary compatible with uavcan
    uint8_t can_frame[8];
    static uint8_t dummy_byte = 0;
    can_frame[0] = 0x000000ff & ((uint32_t)spd_data->time_stamp);
    can_frame[1] = 0x000000ff & ((uint32_t)spd_data->time_stamp >> 8);
    can_frame[2] = 0x000000ff & ((uint32_t)spd_data->time_stamp >> 16);
    can_frame[3] = 0x000000ff & ((uint32_t)spd_data->time_stamp >> 24);
    can_frame[4] = dummy_byte++;

    PIOS_CAN_TxCANFrame(CANTALK_AUTOCAR_CARSPEED_DATA_TYPE_ID, true, can_frame, 5);
}

void AutoCarReceiveCANMessage(uint32_t id, const uint8_t *data, uint8_t data_len)
{
    int8_t servo_cmd = 0;
    int8_t motor_cmd = 0;
    struct pios_can_cmd_data cmd;

    switch(id)
    {
        case CANTALK_AUTOCAR_CARCOMMAND_DATA_TYPE_ID:            
            servo_cmd = (int8_t)data[0];
            motor_cmd = (int8_t)data[1];           
            cmd.steering = servo_cmd / 100.0;
            cmd.throttle = motor_cmd / 100.0;
            AutoCarSetNavigationDesired(&cmd);

            JLinkRTTPrintf(0, "Received car_command, payload lenght %d, byte 1: %d , byte 2: %d\n", data_len, servo_cmd, motor_cmd);
            break;
        case CANTALK_AUTOCAR_SBCHEARTBEAT_DATA_TYPE_ID:
            JLinkRTTPrintf(0, "Received heatbeat from single-board computer\n", 0);
            break;
        default:
            JLinkRTTPrintf(0, "Received unknown can msg of %d bytes from %d \n", data_len, id);
            break;
    }
}

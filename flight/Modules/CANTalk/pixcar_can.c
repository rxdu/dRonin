/* 
 * pixcar_can.c
 * 
 * Created on: Jan 01, 2018 23:35
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include "pixcar_can.h"
#include "jlink_rtt.h"
#include "pixcar.h"

void Pixcar_PublishSpeedData(struct CANSpeedRawData *spd_data)
{
    uint8_t can_frame[8];

    can_frame[0] = 0x000000ff & (spd_data->time_stamp);
    can_frame[1] = 0x000000ff & (spd_data->time_stamp >> 8);
    can_frame[2] = 0x000000ff & (spd_data->time_stamp >> 16);
    can_frame[3] = 0x000000ff & (spd_data->time_stamp >> 24);

    can_frame[4] = 0x000000ff & (spd_data->speed_estimate);
    can_frame[5] = 0x000000ff & (spd_data->speed_estimate >> 8);
    can_frame[6] = 0x000000ff & (spd_data->speed_estimate >> 16);
    can_frame[7] = 0x000000ff & (spd_data->speed_estimate >> 24);

    // PIOS_CAN_TxCANFrame()
}

void Pixcar_CanTalkReceive(uint32_t id, const uint8_t *data, uint8_t data_len)
{
    int8_t servo_cmd = 0;
    int8_t motor_cmd = 0;
    struct pios_can_cmd_data cmd;

    switch(id)
    {
        case CANTALK_PIXCAR_CARCOMMAND_DATA_TYPE_ID:            
            servo_cmd = (int8_t)data[0];
            motor_cmd = (int8_t)data[1];           
            cmd.steering = servo_cmd / 100.0;
            cmd.throttle = motor_cmd / 100.0;
            PIXCAR_SetNavigationDesired(&cmd);

            JLinkRTTPrintf(0, "Received car_command, payload lenght %d, byte 1: %d , byte 2: %d\n", data_len, servo_cmd, motor_cmd);
            break;
        case CANTALK_PIXCAR_SBCHEARTBEAT_DATA_TYPE_ID:
            JLinkRTTPrintf(0, "Received heatbeat from single-board computer\n", 0);
            break;
        default:
            JLinkRTTPrintf(0, "Received unknown can msg of %d bytes from %d \n", data_len, id);
            break;
    }
}

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

static bool CheckCommandValid(float cmd)
{
    if (cmd >= -1.0f && cmd <= 1.0f)
        return true;
    else
        return false;
}

void AutoCarPublishHeartbeat()
{
    uint8_t can_frame[8];
    static uint8_t dummy_byte = 0;
    can_frame[0] = 1;
    can_frame[1] = dummy_byte++;

    PIOS_CAN_TxCANFrame(CANTALK_AUTOCAR_MCUHEARTBEAT_DATA_TYPE_ID, true, can_frame, 2);
}

void AutoCarPublishSpeedData(struct CANSpeedRawData *spd_data)
{
    uint8_t can_frame[8];
    uint32_t speed_bits;
    static uint8_t dummy_byte = 0;

    // encode float: LSByte first, MSBtye last 
    memcpy(&speed_bits, &spd_data->speed_estimate, sizeof(speed_bits));
    can_frame[0] = 0x000000ff & (speed_bits);
    can_frame[1] = 0x000000ff & (speed_bits >> 8);
    can_frame[2] = 0x000000ff & (speed_bits >> 16);
    can_frame[3] = 0x000000ff & (speed_bits >> 24);
    can_frame[4] = dummy_byte++;

    PIOS_CAN_TxCANFrame(CANTALK_AUTOCAR_CARSPEED_DATA_TYPE_ID, true, can_frame, 5);
}

void AutoCarReceiveCANMessage(uint32_t id, const uint8_t *data, uint8_t data_len)
{
    int8_t servo_cmd = 0;
    int8_t motor_cmd = 0;
    struct pios_can_cmd_data cmd;

    switch (id)
    {
    case CANTALK_AUTOCAR_CARCOMMAND_DATA_TYPE_ID:
        cmd.update_flags = data[0];
        servo_cmd = (int8_t)data[1];
        motor_cmd = (int8_t)data[2];
        cmd.steering = (float)servo_cmd / 100.0f;
        cmd.throttle = (float)motor_cmd / 100.0f;
        // make sure command stays within valid range, otherwise discard
        if (CheckCommandValid(cmd.steering) && CheckCommandValid(cmd.throttle))
            AutoCarSetNavigationDesired(&cmd);
        // JLinkRTTPrintf(0, "Received car_command, payload lenght %d, byte 1: %d , byte 2: %d\n", data_len, servo_cmd, motor_cmd);
        break;
    case CANTALK_AUTOCAR_SBCHEARTBEAT_DATA_TYPE_ID:
        JLinkRTTPrintf(0, "Received heatbeat from single-board computer\n", 0);
        break;
    default:
        JLinkRTTPrintf(0, "Received unknown can msg of %d bytes from %d \n", data_len, id);
        break;
    }
}

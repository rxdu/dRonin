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

#include "canard.h"

// Pixcar Node run on SBC
#define UAVCAN_PIXCAR_SBC_NODE_ID 2

/*
 * User defined UAVCAN message types
 */
#define UAVCAN_PIXCAR_CARCOMMAND_DATA_TYPE_ID   25001

#define UAVCAN_PIXCAR_CARRAWIMU_DATA_TYPE_ID    25002
#define UAVCAN_PIXCAR_CARRAWIMU_MESSAGE_SIZE    28
#define UAVCAN_PIXCAR_CARRAWIMU_DATA_TYPE_SIGNATURE 0x938CF09578C5A762ULL

#define UAVCAN_PIXCAR_CARRAWMAG_DATA_TYPE_ID    25003
#define UAVCAN_PIXCAR_CARRAWMAG_MESSAGE_SIZE    16
#define UAVCAN_PIXCAR_CARRAWMAG_DATA_TYPE_SIGNATURE 0xC950CD0CDB7EAD62ULL

/*
 * Data structures used in CanTalk node
 */
#define IMU_MSG_PRIORITY 2
#define MAG_MSG_PRIORITY 3
#define SPD_MSG_PRIORITY 4

struct CANSensorData3Axis
{
    float x;
    float y;
    float z;
};

struct CANCmdData
{
    float servo;
    float motor;
};

struct CANIMURawData
{
    uint32_t time_stamp;
    struct CANSensorData3Axis gyro;
    struct CANSensorData3Axis accel;
};

struct CANMagRawData
{
    uint32_t time_stamp;
    struct CANSensorData3Axis mag;
};

struct CANSpeedRawData
{
    uint32_t time_stamp;
    uint32_t hallsensor_count;
    float speed_estimate;
};

void Pixcar_ReceiveCarCommand(CanardRxTransfer *transfer);
void Pixcar_PublishIMUData(struct CANIMURawData *imu_data);
void Pixcar_PublishMagData(struct CANMagRawData *mag_data);
void Pixcar_PublishSpeedData(struct CANSpeedRawData *spd_data);

#endif /* PIXCAR_CAN_H */

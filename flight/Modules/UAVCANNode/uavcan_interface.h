/* 
 * uavcan_interface.h
 * 
 * Created on: Nov 04, 2017 17:12
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#ifndef UAVCAN_INTERFACE_H
#define UAVCAN_INTERFACE_H

#include <stdint.h>

#define IMU_MSG_PRIORITY    2
#define MAG_MSG_PRIORITY    3
#define SPD_MSG_PRIORITY    4

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

void getCmdFromCAN(float* servo_cmd, float* motor_cmd);
void resetCmdFromCAN(void);

void UAVCANNode_PublishIMUData(struct CANIMURawData *imu_data);
void UAVCANNode_PublishMagData(struct CANMagRawData *mag_data);
void UAVCANNode_PublishSpeedData(struct CANSpeedRawData *spd_data);

#endif /* UAVCAN_INTERFACE_H */

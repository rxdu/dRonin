/* 
 * pixcar_can.c
 * 
 * Created on: Jan 01, 2018 23:35
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include "pios_canard.h"
#include "pixcar_can.h"
#include "jlink_rtt.h"
#include "pixcar.h"

static void makeIMUDataMessage(struct CANIMURawData *imu_data, uint8_t buffer[UAVCAN_PIXCAR_CARRAWIMU_MESSAGE_SIZE]);
static void makeMagDataMessage(struct CANMagRawData *mag_data, uint8_t buffer[UAVCAN_PIXCAR_CARRAWMAG_MESSAGE_SIZE]);

void Pixcar_ReceiveCarCommand(CanardRxTransfer *transfer)
{
    if ((transfer->source_node_id == UAVCAN_PIXCAR_SBC_NODE_ID) &&
        (transfer->data_type_id == UAVCAN_PIXCAR_CARCOMMAND_DATA_TYPE_ID))
    {
        int8_t servo_cmd = 0;
        int8_t motor_cmd = 0;
        canardDecodeScalar(transfer, 0, 8, true, &servo_cmd);
        canardDecodeScalar(transfer, 8, 8, true, &motor_cmd);

        struct pios_can_cmd_data cmd;
        cmd.steering = servo_cmd / 100.0;
        cmd.throttle = motor_cmd / 100.0;
        PIXCAR_SetNavigationDesired(&cmd);

        JLinkRTTPrintf(0, "Received car_command, payload lenght %d, byte 1: %d , byte 2: %d\n", transfer->payload_len, servo_cmd, motor_cmd);
    }
}

static void makeIMUDataMessage(struct CANIMURawData *imu_data, uint8_t buffer[UAVCAN_PIXCAR_CARRAWIMU_MESSAGE_SIZE])
{
    memset(buffer, 0, UAVCAN_PIXCAR_CARRAWIMU_MESSAGE_SIZE);
	
	canardEncodeScalar(buffer, 0, 32, &imu_data->time_stamp);
	canardEncodeScalar(buffer, 32, 32, &imu_data->gyro.x);
    canardEncodeScalar(buffer, 64, 32, &imu_data->gyro.y);
    canardEncodeScalar(buffer, 96, 32, &imu_data->gyro.z);
    canardEncodeScalar(buffer, 128, 32, &imu_data->accel.x);
    canardEncodeScalar(buffer, 160, 32, &imu_data->accel.y);
    canardEncodeScalar(buffer, 192, 32, &imu_data->accel.z);
}

static void makeMagDataMessage(struct CANMagRawData *mag_data, uint8_t buffer[UAVCAN_PIXCAR_CARRAWMAG_MESSAGE_SIZE])
{
    memset(buffer, 0, UAVCAN_PIXCAR_CARRAWMAG_MESSAGE_SIZE);

	canardEncodeScalar(buffer, 0, 32, &mag_data->time_stamp);
	canardEncodeScalar(buffer, 32, 32, &mag_data->mag.x);
    canardEncodeScalar(buffer, 64, 32, &mag_data->mag.y);
    canardEncodeScalar(buffer, 96, 32, &mag_data->mag.z);
}

void Pixcar_PublishIMUData(struct CANIMURawData *imu_data)
{
    uint8_t buffer[UAVCAN_PIXCAR_CARRAWIMU_MESSAGE_SIZE];
    makeIMUDataMessage(imu_data, buffer);

    static uint8_t transfer_id;
    const int bc_res = PIOS_canardBroadcast(getCanardInstance(), UAVCAN_PIXCAR_CARRAWIMU_DATA_TYPE_SIGNATURE,
                                            UAVCAN_PIXCAR_CARRAWIMU_DATA_TYPE_ID, &transfer_id, CANARD_TRANSFER_PRIORITY_MEDIUM,
                                            buffer, UAVCAN_PIXCAR_CARRAWIMU_MESSAGE_SIZE);
    if (bc_res <= 0)
        JLinkRTTPrintf(0, "Could not broadcast imu data; error %d\n", bc_res);
}

void Pixcar_PublishMagData(struct CANMagRawData *mag_data)
{
    uint8_t buffer[UAVCAN_PIXCAR_CARRAWMAG_MESSAGE_SIZE];
    makeMagDataMessage(mag_data, buffer);

    static uint8_t transfer_id;
    const int bc_res = PIOS_canardBroadcast(getCanardInstance(), UAVCAN_PIXCAR_CARRAWMAG_DATA_TYPE_SIGNATURE,
                                            UAVCAN_PIXCAR_CARRAWMAG_DATA_TYPE_ID, &transfer_id, CANARD_TRANSFER_PRIORITY_MEDIUM,
                                            buffer, UAVCAN_PIXCAR_CARRAWMAG_MESSAGE_SIZE);
    if (bc_res <= 0)
        JLinkRTTPrintf(0, "Could not broadcast mag data; error %d\n", bc_res);
}

void Pixcar_PublishSpeedData(struct CANSpeedRawData *spd_data)
{
}
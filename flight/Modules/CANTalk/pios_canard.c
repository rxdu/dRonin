/* 
 * pios_canard.c
 * 
 * Created on: Dec 28, 2017 15:42
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include "pios_canard.h"

#include "pios_tim.h"
#include "pios_can.h"

#include "pixcar.h"
#include "jlink_rtt.h"

static CanardInstance canard;

CanardInstance *getCanardInstance()
{
    return &canard;
}

uint64_t getMonotonicTimestampUSec(void)
{
    return PIOS_UAVCAN_TIM_GetMonoTime();
}

int PIOS_canardTransmit(const CanardCANFrame *const frame)
{
    return PIOS_CAN_TxCANFrame(frame->id, true, frame->data, frame->data_len);
}

void PIOS_canardReceive(uint32_t id, const uint8_t *data, uint8_t data_len)
{
    const uint64_t timestamp = getMonotonicTimestampUSec();

    CanardCANFrame rx_frame;
    rx_frame.id = id;
    rx_frame.data_len = data_len;
    for (int i = 0; i < data_len; i++)
        rx_frame.data[i] = data[i];

    canardHandleRxFrame(&canard, &rx_frame, timestamp);
}

void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{
    JLinkRTTPrintf(0, "Received something\n", 0);

    if ((transfer->source_node_id == UAVCAN_PIXCAR_SBC_NODE_ID) &&
        (transfer->data_type_id == UAVCAN_PIXCAR_CARCOMMAND_DATA_TYPE_ID))
    {
        int8_t servo_cmd = 0;
        int8_t motor_cmd = 0;
        canardDecodeScalar(transfer, 0, 8, true, &servo_cmd);
        canardDecodeScalar(transfer, 8, 8, true, &motor_cmd);

        struct pios_can_cmd_data cmd;
        cmd.steering = servo_cmd/100.0;
        cmd.throttle = motor_cmd/100.0;
        PIXCAR_SetNavigationDesired(&cmd);

        JLinkRTTPrintf(0, "Received car_command, payload lenght %d, byte 1: %d , byte 2: %d\n", transfer->payload_len, servo_cmd, motor_cmd);
    }
}

bool shouldAcceptTransfer(const CanardInstance *ins,
                          uint64_t *out_data_type_signature,
                          uint16_t data_type_id,
                          CanardTransferType transfer_type,
                          uint8_t source_node_id)
{
    // (void)source_node_id;
    if (source_node_id == UAVCAN_PIXCAR_SBC_NODE_ID)
    {
        // ignore node status type
        if ((transfer_type == CanardTransferTypeBroadcast) &&
            (data_type_id == UAVCAN_NODE_STATUS_DATA_TYPE_ID))
            return false;
        else
            return true;
    }

    if (canardGetLocalNodeID(ins) == CANARD_BROADCAST_NODE_ID)
    {
        /*
         * If we're in the process of allocation of dynamic node ID, accept only relevant transfers.
         */
        if ((transfer_type == CanardTransferTypeBroadcast) &&
            (data_type_id == UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_ID))
        {
            *out_data_type_signature = UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_SIGNATURE;
            return true;
        }
    }
    else
    {
        if ((transfer_type == CanardTransferTypeRequest) &&
            (data_type_id == UAVCAN_GET_NODE_INFO_DATA_TYPE_ID))
        {
            *out_data_type_signature = UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE;
            return true;
        }
    }

    return false;
}
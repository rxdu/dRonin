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

void PIOS_canardReceive(const CanardCANFrame *frame)
{
    const uint64_t timestamp = getMonotonicTimestampUSec();
    canardHandleRxFrame(&canard, frame, timestamp);
}

void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{
}

bool shouldAcceptTransfer(const CanardInstance *ins,
                          uint64_t *out_data_type_signature,
                          uint16_t data_type_id,
                          CanardTransferType transfer_type,
                          uint8_t source_node_id)
{
    (void)source_node_id;

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
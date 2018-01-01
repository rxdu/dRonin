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
    CanardCANFrame rx_frame;
    rx_frame.id = id;
    rx_frame.data_len = data_len;
    for(int i = 0; i < data_len; i++)
        rx_frame.data[i] = data[i]; 
	const uint64_t timestamp = getMonotonicTimestampUSec();
    canardHandleRxFrame(&canard, &rx_frame, timestamp);
}

void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{
    JLinkRTTPrintf(0, "Received something\n",0);
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
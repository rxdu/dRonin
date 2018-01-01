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

uint64_t getMonotonicTimestampUSec(void)
{
    return PIOS_UAVCAN_TIM_GetMonoTime(); 
}

int PIOS_canardTransmit(const CanardCANFrame* const frame)
{
    return PIOS_CAN_TxCANFrame(frame->id, true, frame->data, frame->data_len);
}

void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer)
{

}

bool shouldAcceptTransfer(const CanardInstance* ins,
                                 uint64_t* out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id)
{
    return true;
}
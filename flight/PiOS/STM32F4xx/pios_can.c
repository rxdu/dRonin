/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_CAN PiOS CAN interface layer
 * @brief CAN interface for PiOS
 * @{
 *
 * @file       pios_can.c
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013-2014
 * @brief      PiOS CAN interface header
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>
 */


#include "pios.h"

#if defined(PIOS_INCLUDE_CAN)

#include "pios_can_priv.h"

#ifdef PIOS_INCLUDE_UAVCAN 
/* Provide a UAVCAN driver */
/* Functions defined in uavcan_pios_driver.cpp */
void PIOSUAVCAN_ResetDriverState();
void PIOSUAVCAN_TxISR_Callback();
void PIOSUAVCAN_RxISR_Callback();

static void PIOS_CAN_TxUAVCAN(void);
static void PIOS_CAN_RxUAVCAN(void);
#else
/* Provide a COM driver */
static void PIOS_CAN_RegisterRxCallback(uintptr_t can_id, pios_com_callback rx_in_cb, uintptr_t context);
static void PIOS_CAN_RegisterTxCallback(uintptr_t can_id, pios_com_callback tx_out_cb, uintptr_t context);
static void PIOS_CAN_TxStart(uintptr_t can_id, uint16_t tx_bytes_avail);
static void PIOS_CAN_RxStart(uintptr_t can_id, uint16_t rx_bytes_avail);

const struct pios_com_driver pios_can_com_driver = {
	.tx_start   = PIOS_CAN_TxStart,
	.rx_start   = PIOS_CAN_RxStart,
	.bind_tx_cb = PIOS_CAN_RegisterTxCallback,
	.bind_rx_cb = PIOS_CAN_RegisterRxCallback,
};

static void PIOS_CAN_RxGeneric(void);
static void PIOS_CAN_TxGeneric(void);
#endif

enum pios_can_dev_magic {
	PIOS_CAN_DEV_MAGIC = 0x41fa834A,
};

//! Structure for an initialized CAN handle
struct pios_can_dev {
	enum pios_can_dev_magic     magic;
	const struct pios_can_cfg  *cfg;
	pios_com_callback rx_in_cb;
	uintptr_t rx_in_context;
	pios_com_callback tx_out_cb;
	uintptr_t tx_out_context;
};

// Local constants
#define CAN_COM_ID      0x11
#define MAX_SEND_LEN    8

static bool PIOS_CAN_validate(struct pios_can_dev *can_dev)
{
	return (can_dev->magic == PIOS_CAN_DEV_MAGIC);
}

static struct pios_can_dev *PIOS_CAN_alloc(void)
{
	struct pios_can_dev *can_dev;

	can_dev = (struct pios_can_dev *)PIOS_malloc(sizeof(*can_dev));
	if (!can_dev) return(NULL);

	memset(can_dev, 0, sizeof(*can_dev));
	can_dev->magic = PIOS_CAN_DEV_MAGIC;

	return(can_dev);
}

//! The local handle for the CAN device
static struct pios_can_dev *can_dev;

/**
 * Initialize the CAN driver and return an opaque id
 * @param[out]   id the CAN interface handle
 * @param[in]    cfg the configuration structure
 * @return 0 if successful, negative otherwise
 */
int32_t PIOS_CAN_Init(uintptr_t *can_id, const struct pios_can_cfg *cfg)
{
	PIOS_DEBUG_Assert(can_id);
	PIOS_DEBUG_Assert(cfg);

	can_dev = (struct pios_can_dev *) PIOS_CAN_alloc();
	if (!can_dev) goto out_fail;

	/* Bind the configuration to the device instance */
	can_dev->cfg = cfg;

	/* Configure the CAN device */
	if (can_dev->cfg->regs == CAN1) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	} else {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
	}

	/* Map pins to CAN function */
	if (can_dev->cfg->remap) {
		if (can_dev->cfg->rx.gpio != 0)
			GPIO_PinAFConfig(can_dev->cfg->rx.gpio,
				can_dev->cfg->rx.pin_source,
				can_dev->cfg->remap);
		if (can_dev->cfg->tx.gpio != 0)
			GPIO_PinAFConfig(can_dev->cfg->tx.gpio,
				can_dev->cfg->tx.pin_source,
				can_dev->cfg->remap);
	}

	/* Initialize the CAN Rx and Tx pins */
	if (can_dev->cfg->rx.gpio != 0)
		GPIO_Init(can_dev->cfg->rx.gpio, (GPIO_InitTypeDef *)&can_dev->cfg->rx.init);
	if (can_dev->cfg->tx.gpio != 0)
		GPIO_Init(can_dev->cfg->tx.gpio, (GPIO_InitTypeDef *)&can_dev->cfg->tx.init);

	*can_id = (uintptr_t)can_dev;

	CAN_DeInit(can_dev->cfg->regs);
	CAN_Init(can_dev->cfg->regs, (CAN_InitTypeDef *)&can_dev->cfg->init);

	/* CAN filter init */
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;  
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 1;

	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);

	// Enable the receiver IRQ
 	NVIC_Init((NVIC_InitTypeDef*) &can_dev->cfg->rx_irq.init);
 	NVIC_Init((NVIC_InitTypeDef*) &can_dev->cfg->tx_irq.init);

#ifdef PIOS_INCLUDE_UAVCAN 
	PIOSUAVCAN_ResetDriverState();
#endif

	return(0);

out_fail:
	return(-1);
}

/**
 * PIOS_CAN_TxData transmits a data message with a specified ID
 * @param[in] id the CAN device ID
 * @param[in] msg_id The message ID (std ID < 0x7FF)
 * @param[in] data Pointer to data message
 * @returns number of bytes sent if successful, -1 if not
 */
int32_t PIOS_CAN_TxData(uintptr_t id, enum pios_can_messages msg_id, uint8_t *data)
{
	// Fetch the size of this message type or error if unknown
	uint32_t bytes;
	switch(msg_id) {
	case PIOS_CAN_GIMBAL:
		bytes = sizeof(struct pios_can_gimbal_message);
		break;
	default:
		return -1;
	}

	// Look up the CAN BUS Standard ID for this message type
	uint32_t std_id = pios_can_message_stdid[msg_id];

	// Format and send the message
	CanTxMsg msg;
	msg.StdId = std_id & 0x7FF;
	msg.ExtId = 0;
	msg.IDE = CAN_ID_STD;
	msg.RTR = CAN_RTR_DATA;
	msg.DLC = (bytes > 8) ? 8 : bytes;
	memcpy(msg.Data, data, msg.DLC);
	CAN_Transmit(can_dev->cfg->regs, &msg);

	return msg.DLC;
}

/////////////////////////////////////////////////////////////////////////
#ifdef PIOS_INCLUDE_UAVCAN /* start of def PIOS_INCLUDE_UAVCAN */
/////////////////////////////////////////////////////////////////////////

/**
 * PIOS_CAN_TxUAVCANData transmits a data message with a specified ID
 * @param[in] msg_id The message ID 
 * @param[in] is_ext Flag shows whether the ID is extended ID
 * @param[in] data Pointer to data message
 * @returns The number of the mailbox that is used for transmission or
  *         CAN_TxStatus_NoMailBox if there is no empty mailbox.
 */
uint8_t PIOS_CAN_TxUAVCANData(uint32_t msg_id, uint8_t is_ext, uint8_t *data)
{
	// Format and send the message
	CanTxMsg msg;
	if(!is_ext)
	{
		msg.StdId = msg_id;
		msg.ExtId = 0;
		msg.IDE = CAN_ID_STD;
	}
	else
	{
		msg.StdId = 0;
		msg.ExtId = msg_id;
		msg.IDE = CAN_ID_EXT;
	}	
	msg.RTR = CAN_RTR_DATA;
	msg.DLC = 8;
	memcpy(msg.Data, data, msg.DLC);

	uint8_t transmit_mailbox = 0xff;
	transmit_mailbox = CAN_Transmit(can_dev->cfg->regs, &msg);

	return transmit_mailbox;
}

/**
 * PIOS_CAN_CancelTransmit cancel a transmit in a mailbox with a specified ID
 * @param[in] Mailbox  The maibox ID 
 * @returns None.
 */
void PIOS_CAN_CancelTransmit(uint8_t Mailbox)
{
	CAN_CancelTransmit(can_dev->cfg->regs, Mailbox);
}

/**
 * PIOS_CAN_GetLastErrorCode get last error code (LEC)
 * @param[in] None 
 * @returns LEC.
 */
uint8_t PIOS_CAN_GetLastErrorCode()
{
	return CAN_GetLastErrorCode(can_dev->cfg->regs);
}

/**
 * PIOS_CAN_ClearLEC clear last error code (LEC)
 * @param[in] None 
 * @returns None.
 */
void PIOS_CAN_ClearLEC()
{
	CAN_ClearFlag(can_dev->cfg->regs, CAN_FLAG_LEC);
}

/**
 * PIOS_CAN_AllMailboxesBusy check mailbox transmission status
 * @param[in] None 
 * @returns is_busy.
 */
uint8_t PIOS_CAN_AllMailboxesBusy()
{
	uint8_t is_busy;
	
	if(CAN_TransmitStatus(can_dev->cfg->regs, 0) == CAN_TxStatus_Pending &&  
		CAN_TransmitStatus(can_dev->cfg->regs, 1) == CAN_TxStatus_Pending && 
		CAN_TransmitStatus(can_dev->cfg->regs, 2) == CAN_TxStatus_Pending)
		is_busy = 1;
	else
		is_busy = 0;

	return is_busy;
}

static void PIOS_CAN_RxUAVCAN(void)
{
	CAN_ClearITPendingBit(can_dev->cfg->regs, CAN_IT_FMP1);
	
	bool valid = PIOS_CAN_validate(can_dev);
	PIOS_Assert(valid);
	
	CanRxMsg RxMessage;
	CAN_Receive(CAN1, CAN_FIFO1, &RxMessage);
	
	if (RxMessage.StdId == CAN_COM_ID) {
		// TODO: remove this need_yield/woken pattern when f1 is on chibios
		bool rx_need_yield;
		if (can_dev->rx_in_cb) {
			(void) (can_dev->rx_in_cb)(can_dev->rx_in_context, RxMessage.Data, RxMessage.DLC, NULL, &rx_need_yield);
		}
	} else {
		//process_received_message(RxMessage);
	}

	PIOSUAVCAN_RxISR_Callback();
}

static void PIOS_CAN_TxUAVCAN(void)
{
	uint8_t mailbox_id;
	uint8_t txok;

	if(CAN_GetFlagStatus(can_dev->cfg->regs, CAN_FLAG_RQCP0) == SET)
	{	
		mailbox_id = 0;
		txok = (CAN_TransmitStatus(can_dev->cfg->regs, 0) == CAN_TxStatus_Ok)? 0x01:0x00;
	}
	else if(CAN_GetFlagStatus(can_dev->cfg->regs, CAN_FLAG_RQCP1) == SET)
	{
		mailbox_id = 1;
		txok = (CAN_TransmitStatus(can_dev->cfg->regs, 1) == CAN_TxStatus_Ok)? 0x01:0x00;
	}
	else if(CAN_GetFlagStatus(can_dev->cfg->regs, CAN_FLAG_RQCP2) == SET)
	{
		mailbox_id = 2;
		txok = (CAN_TransmitStatus(can_dev->cfg->regs, 2) == CAN_TxStatus_Ok)? 0x01:0x00;
	}
	else
	{
		mailbox_id = 0xff;
		txok = CAN_TxStatus_Failed;
	}

	CAN_ClearITPendingBit(can_dev->cfg->regs, CAN_IT_TME);
	
	bool valid = PIOS_CAN_validate(can_dev);
	PIOS_Assert(valid);
			
	PIOSUAVCAN_TxISR_Callback(mailbox_id, txok);
}

/////////////////////////////////////////////////////////////////////////
#else	/* else of def PIOS_INCLUDE_UAVCAN */
/////////////////////////////////////////////////////////////////////////

static void PIOS_CAN_RxStart(uintptr_t can_id, uint16_t rx_bytes_avail)
{
	struct pios_can_dev *can_dev = (struct pios_can_dev *)can_id;
	
	bool valid = PIOS_CAN_validate(can_dev);
	PIOS_Assert(valid);
	
	CAN_ITConfig(can_dev->cfg->regs, CAN_IT_FMP1, ENABLE);
}

static void PIOS_CAN_TxStart(uintptr_t can_id, uint16_t tx_bytes_avail)
{
	struct pios_can_dev *can_dev = (struct pios_can_dev *)can_id;
	
	bool valid = PIOS_CAN_validate(can_dev);
	PIOS_Assert(valid);

 	CAN_ITConfig(can_dev->cfg->regs, CAN_IT_TME, ENABLE);
	
	PIOS_CAN_TxGeneric();
}

static void PIOS_CAN_RegisterRxCallback(uintptr_t can_id, pios_com_callback rx_in_cb, uintptr_t context)
{
	struct pios_can_dev *can_dev = (struct pios_can_dev *)can_id;

	bool valid = PIOS_CAN_validate(can_dev);
	PIOS_Assert(valid);
	
	/* 
	 * Order is important in these assignments since ISR uses _cb
	 * field to determine if it's ok to dereference _cb and _context
	 */
	can_dev->rx_in_context = context;
	can_dev->rx_in_cb = rx_in_cb;
}

static void PIOS_CAN_RegisterTxCallback(uintptr_t can_id, pios_com_callback tx_out_cb, uintptr_t context)
{
	struct pios_can_dev *can_dev = (struct pios_can_dev *)can_id;

	bool valid = PIOS_CAN_validate(can_dev);
	PIOS_Assert(valid);
	
	/* 
	 * Order is important in these assignments since ISR uses _cb
	 * field to determine if it's ok to dereference _cb and _context
	 */
	can_dev->tx_out_context = context;
	can_dev->tx_out_cb = tx_out_cb;
}


//! The mapping of message types to CAN BUS StdID
static struct pios_queue *pios_can_queues[PIOS_CAN_LAST];

/**
 * Create a queue to receive messages for a particular message
 * and return it
 * @param[in] id the CAN device ID
 * @param[in] msg_id The message ID (std ID < 0x7FF)
 */
struct pios_queue * PIOS_CAN_RegisterMessageQueue(uintptr_t id, enum pios_can_messages msg_id)
{
	// Fetch the size of this message type or error if unknown
	uint32_t bytes;
	switch(msg_id) {
	case PIOS_CAN_GIMBAL:
		bytes = sizeof(struct pios_can_gimbal_message);
		break;
	default:
		return NULL;
	}

	// Return existing queue if created
	if (pios_can_queues[msg_id] != NULL)
		return pios_can_queues[msg_id];

	// Create a queue that can manage the data message size
	struct pios_queue *queue;
	queue = PIOS_Queue_Create(2, bytes);
	if (queue == NULL)
		return NULL;

	// Store the queue handle for the driver
	pios_can_queues[msg_id] = queue;

	return queue;
}

/**
 * Process received CAN messages and push them out any corresponding
 * queues. Called from ISR.
 */
static bool process_received_message(CanRxMsg message)
{
	// Look for a known message that matches this CAN StdId
	uint32_t msg_id;
	for (msg_id = 0; msg_id < PIOS_CAN_LAST && pios_can_message_stdid[msg_id] != message.StdId; msg_id++);

	// If StdId is not one of the known messages, bail out
	if (msg_id == PIOS_CAN_LAST)
		return false;

	// Get the queue for this message and send the data
	struct pios_queue *queue = pios_can_queues[msg_id];
	if (queue == NULL)
		return false;

	bool woken = false;
	PIOS_Queue_Send_FromISR(queue, message.Data, &woken);

	return woken;
}

/**
 * @brief  This function handles CAN1 RX1 request.
 * @note   We are using RX1 instead of RX0 to avoid conflicts with the
 *         USB IRQ handler.
 */
static void PIOS_CAN_RxGeneric(void)
{
	CAN_ClearITPendingBit(can_dev->cfg->regs, CAN_IT_FMP1);

	bool valid = PIOS_CAN_validate(can_dev);
	PIOS_Assert(valid);

	CanRxMsg RxMessage;
	CAN_Receive(CAN1, CAN_FIFO1, &RxMessage);

	if (RxMessage.StdId == CAN_COM_ID) {
		// TODO: remove this need_yield/woken pattern when f1 is on chibios
		bool rx_need_yield;
		if (can_dev->rx_in_cb) {
			(void) (can_dev->rx_in_cb)(can_dev->rx_in_context, RxMessage.Data, RxMessage.DLC, NULL, &rx_need_yield);
		}
	} else {
		process_received_message(RxMessage);
	}
}

/**
 * @brief  This function handles CAN1 TX irq and sends more data if available
 */
static void PIOS_CAN_TxGeneric(void)
{
	CAN_ClearITPendingBit(can_dev->cfg->regs, CAN_IT_TME);

	bool valid = PIOS_CAN_validate(can_dev);
	PIOS_Assert(valid);

	bool tx_need_yield = false;
	
	if (can_dev->tx_out_cb) {

		// Prepare CAN message structure
		CanTxMsg msg;
		msg.StdId = CAN_COM_ID;
		msg.ExtId = 0;
		msg.IDE = CAN_ID_STD;
		msg.RTR = CAN_RTR_DATA;
		msg.DLC = (can_dev->tx_out_cb)(can_dev->tx_out_context, msg.Data, MAX_SEND_LEN, NULL, &tx_need_yield);

		// Send message and get mailbox number
		if (msg.DLC > 0) {
			CAN_Transmit(can_dev->cfg->regs, &msg);
		} else {
			CAN_ITConfig(can_dev->cfg->regs, CAN_IT_TME, DISABLE);
		}

		// TODO: deal with failure to send and keep the message to retransmit
	}
}
/////////////////////////////////////////////////////////////////////////
#endif /* end of def PIOS_INCLUDE_UAVCAN */
/////////////////////////////////////////////////////////////////////////

// Rx handlers
void CAN1_RX0_IRQHandler(void)
{
	PIOS_IRQ_Prologue();
#ifndef PIOS_INCLUDE_UAVCAN 
	PIOS_CAN_RxGeneric();
#else
	PIOS_CAN_RxUAVCAN();
#endif
	PIOS_IRQ_Epilogue();
}
void CAN1_RX1_IRQHandler(void)
{
	PIOS_IRQ_Prologue();
#ifndef PIOS_INCLUDE_UAVCAN 
	PIOS_CAN_RxGeneric();
#else
	PIOS_CAN_RxUAVCAN();
#endif
	PIOS_IRQ_Epilogue();
}
void CAN2_RX0_IRQHandler(void)
{
	PIOS_IRQ_Prologue();
#ifndef PIOS_INCLUDE_UAVCAN 
	PIOS_CAN_RxGeneric();
#else
	PIOS_CAN_RxUAVCAN();
#endif
	PIOS_IRQ_Epilogue();
}
void CAN2_RX1_IRQHandler(void)
{
	PIOS_IRQ_Prologue();
#ifndef PIOS_INCLUDE_UAVCAN 
	PIOS_CAN_RxGeneric();
#else
	PIOS_CAN_RxUAVCAN();
#endif
	PIOS_IRQ_Epilogue();
}

// Tx handlers
void CAN1_TX_IRQHandler(void)
{
	PIOS_IRQ_Prologue();
#ifndef PIOS_INCLUDE_UAVCAN 
	PIOS_CAN_TxGeneric();
#else
	PIOS_CAN_TxUAVCAN();
#endif
	PIOS_IRQ_Epilogue();
}
void CAN2_TX_IRQHandler(void)
{
	PIOS_IRQ_Prologue();
#ifndef PIOS_INCLUDE_UAVCAN 
	PIOS_CAN_TxGeneric();
#else
	PIOS_CAN_TxUAVCAN();
#endif
	PIOS_IRQ_Epilogue();
}

#endif /* PIOS_INCLUDE_CAN */
/**
 * @}
 * @}
 */

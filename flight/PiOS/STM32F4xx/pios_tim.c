/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_TIM Timer Functions
 * @brief PIOS Timer control code
 * @{
 *
 * @file       pios_tim.c  
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013-2014
 * @author     dRonin, http://dRonin.org/, Copyright (C) 2015
 * @brief      Sets up timers and ways to register callbacks on them
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

#include "pios_tim.h"
#include "pios_tim_priv.h"

#include "pixcar.h"
#include "jlink_rtt.h"

volatile static uint64_t time_mono = 0;

enum pios_tim_dev_magic {
	PIOS_TIM_DEV_MAGIC = 0x87654098,
};

struct pios_tim_dev {
	enum pios_tim_dev_magic     magic;

	const struct pios_tim_channel * channels;
	uint8_t num_channels;

	const struct pios_tim_callbacks * callbacks;
	uintptr_t context;
};

static struct pios_tim_dev pios_tim_devs[PIOS_TIM_MAX_DEVS];
static uint8_t pios_tim_num_devs;
static struct pios_tim_dev * PIOS_TIM_alloc(void)
{
	struct pios_tim_dev * tim_dev;

	if (pios_tim_num_devs >= PIOS_TIM_MAX_DEVS) {
		return (NULL);
	}

	tim_dev = &pios_tim_devs[pios_tim_num_devs++];
	tim_dev->magic = PIOS_TIM_DEV_MAGIC;

	return (tim_dev);
}

void PIOS_TIM_ITConfig(const struct pios_tim_clock_cfg * cfg, uint16_t TIM_IT, FunctionalState NewState)
{
	TIM_ITConfig(cfg->timer,TIM_IT,NewState);
}

int32_t PIOS_TIM_InitClock(const struct pios_tim_clock_cfg * cfg)
{
	PIOS_DEBUG_Assert(cfg);

	/* Configure the dividers for this timer */
	TIM_TimeBaseInit(cfg->timer, (TIM_TimeBaseInitTypeDef*)cfg->time_base_init);

	/* Configure internal timer clocks */
	TIM_InternalClockConfig(cfg->timer);

	/* Enable timers */
	TIM_Cmd(cfg->timer, ENABLE);

	/* Enable Interrupts */
	NVIC_Init((NVIC_InitTypeDef*)&cfg->irq.init);

	/* Check for optional second vector (dirty hack)
	 * This is needed for timers 1 and 8 when requiring more than one event
	 * to generate an interrupt. Actually up to 4 interrupts may be necessary.
	 */
	if (cfg->irq2.init.NVIC_IRQChannel != 0)
		NVIC_Init((NVIC_InitTypeDef*)&cfg->irq2.init);
	
	if (cfg->irq3.init.NVIC_IRQChannel != 0)
		NVIC_Init((NVIC_InitTypeDef*)&cfg->irq3.init);

	if (cfg->irq4.init.NVIC_IRQChannel != 0)
		NVIC_Init((NVIC_InitTypeDef*)&cfg->irq4.init);

	return 0;
}

void PIOS_TIM_InitHallSensorIF(const struct pios_tim_clock_cfg * tim_cfg, const struct pios_hall_cfg * hall_cfg)
{
	/* Config timer base */
	PIOS_TIM_InitClock(tim_cfg);
	/* Disable timer for more configurations */
	TIM_Cmd(tim_cfg->timer, DISABLE);

	/* Config GPIO */
	for (int i = 0; i < hall_cfg->num_channels; i++) {
		const struct pios_tim_channel * chan = &hall_cfg->channels[i];
		GPIO_Init(chan->pin.gpio, (GPIO_InitTypeDef*)&chan->pin.init);	
		PIOS_DEBUG_Assert(chan->remap);
		GPIO_PinAFConfig(chan->pin.gpio, chan->pin.pin_source, chan->remap);
	}

	/* Enable hall sensor interface */
	TIM_SelectHallSensor(tim_cfg->timer, ENABLE);	

	/* Select slave input trigger */
	TIM_SelectInputTrigger(tim_cfg->timer,TIM_TS_TI1F_ED);
	TIM_SelectSlaveMode(tim_cfg->timer, TIM_SlaveMode_Reset);

	/* Config input capture channel */
	const struct pios_tim_channel * ic_chan = &hall_cfg->channels[2];
	
	TIM_ICInitTypeDef TIM_ICInitStructure = hall_cfg->tim_ic_init;
	TIM_ICInitStructure.TIM_Channel = ic_chan->timer_chan;
	TIM_ICInit(ic_chan->timer, &TIM_ICInitStructure);

	/* Enable the Capture Compare Interrupt Request */
	TIM_ITConfig(ic_chan->timer, TIM_IT_Trigger, ENABLE);
	// TIM_ITConfig(ic_chan->timer, TIM_IT_CC1 | TIM_IT_Trigger | TIM_IT_Update, ENABLE);	

	TIM_Cmd(tim_cfg->timer, ENABLE);

	/* Setup local variable which stays in this scope */
	/* Doing this here and using a local variable saves doing it in the ISR */
	// TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	// TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	// TIM_ICInitStructure.TIM_ICFilter = 0x0;
}

void PIOS_TIM_InitTimerPin(uintptr_t tim_id, int idx)
{
	struct pios_tim_dev * tim_dev = (struct pios_tim_dev *) tim_id;

	PIOS_Assert(idx < tim_dev->num_channels);

	const struct pios_tim_channel * chan = &tim_dev->channels[idx];

	GPIO_Init(chan->pin.gpio, (GPIO_InitTypeDef*)&chan->pin.init);

	PIOS_DEBUG_Assert(chan->remap);

	GPIO_PinAFConfig(chan->pin.gpio, chan->pin.pin_source, chan->remap);
}

void PIOS_TIM_InitAllTimerPins(uintptr_t tim_id)
{
	struct pios_tim_dev * tim_dev = (struct pios_tim_dev *) tim_id;

	/* Configure the pins */
	for (uint8_t i = 0; i < tim_dev->num_channels; i++) {
		PIOS_TIM_InitTimerPin(tim_id, i);
	}
}

void PIOS_TIM_SetBankToGPOut(uintptr_t tim_id, TIM_TypeDef *timer)
{
	struct pios_tim_dev * tim_dev = (struct pios_tim_dev *) tim_id;

	GPIO_InitTypeDef gpio_inf;
	gpio_inf.GPIO_Speed = GPIO_Speed_25MHz;
	gpio_inf.GPIO_Mode = GPIO_Mode_OUT;
	gpio_inf.GPIO_OType = GPIO_OType_PP;
	gpio_inf.GPIO_PuPd = GPIO_PuPd_DOWN;

	for (uint8_t i = 0; i < tim_dev->num_channels; i++) {
		const struct pios_tim_channel * chan = &tim_dev->channels[i];

		if (chan->timer != timer) {
			continue;
		}

		// Only steal the pin info from the matching pin
		gpio_inf.GPIO_Pin = chan->pin.init.GPIO_Pin;
		GPIO_Init(chan->pin.gpio, &gpio_inf);
	}
}

int32_t PIOS_TIM_InitChannels(uintptr_t * tim_id, const struct pios_tim_channel * channels, uint8_t num_channels, const struct pios_tim_callbacks * callbacks, uintptr_t context)
{
	PIOS_Assert(channels);
	PIOS_Assert(num_channels);

	struct pios_tim_dev * tim_dev;
	tim_dev = (struct pios_tim_dev *) PIOS_TIM_alloc();
	if (!tim_dev) goto out_fail;

	/* Bind the configuration to the device instance */
	tim_dev->channels = channels;
	tim_dev->num_channels = num_channels;
	tim_dev->callbacks = callbacks;
	tim_dev->context = context;

	*tim_id = (uintptr_t)tim_dev;

	PIOS_TIM_InitAllTimerPins(*tim_id);

	return(0);

out_fail:
	return(-1);
}

static void PIOS_TIM_generic_irq_handler(TIM_TypeDef * timer)
{
	/* Iterate over all registered clients of the TIM layer to find channels on this timer */
	for (uint8_t i = 0; i < pios_tim_num_devs; i++) {
		const struct pios_tim_dev * tim_dev = &pios_tim_devs[i];

		if (!tim_dev->channels || tim_dev->num_channels == 0) {
			/* No channels to process on this client */
			continue;
		}

		/* Check for an overflow event on this timer */
		bool overflow_event;
		uint16_t overflow_count;
		if (TIM_GetITStatus(timer, TIM_IT_Update) == SET) {
			TIM_ClearITPendingBit(timer, TIM_IT_Update);
			overflow_count = timer->ARR;
			overflow_event = true;
		} else {
			overflow_count = 0;
			overflow_event = false;
		}
		
		for (uint8_t j = 0; j < tim_dev->num_channels; j++) {
			const struct pios_tim_channel * chan = &tim_dev->channels[j];

			if (chan->timer != timer) {
				/* channel is not on this timer */
				continue;
			}

			/* Figure out which interrupt bit we should be looking at */
			uint16_t timer_it;
			switch (chan->timer_chan) {
			case TIM_Channel_1:
				timer_it = TIM_IT_CC1;
				break;
			case TIM_Channel_2:
				timer_it = TIM_IT_CC2;
				break;
			case TIM_Channel_3:
				timer_it = TIM_IT_CC3;
				break;
			case TIM_Channel_4:
				timer_it = TIM_IT_CC4;
				break;
			default:
				PIOS_Assert(0);
				break;
			}

			bool edge_event;
			uint16_t edge_count;
			if (TIM_GetITStatus(chan->timer, timer_it) == SET) {
				TIM_ClearITPendingBit(chan->timer, timer_it);

				/* Read the current counter */
				switch(chan->timer_chan) {
				case TIM_Channel_1:
					edge_count = TIM_GetCapture1(chan->timer);
					break;
				case TIM_Channel_2:
					edge_count = TIM_GetCapture2(chan->timer);
					break;
				case TIM_Channel_3:
					edge_count = TIM_GetCapture3(chan->timer);
					break;
				case TIM_Channel_4:
					edge_count = TIM_GetCapture4(chan->timer);
					break;
				default:
					PIOS_Assert(0);
					break;
				}
				edge_event = true;
			} else {
				edge_event = false;
				edge_count = 0;
			}

			if (!tim_dev->callbacks) {
				/* No callbacks registered, we're done with this channel */
				continue;
			}

			/* Generate the appropriate callbacks */
			if (overflow_event & edge_event) {
				/*
				 * When both edge and overflow happen in the same interrupt, we
				 * need a heuristic to determine the order of the edge and overflow
				 * events so that the callbacks happen in the right order.  If we
				 * get the order wrong, our pulse width calculations could be off by up
				 * to ARR ticks.  That could be bad.
				 *
				 * Heuristic: If the edge_count is < 16 ticks above zero then we assume the
				 *            edge happened just after the overflow.
				 */

				if (edge_count < 16) {
					/* Call the overflow callback first */
					if (tim_dev->callbacks->overflow) {
						(*tim_dev->callbacks->overflow)((uintptr_t)tim_dev,
									tim_dev->context,
									j,
									overflow_count);
					}
					/* Call the edge callback second */
					if (tim_dev->callbacks->edge) {
						(*tim_dev->callbacks->edge)((uintptr_t)tim_dev,
									tim_dev->context,
									j,
									edge_count);
					}
				} else {
					/* Call the edge callback first */
					if (tim_dev->callbacks->edge) {
						(*tim_dev->callbacks->edge)((uintptr_t)tim_dev,
									tim_dev->context,
									j,
									edge_count);
					}
					/* Call the overflow callback second */
					if (tim_dev->callbacks->overflow) {
						(*tim_dev->callbacks->overflow)((uintptr_t)tim_dev,
									tim_dev->context,
									j,
									overflow_count);
					}
				}
			} else if (overflow_event && tim_dev->callbacks->overflow) {
				(*tim_dev->callbacks->overflow)((uintptr_t)tim_dev,
								tim_dev->context,
								j,
								overflow_count);
			} else if (edge_event && tim_dev->callbacks->edge) {
				(*tim_dev->callbacks->edge)((uintptr_t)tim_dev,
							tim_dev->context,
							j,
							edge_count);
			}
		}
	}
}

uint64_t PIOS_UAVCAN_TIM_GetMonoTime()
{
	uint64_t usec = 0;
	volatile uint64_t time = time_mono;
	
	uint32_t cnt = TIM5->CNT;
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) == SET)
	{
		cnt = TIM5->CNT;
		time += TIM5->ARR;
	}
	usec = time + cnt;

	// JLinkRTTPrintf(0, "USEC: %ld, ARR: %ld\n", usec, TIM2->ARR);

	return usec;
}

static void PIOS_UAVCAN_TIM_irq_handler(TIM_TypeDef * timer)
{
	/* Check for an overflow event on this timer */
	uint16_t overflow_count;
	if (TIM_GetITStatus(timer, TIM_IT_Update) == SET) {
		TIM_ClearITPendingBit(timer, TIM_IT_Update);
		overflow_count = timer->ARR;
	} else {
		overflow_count = 0;
	}

	time_mono += overflow_count;
	
	// JLinkRTTPrintf(0, "ARR: %ld\n", overflow_count);
}

// uint16_t PIOS_TIM_GetHallSensorReading()
// {
// 	return hall_sensor_reading;
// }
 
static void PIOS_HALLSENSOR_TIM_irq_handler(TIM_TypeDef * timer)
{
	volatile static uint16_t hall_sensor_reading = 0;

	/* Check for a trigger event on this timer */
	bool trg_event;
	if (TIM_GetITStatus(timer, TIM_IT_Trigger) == SET) {
		/* Read the current counter */
		TIM_ClearITPendingBit(timer, TIM_IT_Trigger);		
		trg_event = true;
		hall_sensor_reading = TIM_GetCapture1(timer);
		updateHallSensorData(hall_sensor_reading);
	} else {
		trg_event = false;
		hall_sensor_reading = 0;
	}

	/* Check for an overflow event on this timer */
	bool overflow_event;
	uint16_t overflow_count;
	if (TIM_GetITStatus(timer, TIM_IT_Update) == SET) {
		TIM_ClearITPendingBit(timer, TIM_IT_Update);
		overflow_count = timer->ARR;
		overflow_event = true;
		// set sensor reading as 0 if timer overflows
		hall_sensor_reading = 0;
	} else {
		overflow_count = 0;
		overflow_event = false;
	}
		
	/* Check for an edge count event on this timer */
	bool edge_event;
	uint16_t edge_count;
	if (TIM_GetITStatus(timer, TIM_IT_CC1) == SET) {
		TIM_ClearITPendingBit(timer, TIM_IT_CC1);
		/* Read the current counter */
		edge_count = TIM_GetCapture1(timer);
		edge_event = true;
	} else {
		edge_event = false;
		edge_count = 0;
	}

	(void)trg_event;
	(void)overflow_event;
	(void)edge_event;
	(void)overflow_count;
	(void)hall_sensor_reading;
	(void)edge_count;
	// if(overflow_event)
	// {
	// 	JLinkRTTPrintf(0, "Timer 1 overflow ISR triggered\n", overflow_count);
	// }

	// if(edge_event)
	// {	
	// 	JLinkRTTPrintf(0, "Timer 1 edge ISR triggered, raw: %ld\n", edge_count);
	// }

	// if(trg_event)
	// {		
	// 	JLinkRTTPrintf(0, "Timer 1 trigger ISR triggered, hall sensor reading: %ld\n", hall_sensor_reading);
	// }
}


/* Bind Interrupt Handlers
 *
 * Map all valid TIM IRQs to the common interrupt handler
 * and give it enough context to properly demux the various timers
 */
void TIM1_CC_IRQHandler(void) __attribute__ ((alias ("PIOS_TIM_1_CC_irq_handler")));
static void PIOS_TIM_1_CC_irq_handler (void)
{
	PIOS_IRQ_Prologue();
	// PIOS_TIM_generic_irq_handler (TIM1);
	PIOS_HALLSENSOR_TIM_irq_handler(TIM1);
	PIOS_IRQ_Epilogue();
}

// The rest of TIM1 interrupts are overlapped
void TIM1_BRK_TIM9_IRQHandler(void) __attribute__ ((alias ("PIOS_TIM_1_BRK_TIM_9_irq_handler")));
static void PIOS_TIM_1_BRK_TIM_9_irq_handler (void)
{
	PIOS_IRQ_Prologue();

	if (TIM_GetITStatus(TIM1, TIM_IT_Break)) {
		// PIOS_TIM_generic_irq_handler(TIM1);
		PIOS_HALLSENSOR_TIM_irq_handler(TIM1);
	} else if (TIM_GetITStatus(TIM9, TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 | TIM_IT_COM | TIM_IT_Trigger | TIM_IT_Break)) {
		PIOS_TIM_generic_irq_handler (TIM9);
	}

	PIOS_IRQ_Epilogue();
}

void TIM1_UP_TIM10_IRQHandler(void) __attribute__ ((alias ("PIOS_TIM_1_UP_TIM_10_irq_handler")));
static void PIOS_TIM_1_UP_TIM_10_irq_handler (void)
{
	PIOS_IRQ_Prologue();

	if (TIM_GetITStatus(TIM1, TIM_IT_Update)) {
		// PIOS_TIM_generic_irq_handler(TIM1);
		PIOS_HALLSENSOR_TIM_irq_handler(TIM1);
	} else if (TIM_GetITStatus(TIM10, TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 | TIM_IT_COM | TIM_IT_Trigger | TIM_IT_Break)) {
		PIOS_TIM_generic_irq_handler (TIM10);
	}

	PIOS_IRQ_Epilogue();
}
void TIM1_TRG_COM_TIM11_IRQHandler(void) __attribute__ ((alias ("PIOS_TIM_1_TRG_COM_TIM_11_irq_handler")));
static void PIOS_TIM_1_TRG_COM_TIM_11_irq_handler (void)
{
	PIOS_IRQ_Prologue();

	if (TIM_GetITStatus(TIM1, TIM_IT_Trigger | TIM_IT_COM)) {
		// PIOS_TIM_generic_irq_handler(TIM1);
		PIOS_HALLSENSOR_TIM_irq_handler(TIM1);
	} else if (TIM_GetITStatus(TIM11, TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 | TIM_IT_COM | TIM_IT_Trigger | TIM_IT_Break)) {
		PIOS_TIM_generic_irq_handler (TIM11);
	}

	PIOS_IRQ_Epilogue();
}

#ifndef PIOS_OMIT_TIM2IRQ
void TIM2_IRQHandler(void) __attribute__ ((alias ("PIOS_TIM_2_irq_handler")));
static void PIOS_TIM_2_irq_handler (void)
{
	PIOS_IRQ_Prologue();
	PIOS_TIM_generic_irq_handler (TIM2);	
	PIOS_IRQ_Epilogue();
}
#endif

void TIM3_IRQHandler(void) __attribute__ ((alias ("PIOS_TIM_3_irq_handler")));
static void PIOS_TIM_3_irq_handler (void)
{
	PIOS_IRQ_Prologue();
	PIOS_TIM_generic_irq_handler (TIM3);
	PIOS_IRQ_Epilogue();
}

#if !defined(PIOS_VIDEO_TIM4_COUNTER)
void TIM4_IRQHandler(void) __attribute__ ((alias ("PIOS_TIM_4_irq_handler")));
static void PIOS_TIM_4_irq_handler (void)
{
	PIOS_IRQ_Prologue();
	PIOS_TIM_generic_irq_handler (TIM4);
	PIOS_IRQ_Epilogue();
}
#endif /* !defined(PIOS_VIDEO_TIM4_COUNTER) */

void TIM5_IRQHandler(void) __attribute__ ((alias ("PIOS_TIM_5_irq_handler")));
static void PIOS_TIM_5_irq_handler (void)
{
	PIOS_IRQ_Prologue();
	// PIOS_TIM_generic_irq_handler (TIM5);
	PIOS_UAVCAN_TIM_irq_handler(TIM5);
	PIOS_IRQ_Epilogue();
}

void TIM6_DAC_IRQHandler(void) __attribute__ ((alias ("PIOS_TIM_6_DAC_irq_handler")));
static void PIOS_TIM_6_DAC_irq_handler (void)
{
	PIOS_IRQ_Prologue();
	// TODO: Check for DAC
	PIOS_TIM_generic_irq_handler (TIM6);
	PIOS_IRQ_Epilogue();
}

void TIM7_IRQHandler(void) __attribute__ ((alias ("PIOS_TIM_7_irq_handler")));
static void PIOS_TIM_7_irq_handler (void)
{
	PIOS_IRQ_Prologue();
	PIOS_TIM_generic_irq_handler (TIM7);
	PIOS_IRQ_Epilogue();
}

void TIM8_CC_IRQHandler(void) __attribute__ ((alias ("PIOS_TIM_8_CC_irq_handler")));
static void PIOS_TIM_8_CC_irq_handler (void)
{
	PIOS_IRQ_Prologue();
	PIOS_TIM_generic_irq_handler (TIM8);
	PIOS_IRQ_Epilogue();
}

// The rest of TIM8 interrupts are overlapped
void TIM8_BRK_TIM12_IRQHandler(void) __attribute__ ((alias ("PIOS_TIM_8_BRK_TIM_12_irq_handler")));
static void PIOS_TIM_8_BRK_TIM_12_irq_handler (void)
{
	PIOS_IRQ_Prologue();

	if (TIM_GetITStatus(TIM8, TIM_IT_Break)) {
		PIOS_TIM_generic_irq_handler(TIM8);
	} else if (TIM_GetITStatus(TIM12, TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 | TIM_IT_COM | TIM_IT_Trigger | TIM_IT_Break)) {
		PIOS_TIM_generic_irq_handler (TIM12);
	}

	PIOS_IRQ_Epilogue();
}

void TIM8_UP_TIM13_IRQHandler(void) __attribute__ ((alias ("PIOS_TIM_8_UP_TIM_13_irq_handler")));
static void PIOS_TIM_8_UP_TIM_13_irq_handler (void)
{
	PIOS_IRQ_Prologue();

	if (TIM_GetITStatus(TIM8, TIM_IT_Update)) {
		PIOS_TIM_generic_irq_handler(TIM8);
	} else if (TIM_GetITStatus(TIM13, TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 | TIM_IT_COM | TIM_IT_Trigger | TIM_IT_Break)) {
		PIOS_TIM_generic_irq_handler (TIM13);
	}

	PIOS_IRQ_Epilogue();
}

void TIM8_TRG_COM_TIM14_IRQHandler(void) __attribute__ ((alias ("PIOS_TIM_8_TRG_COM_TIM_14_irq_handler")));
static void PIOS_TIM_8_TRG_COM_TIM_14_irq_handler (void)
{
	PIOS_IRQ_Prologue();

	if (TIM_GetITStatus(TIM8, TIM_IT_Trigger | TIM_IT_COM)) {
		PIOS_TIM_generic_irq_handler(TIM8);
	} else if (TIM_GetITStatus(TIM14, TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 | TIM_IT_COM | TIM_IT_Trigger | TIM_IT_Break)) {
		PIOS_TIM_generic_irq_handler (TIM14);
	}

	PIOS_IRQ_Epilogue();
}

/**
 * @}
 * @}
 */

/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup   PIOS_PWM PWM Input Functions
 * @brief		Code to measure with PWM input
 * @{
 *
 * @file       pios_pwm.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
 * @brief      PWM Input functions (STM32 dependent)
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

/* Project Includes */
#include "pios.h"

#if defined(PIOS_INCLUDE_RTC)
#include <pios_rtc_priv.h>

#include "jlink_rtt.h"

#ifndef PIOS_RTC_PRESCALER
#define PIOS_RTC_PRESCALER 100
#endif

struct rtc_callback_entry {
  void (*fn)(uintptr_t);
  uintptr_t data;
};

#define PIOS_RTC_MAX_CALLBACKS 3
struct rtc_callback_entry rtc_callback_list[PIOS_RTC_MAX_CALLBACKS];
static uint8_t rtc_callback_next = 0;

void PIOS_RTC_Init(const struct pios_rtc_cfg * cfg)
{
	RCC_BackupResetCmd(ENABLE);
	RCC_BackupResetCmd(DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	// Divide external clock to 1 MHz
	RCC_RTCCLKConfig(cfg->clksrc);
	RCC_RTCCLKCmd(ENABLE);

	RTC_WakeUpCmd(DISABLE);
	// Divide 1 Mhz clock by 8 -> 125 khz
	RTC_WakeUpClockConfig(RTC_WakeUpClock_RTCCLK_Div8);
	// Divide 125 khz by 125 to get 1kHz
	RTC_SetWakeUpCounter(cfg->prescaler); //cfg->prescaler);
	RTC_WakeUpCmd(ENABLE);
	
	/* Configure and enable the RTC Second interrupt */
	EXTI_InitTypeDef ExtiInit = {
		.EXTI_Line = EXTI_Line22, // matches above GPIO pin
		.EXTI_Mode = EXTI_Mode_Interrupt,
		.EXTI_Trigger = EXTI_Trigger_Rising,
		.EXTI_LineCmd = ENABLE,
	};
	EXTI_Init(&ExtiInit);
	NVIC_Init((NVIC_InitTypeDef*)&cfg->irq.init);
	RTC_ITConfig(RTC_IT_WUT, ENABLE);
	
	RTC_ClearFlag(RTC_FLAG_WUTF);
}

#define CVT_ST2MS(n) (((((n) - 1ULL) * 1000ULL) / ((uint64_t)CH_FREQUENCY)) + 1UL)

uint32_t PIOS_RTC_GetSystemTime()
{
	// RTC_TimeTypeDef current_time;

	// RTC_GetTime(RTC_Format_BIN, &current_time);

	// JLinkRTTPrintf(0, "RTC System Time (H-M-S): %d, %d, %d \n", current_time.RTC_Hours, current_time.RTC_Minutes, current_time.RTC_Seconds);

	//return CVT_ST2MS(chTimeNow());
	return chTimeNow();
	
	//return (((uint32_t)current_time.RTC_Hours)*60*60 + ((uint32_t)current_time.RTC_Minutes)*60 + ((uint32_t)current_time.RTC_Seconds))*1000 + CVT_ST2MS(chTimeNow());
}

uint32_t PIOS_RTC_Counter()
{
	return RTC_GetWakeUpCounter();
}

/* FIXME: This shouldn't use hard-coded clock rates, dividers or prescalers.
 *        Should get these from the cfg struct passed to init.
 */
float PIOS_RTC_Rate()
{
	return (float) (8e6 / 128) / (1 + PIOS_RTC_PRESCALER);
}

float PIOS_RTC_MsPerTick() 
{
	return 1000.0f / PIOS_RTC_Rate();
}

/* TODO: This needs a mutex around rtc_callbacks[] */
bool PIOS_RTC_RegisterTickCallback(void (*fn)(uintptr_t id), uintptr_t data)
{
	struct rtc_callback_entry * cb;
	if (rtc_callback_next >= PIOS_RTC_MAX_CALLBACKS) {
		return false;
	}

	cb = &rtc_callback_list[rtc_callback_next++];

	cb->fn   = fn;
	cb->data = data;
	return true;
}

void PIOS_RTC_irq_handler (void)
{
	PIOS_IRQ_Prologue();

	if (RTC_GetITStatus(RTC_IT_WUT))
	{
		/* Call all registered callbacks */
		for (uint8_t i = 0; i < rtc_callback_next; i++) {
			struct rtc_callback_entry * cb = &rtc_callback_list[i];
			if (cb->fn) {
				(cb->fn)(cb->data);
			}
		}

		/* Clear the RTC Second interrupt */
		RTC_ClearITPendingBit(RTC_IT_WUT);
	}

	if (EXTI_GetITStatus(EXTI_Line22) != RESET)
		EXTI_ClearITPendingBit(EXTI_Line22);

	PIOS_IRQ_Epilogue();
}
#endif

/** 
 * @}
 * @}
 */

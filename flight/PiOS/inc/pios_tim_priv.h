#ifndef PIOS_TIM_PRIV_H
#define PIOS_TIM_PRIV_H

#include <pios_stm32.h>

struct pios_tim_clock_cfg {
	TIM_TypeDef * timer;
	const TIM_TimeBaseInitTypeDef * time_base_init;
	struct stm32_irq irq;
	struct stm32_irq irq2;
	struct stm32_irq irq3;
	struct stm32_irq irq4;
};

struct pios_tim_channel {
	TIM_TypeDef * timer;
	uint8_t timer_chan;

	struct stm32_gpio pin;
	uint32_t remap;
};

struct pios_hall_cfg {
	TIM_ICInitTypeDef tim_ic_init;
	const struct pios_tim_channel * channels;
	uint8_t num_channels;
};

struct pios_tim_callbacks {
	void (*overflow)(uintptr_t tim_id, uintptr_t context, uint8_t chan_idx, uint16_t count);
	void (*edge)(uintptr_t tim_id, uintptr_t context, uint8_t chan_idx, uint16_t count);
};

void PIOS_TIM_ITConfig(const struct pios_tim_clock_cfg * cfg, uint16_t TIM_IT, FunctionalState NewState);
int32_t PIOS_TIM_InitClock(const struct pios_tim_clock_cfg * cfg);
int32_t PIOS_TIM_InitChannels(uintptr_t * tim_id, const struct pios_tim_channel * channels, uint8_t num_channels, const struct pios_tim_callbacks * callbacks, uintptr_t context);

void PIOS_TIM_InitTimerPin(uintptr_t tim_id, int idx);
void PIOS_TIM_InitAllTimerPins(uintptr_t tim_id);
void PIOS_TIM_SetBankToGPOut(uintptr_t tim_id, TIM_TypeDef *timer);

/* Only supported on target pixracer */
void PIOS_TIM_InitHallSensorIF(const struct pios_tim_clock_cfg * tim_cfg, const struct pios_hall_cfg * hall_cfg);
uint16_t PIOS_TIM_GetHallSensorReading();

#endif	/* PIOS_TIM_PRIV_H */

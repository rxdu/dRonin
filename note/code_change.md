Code changed for Pixracer board 

1. To support STM32 type STM32F427_437xx

* /home/rdu/Workspace/dronin/dRonin/make/firmware-arches.mk

```
else ifneq "$(findstring STM32F42,$(CHIP))" ""
OPENOCD_JTAG_CONFIG ?= stlink-v2.cfg
OPENOCD_CONFIG      := stm32f4x.cfg
MCU                 := cortex-m4
# Note: Current code base doesn't provide an easy way to add a new 
#	type without modifying code at multiple places.
STM32_TYPE          := STM32F427_437xx 
ARCH_TYPES          := STM32F4xx STM32
```

* /home/rdu/Workspace/dronin/dRonin/flight/PiOS/STM32F4xx/library_chibios.mk

```
#
# Linker script depending on STM32 type
#
ifneq "$(findstring STM32F40_41xxx,$(STM32_TYPE))" ""
LINKER_SCRIPTS_APP	 =	$(PIOS_DEVLIB)/sections_chibios.ld
else ifneq "$(findstring STM32F427_437xx,$(STM32_TYPE))" ""
LINKER_SCRIPTS_APP	 =	$(PIOS_DEVLIB)/sections_chibios.ld
else ifneq "$(findstring STM32F446xx,$(STM32_TYPE))" ""
LINKER_SCRIPTS_APP	 =	$(PIOS_DEVLIB)/sections_chibios_STM32F446xx.ld
else
$(error No linker script found for $(STM32_TYPE))
endif
```

* /home/rdu/Workspace/dronin/dRonin/flight/PiOS/inc/pios_stm32.h

```
#if defined(STM32F40_41xxx) || defined(STM32F427_437xx) ||defined(STM32F446xx) /*  F4 */
#define PIOS_PERIPHERAL_APB1_COUNTER_CLOCK (PIOS_PERIPHERAL_APB1_CLOCK * 2)
#define PIOS_PERIPHERAL_APB2_COUNTER_CLOCK (PIOS_PERIPHERAL_APB2_CLOCK * 2)
#endif
```

* /home/rdu/Workspace/dronin/dRonin/flight/PiOS/STM32/pios_spi.c

```
// line 180
#if defined(STM32F40_41xxx) || defined(STM32F427_437xx) || defined(STM32F446xx)
```

* /home/rdu/Workspace/dronin/dRonin/flight/PiOS/STM32/pios_wdg.c

```
// line 40
#if defined(STM32F40_41xxx) || defined(STM32F427_437xx) || defined(STM32F446xx)  /* F4 */
```

* /home/rdu/Workspace/dronin/dRonin/flight/PiOS/STM32/pios_servo.c

```
// line 779
#if defined(STM32F40_41xxx) || defined(STM32F427_437xx) || defined(STM32F446xx)
#define CLEAR_BITS(gpio, bits) do { (gpio)->BSRRH = bits; } while (0)
#define SET_BITS(gpio, bits) do { (gpio)->BSRRL = bits; } while (0)
#else

// line 912
#elif defined(STM32F40_41xxx) || defined(STM32F427_437xx) || defined(STM32F446xx) /*  F4 */
```

2. PiOS CAN Bus support 

* added "pios_can_common.c" to /home/rdu/Workspace/dronin/dRonin/flight/PiOS/Common
* replaced "pios_can.h" and "pios_can_priv.h" at /home/rdu/Workspace/dronin/dRonin/flight/PiOS/inc


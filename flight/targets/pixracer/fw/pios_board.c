/**
 ******************************************************************************
 * @addtogroup Targets Target Boards
 * @{
 * @addtogroup Pixracer
 * @{
 *
 * @file       pixracer/fw/pios_board.c
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2014
 * @author     dRonin, http://dronin.org Copyright (C) 2015
 * @brief      The board specific initialization routines
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

/* Pull in the board-specific static HW definitions.
 * Including .c files is a bit ugly but this allows all of
 * the HW definitions to be const and static to limit their
 * scope.
 *
 * NOTE: THIS IS THE ONLY PLACE THAT SHOULD EVER INCLUDE THIS FILE
 */

#include "board_hw_defs.c"

#include <pios.h>
#include <pios_hal.h>
#include <openpilot.h>
#include <uavobjectsinit.h>
#include <taskmonitor.h>

#include "jlink_rtt.h"

#define PIOS_COM_CAN_RX_BUF_LEN 256
#define PIOS_COM_CAN_TX_BUF_LEN 256

uintptr_t pios_com_can_id;
uintptr_t pios_can_id;

uintptr_t pios_com_openlog_logging_id;
uintptr_t pios_uavo_settings_fs_id;
uintptr_t pios_internal_adc_id;

pios_i2c_t external_i2c_adapter_id;

/**
 * PIOS_Board_Init()
 * initializes all the core subsystems on this specific hardware
 * called from System/openpilot.c
 */

#include <pios_board_info.h>

void PIOS_Board_Init(void)
{
	/* ----------------------------------------------------------------- */
	/*								System 								 */
	/* ----------------------------------------------------------------- */
	/* Delay system */
	PIOS_DELAY_Init();

	/* LED */
	const struct pios_board_info *bdinfo = &pios_board_info_blob;

	const struct pios_annunc_cfg *led_cfg = PIOS_BOARD_HW_DEFS_GetLedCfg(bdinfo->board_rev);
	PIOS_Assert(led_cfg);
	PIOS_ANNUNC_Init(led_cfg);

	/* Initialize Sensor SPI interface */
	if (PIOS_SPI_Init(&pios_spi_baro_id, &pios_spi_baro_cfg))
	{
		PIOS_DEBUG_Assert(0);
	}
	if (PIOS_SPI_Init(&pios_spi_gyro_accel_mag_id, &pios_spi_gyro_accel_mag_cfg))
	{
		PIOS_Assert(0);
	}

	/* Initialize the task monitor library */
	TaskMonitorInitialize();

	/* Initialize UAVObject libraries */
	UAVObjInitialize();

	/* Initialize the alarms library. Reads RCC reset flags */
	// AlarmsInitialize();
	PIOS_RESET_Clear(); // Clear the RCC reset flags after use.

	/* Initialize the real-time clock and its associated tick */
	PIOS_RTC_Init(&pios_rtc_main_cfg);

	/* Initialize watchdog as early as possible to catch faults during init
	 * but do it only if there is no debugger connected
	 */
	if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) == 0)
	{
		PIOS_WDG_Init();
	}

	/* Set up timers */
	//Timers used for PPM inputs 
	PIOS_TIM_InitClock(&tim_8_cfg);
	// Timers used for PWM outputs
	PIOS_TIM_InitClock(&tim_4_cfg);
#ifdef PIOS_INCLUDE_UAVCAN 
	// Timer for UAVCAN clock
	PIOS_TIM_ITConfig(&tim_5_cfg, TIM_IT_Update, ENABLE);
	PIOS_TIM_InitClock(&tim_5_cfg);
#endif
	// Timer for speed measurement
	PIOS_TIM_InitHallSensorIF(&tim_1_cfg, &pios_hall_input_cfg);


	/* ----------------------------------------------------------------- */
	/*							Control Input							 */
	/* ----------------------------------------------------------------- */

	/* Configure the inport */
	PIOS_HAL_ConfigurePort(HWSHARED_PORTTYPES_PPM, // port type protocol
						   NULL,				   // usart_port_cfg
						   NULL,				   // com_driver
						   NULL,				   // i2c_id
						   NULL,				   // i2c_cfg
						   &pios_ppm_cfg,		   // ppm_cfg
						   NULL,				   // pwm_cfg
						   PIOS_LED_ALARM,		   // led_id
						   NULL,				   // dsm_cfg
						   0,					   // dsm_mode
						   NULL);				   // sbus_cfg

#if defined(PIOS_INCLUDE_CAN)
	if (PIOS_CAN_Init(&pios_can_id, &pios_can_cfg) != 0)
		PIOS_HAL_CriticalError(PIOS_LED_ALARM, PIOS_HAL_PANIC_CAN);
	else
		JLinkWriteString(0, "CAN init successful\n");

#ifndef PIOS_INCLUDE_UAVCAN 
	if (PIOS_COM_Init(&pios_com_can_id, &pios_can_com_driver, pios_can_id,
					  PIOS_COM_CAN_RX_BUF_LEN,
					  PIOS_COM_CAN_TX_BUF_LEN))
		PIOS_HAL_CriticalError(PIOS_LED_ALARM, PIOS_HAL_PANIC_CAN);
	else
		JLinkWriteString(0, "CAN comm init successful\n");
#endif
	// Otherwise, delegate COM functions to UAVCAN library 

	/* pios_com_bridge_id = pios_com_can_id; */
#endif

	/* ----------------------------------------------------------------- */
	/*							Control Output							 */
	/* ----------------------------------------------------------------- */
	PIOS_Servo_Init(&pios_servo_cfg);

	/* ----------------------------------------------------------------- */
	/*							  Sensors								 */
	/* ----------------------------------------------------------------- */
	/* init sensor queue registration */
	PIOS_SENSORS_Init();

	PIOS_WDG_Clear();
	PIOS_DELAY_WaitmS(200);
	PIOS_WDG_Clear();

#if defined(PIOS_INCLUDE_MPU)
	pios_mpu_dev_t mpu_dev = NULL;
	if (PIOS_MPU_SPI_Init(&mpu_dev, pios_spi_gyro_accel_mag_id, 0, &pios_mpu_cfg) != 0)
		PIOS_HAL_CriticalError(PIOS_LED_ALARM, PIOS_HAL_PANIC_IMU);

	// set the gyro range:
	// PIOS_MPU_SCALE_250_DEG, PIOS_MPU_SCALE_500_DEG
	// PIOS_MPU_SCALE_1000_DEG, PIOS_MPU_SCALE_2000_DEG
	PIOS_MPU_SetGyroRange(PIOS_MPU_SCALE_1000_DEG);

	// set the acc range:
	// PIOS_MPU_SCALE_2G, PIOS_MPU_SCALE_4G
	// PIOS_MPU_SCALE_8G, PIOS_MPU_SCALE_8G
	PIOS_MPU_SetAccelRange(PIOS_MPU_SCALE_2G);

	// the filter has to be set before rate else divisor calculation will fail
	// 188 , 98 , 42 , 20 , 10 , 5
	uint16_t bandwidth = 188;
	PIOS_MPU_SetGyroBandwidth(bandwidth);
#endif

#if defined(PIOS_INCLUDE_HMC5983)
	if( PIOS_HMC5983_Init(pios_spi_gyro_accel_mag_id, 1, &pios_hmc5983_internal_cfg) != 0)
		PIOS_HAL_CriticalError(PIOS_LED_ALARM, PIOS_HAL_PANIC_MAG);
	else
		JLinkWriteString(0, "HMC5983 init successful\n");
#endif
}

/**
 * @}
 * @}
 */

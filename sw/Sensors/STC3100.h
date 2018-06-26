 /***
 *      ____    ____  _  ____   ____    ___   __  __ 
 *     / ___|  / ___|(_)|  _ \ |  _ \  / _ \ |  \/  |
 *     \___ \ | |    | || | | || |_) || | | || |\/| |
 *      ___) || |___ | || |_| ||  _ < | |_| || |  | |
 *     |____/  \____||_||____/ |_| \_\ \___/ |_|  |_|
 *        (C)2018 Scidrom 
 
	Description: Driver for STC3100, Battery monitor IC with Coulomb counter/gas gauge
	License: GNU General Public License
	Maintainer: S54MTB
*/

/******************************************************************************
  * @file    STC3100.h
  * @author  S54MTB
  * @version V1.0.0
  * @date    15-January-2018
  * @brief   STC3100 header file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 Scidrom 
	* This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * (at your option) any later version.
	*
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program.  If not, see <https://www.gnu.org/licenses/>.  *
  ******************************************************************************
  */


#ifndef __STC3100_h__
#define __STC3100_h__
#include "stm32l0xx_hal.h"
#include "config.h" 

/**I2C Device Address*/
#define STC3100_ADDRESS 0x70

/** STC3100 register addresses */
#define STC3100_REG_MODE		0x00 
#define STC3100_REG_CTRL		0x01 
#define STC3100_REG_RSOCL		0x02 /* Relative State-of-Charge */
#define STC3100_REG_RSOCH		0x03
#define STC3100_REG_AIL			0x06
#define STC3100_REG_AIH			0x07
#define STC3100_REG_VOLTL		0x08
#define STC3100_REG_VOLTH		0x09
#define STC3100_REG_TEMPL		0x0A
#define STC3100_REG_TEMPH		0x0B
#define STC3100_REG_ID0		  0x18
#define STC3100_REG_RAM0		0x20


#define POWER_STATUS_CHARGING 			1
#define POWER_STATUS_NOT_CHARGING 	0

/** 
  * \brief  Battery device
	* 
  */
typedef struct
{
	I2C_HandleTypeDef *hi2c;
	float voltage;	// battery voltage in V
	float current;  // battery current in A
	float rsoc;			// battery charge in Ah
	float temperature;  // chip temperature in deg. C
#ifdef BATTERY_CHARGER
	uint8_t charging;
#endif
	uint8_t ids[8];
} stc3100_device_info_t;


/** 
  * \brief  Battery properties
	* 
  */
typedef enum {
	BATTERY_PROP_PRESENT,
	BATTERY_PROP_VOLTAGE_NOW,
	BATTERY_PROP_CURRENT_NOW,
	BATTERY_PROP_CAPACITY,
	BATTERY_PROP_TEMP,	
	BATTERY_PROP_UNIQUEID	,
#ifdef 	BATTERY_CHARGER
	BATTERY_PROP_CHARGING	,
#endif
} stc3100_battery_prop_t;



/** 
  * \brief  STC3100 Mode register bits
	* 
  */
typedef union
{
	struct
	{
		uint8_t SEL_EXT_CLK 	: 1;	/// 32,768 Hz clock source: 0: auto-detect, 1: external clock
		uint8_t GG_RES 				: 2;	/// Gas gauge ADC resolution: 00:14 bits, 01:13 bits, 10:12 bits
		uint8_t GG_CAL				: 1;	/// ADC Calibration: 0: no effect, 1: used to calibrate the AD converters
		uint8_t GG_RUN				: 1; 	/// 0: standby mode, 1: operating mode
		/** Unused */
		uint8_t 							: 3;
	} b;
	uint8_t w;
} STC3100_MODE_t;




/** 
  * \brief  STC3100 Control / status register bits
  */
typedef union
{
	struct
	{
		/**Unused*/
		uint8_t IO0DATA 			: 1;	/// Port IO0 data status / Drive
		uint8_t GG_RST 				: 1;	/// Gas Gauge Reset (self-clearing bit)
		uint8_t GG_EOC 				: 1;	/// end of battery current conversion
		uint8_t VTM_EOC 			: 1;	/// end of a battery voltage or temperature conversion cycle
		uint8_t PORDET 				: 1;	/// Power on reset
		/** Unused */
		uint8_t 							: 3;
		} b;
	uint8_t w;
} STC3100_CTRL_t;


HAL_StatusTypeDef Battery_Get(stc3100_device_info_t *bat, stc3100_battery_prop_t prop);
HAL_StatusTypeDef Battery_Init(stc3100_device_info_t *bat);
void stc3100_battery_chargerinit(GPIO_TypeDef *port, uint32_t Pin);


#endif


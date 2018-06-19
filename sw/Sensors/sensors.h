 /***
 *      ____    ____  _  ____   ____    ___   __  __ 
 *     / ___|  / ___|(_)|  _ \ |  _ \  / _ \ |  \/  |
 *     \___ \ | |    | || | | || |_) || | | || |\/| |
 *      ___) || |___ | || |_| ||  _ < | |_| || |  | |
 *     |____/  \____||_||____/ |_| \_\ \___/ |_|  |_|
 *        (C)2018 Scidrom 
 
	Description: External sensors driver and setup
	License: GNU General Public License
	Maintainer: S54MTB
*/

/******************************************************************************
  * @file    sensors.h
  * @author  S54MTB
  * @version V1.0.0
  * @date    15-June-2018
  * @brief   Sensors header file
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



#ifndef __SENSORS_h__
#define __SENSORS_h__

#include "stm32l0xx_hal.h"
#include "hpm.h"
#include "si7013.h"
#include "tmp75.h"
#include "sht31.h"
#include "bme280.h"

extern I2C_HandleTypeDef SEN_hi2c1;


/**
  Sensors readouts structure
	*/
typedef struct
{
	uint16_t pm2_5;		// PM 2.5 
	uint16_t pm10;		// PM 10
	int16_t tmp75_T;	// Temperature from TMP75 in 1/100 deg. C
	int16_t si7013_T; // Temperature from SI7013 in 1/100 deg. C
	int16_t sht31_T;	// Temperature from SHT31 in 1/100 deg. C
	int16_t bme280_T;	// Temperature from BME280 in 1/100 deg. C
	uint8_t si7013_RH;// Relative humidity from SI7013 in %RH
	uint8_t sht31_RH;	// Relative humidity from SHT31 in %RH
	uint8_t bme280_RH;// Relative humidity from BME280 in %RH
	uint16_t bme280_p;// Pressure from BME280, units 0,1 hPa
} sen_readout_t;

HAL_StatusTypeDef SensorsInit(void);

void Sensor_readouts(sen_readout_t *readouts);


#define HPM_SENSOR 1
#define BME280_SENSOR 1

#define SI7013_ADDR  0x40

#endif




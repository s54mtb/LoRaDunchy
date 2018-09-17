 /***
 *      ____    ____  _  ____   ____    ___   __  __ 
 *     / ___|  / ___|(_)|  _ \ |  _ \  / _ \ |  \/  |
 *     \___ \ | |    | || | | || |_) || | | || |\/| |
 *      ___) || |___ | || |_| ||  _ < | |_| || |  | |
 *     |____/  \____||_||____/ |_| \_\ \___/ |_|  |_|
 *        (C)2018 Scidrom 
 
	Description: Loradunchy sensors application level driver --- header file
	License: GNU General Public License
	Maintainer: S54MTB
*/

/******************************************************************************
  * @file    loradunchy_sensors.h
  * @author  S54MTB
  * @version V1.0.0
  * @date    15-Augiust-2018
  * @brief   Loradunchy sensors application level driver
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


#ifndef _LORADUNCHY_SENSORS_H_
#define _LORADUNCHY_SENSORS_H_

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l0xx.h"                  // Device header

/* Sensors readout structure 
     Just uncomment .... 
 */

/**
  Sensors readouts structure
	*/
typedef struct
{
	uint16_t pm2_5;		// PM 2.5 
	uint16_t pm10;		// PM 10
	
	int16_t tmp75_T;	// Temperature from TMP75 in 1/100 deg. C

	int16_t si7013_T; // Temperature from SI7013 in 1/100 deg. C
	uint8_t si7013_RH;// Relative humidity from SI7013 in %RH
	
//	int16_t sht31_T;	// Temperature from SHT31 in 1/100 deg. C
//	uint8_t sht31_RH;	// Relative humidity from SHT31 in %RH
	
//	int16_t bme280_T;	// Temperature from BME280 in 1/100 deg. C
//	uint8_t bme280_RH;// Relative humidity from BME280 in %RH
//	uint16_t bme280_p;// Pressure from BME280, units 0,1 hPa

	uint16_t battery; // Current battery RSOC in mAh
	 int16_t current; 	// Battery current in mA (+ charging, - discharging)
	uint8_t voltage;  // Battery voltage value = 100 * (Ubat[V] - 2,5V)
} sen_readout_t;


/* Si7013:  
      si7013_get_device_id
      si7013_measure_intemperature
      si7013_measure_humidity  
      si7013_measure_thermistor
*/  
#include "si7013.h"
#include "tmp75.h"
#include "hpm.h"
#include "stc3100.h"

/* I2C */  
extern uint8_t I2C_Loradunchy_Init( void );
extern I2C_HandleTypeDef I2C_Loradunchy_Handle;
extern uint8_t I2C_Loradunchy_WriteData( uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size );
extern uint8_t I2C_Loradunchy_ReadData( uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size );


/* Loradunchy sensors */
uint8_t   Loradunchy_sensor_Init( void );
void      Loradunchy_Read_sensors(sen_readout_t *readouts);
void      Loradunchy_ReadHPMsensor(void);
void Loradunchy_sensor_SetPM(uint16_t pm2_5, uint16_t pm10);





#ifdef __cplusplus
}
#endif /* End of CPP guard */
#endif /* _LORADUNCHY_SENSORS_H_ */




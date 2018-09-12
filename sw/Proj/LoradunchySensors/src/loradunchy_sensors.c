 /***
 *      ____    ____  _  ____   ____    ___   __  __ 
 *     / ___|  / ___|(_)|  _ \ |  _ \  / _ \ |  \/  |
 *     \___ \ | |    | || | | || |_) || | | || |\/| |
 *      ___) || |___ | || |_| ||  _ < | |_| || |  | |
 *     |____/  \____||_||____/ |_| \_\ \___/ |_|  |_|
 *        (C)2018 Scidrom 
 
	Description: Loradunchy sensors application level driver 
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


#include "../inc/loradunchy_sensors.h"


static   uint16_t lastpm10=0, lastpm2_5=0;


void Loradunchy_sensor_SetPM(uint16_t pm2_5, uint16_t pm10)
{
	lastpm10 = pm10;
	lastpm2_5 = pm2_5;
}

/* Init Sensors */
uint8_t     Loradunchy_sensor_Init( void )
{
  HPM_Init();
	HPM_Reset();
	
  return I2C_Loradunchy_Init();
}




void Loradunchy_Read_sensors(sen_readout_t *readouts)
{
  int32_t temperature, humidity;
  int32_t Tmp75_t;
  
  /* Measure temperature from Si7013 */
  si7013_measure_intemperature(&I2C_Loradunchy_Handle, 0x40, &temperature); // m°C
  /* Convert to 1/100 °C */
  readouts->si7013_T = temperature / 10;
  
  si7013_measure_humidity(&I2C_Loradunchy_Handle, 0x40, &humidity); // m% RH
  /* Convert to % RH */
  readouts->si7013_RH = humidity/1000;
  
  Tmp75_Read_Int_Teperature(&I2C_Loradunchy_Handle, Tmp75_SlaveAddress(Tmp75Addr_Zero, Tmp75Addr_Zero, Tmp75Addr_One), &Tmp75_t);
  readouts->tmp75_T = Tmp75_t;
  
	readouts->pm2_5 = lastpm2_5;
	readouts->pm10 = lastpm10;

}





















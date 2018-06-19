 /***
 *      ____    ____  _  ____   ____    ___   __  __ 
 *     / ___|  / ___|(_)|  _ \ |  _ \  / _ \ |  \/  |
 *     \___ \ | |    | || | | || |_) || | | || |\/| |
 *      ___) || |___ | || |_| ||  _ < | |_| || |  | |
 *     |____/  \____||_||____/ |_| \_\ \___/ |_|  |_|
 *        (C)2018 Scidrom 
 
	Description: SHT31 driver
	License: GNU General Public License
	Maintainer: S54MTB
*/

/******************************************************************************
  * @file    SHT31.c
  * @author  S54MTB
  * @version V1.0.0
  * @date    14-January-2018
  * @brief   SHT31 driver - simple one! No clock stretching, no 
	*          periodic measurements
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
  * along with this program.  If not, see <https://www.gnu.org/licenses/>.  
  ******************************************************************************
  */


#include "sht31.h"

static uint8_t SHT31_CalcCrc(uint8_t *data, uint8_t nbrOfBytes);



/*
 * sht31_meas_oneshot() - Read measurement in single shot mode, No CLK Stretching
 * @hi2c:  handle to I2C interface
 * @adr: I2C address
 * @rep: Repeatability (High, Medium, Low)
 * temperature: temperature in deg. C
 * humidity: relative humidity in %
 * Returns HAL status 
 */
HAL_StatusTypeDef sht31_meas_oneshot(I2C_HandleTypeDef *hi2c, uint8_t adr,
								sht31_repeatability_t rep,
							  float *temperature, float *humidity)
{
	
	uint8_t cmd[2];
	uint8_t result[6];
	HAL_StatusTypeDef  error;
	
//	cmd[1] = 0x24; 	// No Clock Stretching
//	switch (rep)
//	{
//		case SHT31_rep_High: cmd[0] = 0x30; break;
//		case SHT31_rep_Medium: cmd[0] = 0x26; break;
//		case SHT31_rep_Low: cmd[0] = 0x2d; break;
//	}
	cmd[0] = 0x2c;
	cmd[1] = 0x06;
	error = HAL_I2C_Master_Transmit(hi2c,adr<<1,cmd,2,100);
	if (error != HAL_OK) return error;
	//HAL_Delay(2000);
	error = HAL_I2C_Master_Receive(hi2c,adr<<1 | 0x01, result, 6, 100);
	if (error != HAL_OK) return error;
	
	if (SHT31_CalcCrc(&result[0],2) != result[2]) return HAL_ERROR; // T CRC not OK
	if (SHT31_CalcCrc(&result[3],2) != result[5]) return HAL_ERROR; // RH CRC not OK
	
	*humidity = 100.0f * (result[3] * 256.0f + result[4]) / 65535.0f;
	*temperature = -45.0f +175 * ((result[0] * 256.0f + result[1]) / 65535.0f);
	
	return HAL_OK;  /* Success */
	
}


//-----------------------------------------------------------------------------
#define POLYNOMIAL  0x131 // P(x) = x^8 + x^5 + x^4 + 1 = 100110001

static uint8_t SHT31_CalcCrc(uint8_t *data, uint8_t nbrOfBytes)
{
  uint8_t bit;        // bit mask
  uint8_t crc = 0xFF; // calculated checksum
  uint8_t byteCtr;    // byte counter
  
  // calculates 8-Bit checksum with given polynomial
  for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++)
  {
    crc ^= (data[byteCtr]);
    for(bit = 8; bit > 0; --bit)
    {
      if(crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
      else           crc = (crc << 1);
    }
  }
  
  return crc;
}



/*
 * sht31_softreset() - Soft Reset / Re-Initialization
 * @hi2c:  handle to I2C interface
 * @adr: I2C address
 * Returns HAL status 
 */
HAL_StatusTypeDef sht31_softreset(I2C_HandleTypeDef *hi2c, uint8_t adr)
{
	uint8_t cmd[2] = {0x30, 0xa2};	// Soft reset command
	return HAL_I2C_Master_Transmit(hi2c,adr<<1,cmd,2,100);	
}


/*
 * sht31_read_status() - Read Status Register
 * @hi2c:  handle to I2C interface
 * @adr: I2C address
 * status: status register
 * Returns HAL status 
 */
HAL_StatusTypeDef sht31_read_status(I2C_HandleTypeDef *hi2c, uint8_t adr,
								sht_status_reg_t *status)
{
	HAL_StatusTypeDef  error;
	uint8_t cmd[2] = {0xf3, 0x2d};	// Soft reset command
	uint8_t result[2];
	
	error = HAL_I2C_Master_Transmit(hi2c,adr<<1,cmd,2,100);		
	if (error != HAL_OK) return error;
	error = HAL_I2C_Master_Receive(hi2c,adr<<1 | 0x01, result, 6, 100);
	if (error != HAL_OK) return error;
	
	status->u16 = result[1]<<8 | result[0];
	
	return HAL_OK;  /* Success */
	
}


/*
 * sht31_meas_oneshot() - Read measurement in single shot mode, No CLK Stretching
 * @hi2c:  handle to I2C interface
 * @adr: I2C address
 * @rep: Repeatability (High, Medium, Low)
 * temperature: int temperature in mdeg. C
 * humidity: int relative humidity in m%RH
 * Returns HAL status 
 */
HAL_StatusTypeDef sht31_meas_oneshot_int(I2C_HandleTypeDef *hi2c, uint8_t adr,
								sht31_repeatability_t rep,
							  int32_t *temperature, int32_t *humidity)
{
	HAL_StatusTypeDef status;
	float ftemp, fhumi;
	
	status = sht31_meas_oneshot(hi2c, adr, rep, &ftemp, &fhumi);
	
	*temperature = (int32_t)(ftemp * 1000.0);
	*humidity = (int32_t)(fhumi*1000.0);
	return status;

}







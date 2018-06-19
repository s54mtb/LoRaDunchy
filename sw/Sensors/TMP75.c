 /***
 *      ____    ____  _  ____   ____    ___   __  __ 
 *     / ___|  / ___|(_)|  _ \ |  _ \  / _ \ |  \/  |
 *     \___ \ | |    | || | | || |_) || | | || |\/| |
 *      ___) || |___ | || |_| ||  _ < | |_| || |  | |
 *     |____/  \____||_||____/ |_| \_\ \___/ |_|  |_|
 *        (C)2018 Scidrom 
 
	Description: TMP75 driver
	License: GNU General Public License
	Maintainer: S54MTB
*/

/******************************************************************************
  * @file    TMP75.c
  * @author  S54MTB
  * @version V1.0.0
  * @date    14-January-2018
  * @brief   TMP75 driver
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


/**
  * The TMP75 and TMP175 devices are digital temperature sensors ideal for NTC 
  * and PTC thermistor replacement. The devices offer a typical accuracy of ±1°C
  *	without requiring calibration or external component signal conditioning. IC 
	* temperature sensors are highly linear and do not require complex 
	* calculations or look-up tables to derive the temperature. The on-chip 12-bit
	* ADC offers resolutions down to 0.0625°C. The devices are available in the 
	* industry standard LM75 SOIC-8 and MSOP-8 footprint.
	*/

#include "tmp75.h"


static HAL_StatusTypeDef Tmp75_Write_Register(I2C_HandleTypeDef *hi2c, uint8_t adr, 
							uint8_t register_pointer, uint16_t register_value);



static HAL_StatusTypeDef Tmp75_Read_Register(I2C_HandleTypeDef *hi2c, uint8_t adr, 
							uint8_t register_pointer, uint8_t* receive_buffer);
							


/*!
 * \brief Convert address line state to slave address
 *
 * \retval slave address or 0xff if address line state combination is invalid
 */
uint8_t Tmp75_SlaveAddress(Tmp75_AddressLine_t a2, Tmp75_AddressLine_t a1, 
Tmp75_AddressLine_t a0)
{
	
	typedef struct
	{
		Tmp75_AddressLine_t a2;
		Tmp75_AddressLine_t a1;
		Tmp75_AddressLine_t a0;
		uint8_t SlaveAddress;
	} Lm75_addrconv_t;
	
	const Lm75_addrconv_t Lm75_addrconv[] =
	{
		{Tmp75Addr_Zero, Tmp75Addr_Zero, Tmp75Addr_Zero, 0x48},
		{Tmp75Addr_Zero, Tmp75Addr_Zero, Tmp75Addr_One, 0x49},
		{Tmp75Addr_Zero, Tmp75Addr_One, Tmp75Addr_Zero, 0x4A},
		{Tmp75Addr_Zero, Tmp75Addr_One, Tmp75Addr_One, 0x4B},
		{Tmp75Addr_One, Tmp75Addr_Zero, Tmp75Addr_Zero, 0x4C},
		{Tmp75Addr_One, Tmp75Addr_Zero, Tmp75Addr_One, 0x4D},
		{Tmp75Addr_One, Tmp75Addr_One, Tmp75Addr_Zero, 0x4E},
		{Tmp75Addr_One, Tmp75Addr_One, Tmp75Addr_One, 0x4F},
		{Tmp75Addr_Float, Tmp75Addr_Zero, Tmp75Addr_Zero, 0x70},
		{Tmp75Addr_Float, Tmp75Addr_Zero, Tmp75Addr_Float, 0x71},
		{Tmp75Addr_Float, Tmp75Addr_Zero, Tmp75Addr_One, 0x72},
		{Tmp75Addr_Float, Tmp75Addr_One, Tmp75Addr_Zero, 0x73},
		{Tmp75Addr_Float, Tmp75Addr_One, Tmp75Addr_Float, 0x74},
		{Tmp75Addr_Float, Tmp75Addr_One, Tmp75Addr_One, 0x75},
		{Tmp75Addr_Float, Tmp75Addr_Float, Tmp75Addr_Zero, 0x76},
		{Tmp75Addr_Float, Tmp75Addr_Float, Tmp75Addr_One, 0x77},
		{Tmp75Addr_Zero, Tmp75Addr_Float, Tmp75Addr_Zero, 0x28},
		{Tmp75Addr_Zero, Tmp75Addr_Float, Tmp75Addr_One, 0x29},
		{Tmp75Addr_One, Tmp75Addr_Float, Tmp75Addr_Zero, 0x2A},
		{Tmp75Addr_One, Tmp75Addr_Float, Tmp75Addr_One, 0x2B},
		{Tmp75Addr_Zero, Tmp75Addr_Zero, Tmp75Addr_Float, 0x2C},
		{Tmp75Addr_Zero, Tmp75Addr_One, Tmp75Addr_Float, 0x2D},
		{Tmp75Addr_One, Tmp75Addr_Zero, Tmp75Addr_Float, 0x2E},
		{Tmp75Addr_One, Tmp75Addr_One, Tmp75Addr_Float, 0x2F},
		{Tmp75Addr_Zero, Tmp75Addr_Float, Tmp75Addr_Float, 0x35},
		{Tmp75Addr_One, Tmp75Addr_Float, Tmp75Addr_Float, 0x36},
		{Tmp75Addr_Float, Tmp75Addr_Float, Tmp75Addr_Float, 0x37},
		{Tmp75Addr_Zero, Tmp75Addr_Zero, Tmp75Addr_Zero, 0x48},
		{Tmp75Addr_Zero, Tmp75Addr_Zero, Tmp75Addr_One, 0x49},
		{Tmp75Addr_Zero, Tmp75Addr_One, Tmp75Addr_Zero, 0x4A},
		{Tmp75Addr_Zero, Tmp75Addr_One, Tmp75Addr_One, 0x4B},
		{Tmp75Addr_One, Tmp75Addr_Zero, Tmp75Addr_Zero, 0x4C},
		{Tmp75Addr_One, Tmp75Addr_Zero, Tmp75Addr_One, 0x4D},
		{Tmp75Addr_One, Tmp75Addr_One, Tmp75Addr_Zero, 0x4E},
		{Tmp75Addr_One, Tmp75Addr_One, Tmp75Addr_One, 0x4F}
	};
	
	int i;
	
	for (i = 0; i<(sizeof(Lm75_addrconv) / sizeof(Lm75_addrconv[0])); i++)
	{
		if ( (Lm75_addrconv[i].a0 == a0) &
			    (Lm75_addrconv[i].a1 == a1) &
		       (Lm75_addrconv[i].a2 == a2) ) return Lm75_addrconv[i].SlaveAddress;
	}
	
	return 0xff;
	
	
}

/**
  * @brief  Init TMP75  
  * @param  hi2c: pointer to I2C handle
	* @param  adr: Slave address
	* @param  cfg: configuration register
  * @retval None
  */
HAL_StatusTypeDef Tmp75_Init(I2C_HandleTypeDef *hi2c, uint8_t adr, 
Tmp75_regconf_t cfg) {
															 
  uint16_t regval = cfg.r << 8;
	return Tmp75_Write_Register(hi2c, adr, TMP75_RCONF, regval);

}


static HAL_StatusTypeDef Tmp75_Write_Register(I2C_HandleTypeDef *hi2c, uint8_t adr, 
uint8_t register_pointer, uint16_t register_value) {

	uint8_t data[3];

  data[0] = register_pointer;     // Register address
	data[1] = register_value >> 8;  // MSB byte of 16bit data
	data[2] = register_value;       // LSB byte of 16bit data

	return HAL_I2C_Master_Transmit(hi2c, adr<<1, data, 3, 100);

}

static HAL_StatusTypeDef Tmp75_Read_Register(I2C_HandleTypeDef *hi2c, uint8_t adr, 
uint8_t register_pointer, uint8_t* receive_buffer) {

	HAL_StatusTypeDef status;
	
  // first set the register pointer to the register wanted to be read
	status = HAL_I2C_Master_Transmit(hi2c, adr<<1, &register_pointer, 1, 100);
  
	// receive the 2 x 8bit data into the receive buffer
	status |= HAL_I2C_Master_Receive(hi2c, adr<<1 | 0x01, receive_buffer, 2, 100);
	return status;
	
}
							
HAL_StatusTypeDef Tmp75_Read_Teperature(I2C_HandleTypeDef *hi2c, uint8_t adr, 
float* temperature) {
	HAL_StatusTypeDef status;
	uint8_t T_buffer[2];
	uint16_t temp;
	
	status = Tmp75_Read_Register(hi2c, adr, TMP75_RTEMP, T_buffer);
	if (status == HAL_OK)
	{
		temp = ((T_buffer[0]<<8) | T_buffer[1]) >> 4;
		*temperature = (float)temp * 0.0625f; 
	}
	
	return status;	
}



HAL_StatusTypeDef Tmp75_Read_Int_Teperature(I2C_HandleTypeDef *hi2c, uint8_t tmp75addr, 
int32_t* temperature) 
{
  float T;
	Tmp75_regconf_t TMP75_cfg;
	HAL_StatusTypeDef status;
	TMP75_cfg.b.R = TMP75_RESOLUTION_12BITS;
	TMP75_cfg.b.OS = 1;
	TMP75_cfg.b.F = 0;
	TMP75_cfg.b.POL = 0;
	TMP75_cfg.b.TM = 0;
	TMP75_cfg.b.SD = 0;
	
  status = Tmp75_Init(hi2c, tmp75addr, TMP75_cfg);
	if (status == HAL_OK)  status = Tmp75_Read_Teperature(hi2c, tmp75addr, &T);
  
	*temperature = (int32_t)(T*1000);
	
	return status;
}
	


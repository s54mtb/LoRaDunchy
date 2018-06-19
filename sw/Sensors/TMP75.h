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
  * @file    TMP75.h
  * @author  S54MTB
  * @version V1.0.0
  * @date    15-January-2018
  * @brief   TMP75 header file
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

#ifndef __TMP75_H__
#define __TMP75_H__

#include "stm32l0xx_hal.h"


/** Register addresses */
#define TMP75_RTEMP		0x00 			/// Temperature register 
#define TMP75_RCONF		0x01			/// Configuration register
#define TMP75_RTEMPL	0x02			/// TLOW register
#define TMP75_RTEMPH	0x03			/// THIGH register


#define TMP75_RESOLUTION_9BITS	0
#define TMP75_RESOLUTION_10BITS	1
#define TMP75_RESOLUTION_11BITS	2
#define TMP75_RESOLUTION_12BITS	3


/** \brief  Structure of Configuration register in TMP75
 */
typedef union
{
	struct
	{
			uint8_t SD		:1;  		/*!< bit:  0 Shutdown Mode */
			uint8_t TM		:1;  		/*!< bit:  1 Thermostat Mode */
			uint8_t POL		:1;  		/*!< bit:  2 Polarity */
			uint8_t F   	:2;  		/*!< bit:  3...4 Fault Queue */
			uint8_t R			:2;  		/*!< bit:  5...6 Converter Resolution */
			uint8_t OS		:1;  		/*!< bit:  7 One-Shot */
	} b;
	uint8_t r;
} Tmp75_regconf_t;

	
typedef enum 
{
	Tmp75Addr_Zero, 
	Tmp75Addr_One, 
	Tmp75Addr_Float
} Tmp75_AddressLine_t;


/*!
 * \brief Convert address line state to slave address
 *
 * \retval slave address or 0xff if address line state combination is invalid
 */
uint8_t Tmp75_SlaveAddress(Tmp75_AddressLine_t a2, Tmp75_AddressLine_t a1, 
Tmp75_AddressLine_t a0);



HAL_StatusTypeDef Tmp75_Init(I2C_HandleTypeDef *hi2c, uint8_t adr, 
                             Tmp75_regconf_t cfg);
							
							
HAL_StatusTypeDef Tmp75_Read_TempCelsius(I2C_HandleTypeDef *hi2c, uint8_t adr, 
							float* receive_buffer);
							
							
							
HAL_StatusTypeDef Tmp75_Read_TempEeprom(I2C_HandleTypeDef *hi2c, uint8_t adr, 
							uint16_t* receive_buffer);
							
							
							
HAL_StatusTypeDef Tmp75_One_ShotTemp(I2C_HandleTypeDef *hi2c, uint8_t adr);



HAL_StatusTypeDef Tmp75_Read_Teperature(I2C_HandleTypeDef *hi2c, uint8_t adr, 
float* temperature);


HAL_StatusTypeDef Tmp75_Read_Int_Teperature(I2C_HandleTypeDef *hi2c, uint8_t tmp75addr, 
int32_t* temperature) ;



#endif


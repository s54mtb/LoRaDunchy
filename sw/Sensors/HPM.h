 /***
 *      ____    ____  _  ____   ____    ___   __  __ 
 *     / ___|  / ___|(_)|  _ \ |  _ \  / _ \ |  \/  |
 *     \___ \ | |    | || | | || |_) || | | || |\/| |
 *      ___) || |___ | || |_| ||  _ < | |_| || |  | |
 *     |____/  \____||_||____/ |_| \_\ \___/ |_|  |_|
 *        (C)2018 Scidrom 
 
	Description: HPM driver
	License: GNU General Public License
	Maintainer: S54MTB
*/

/******************************************************************************
  * @file    HPM.h
  * @author  S54MTB
  * @version V1.0.0
  * @date    15-January-2018
  * @brief   HPM header file
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







#ifndef __HPM_h__
#define __HPM_h__

//#include "main.h"
#include "stm32l0xx_hal.h"

/* Commands */
#define HPM_READ							{0x68, 0x01, 0x04, 0x93}
#define HPM_START							{0x68, 0x01, 0x01, 0x96}
#define HPM_STOP							{0x68, 0x01, 0x02, 0x95}
#define HPM_START_AUTO				{0x68, 0x01, 0x40, 0x57}
#define HPM_STOP_AUTO					{0x68, 0x01, 0x20, 0x77}
#define HPM_AUTO_SEND_SIG			{0x42, 0x4d, 0x00, 0x1c}  // 42 4d length (00 1c) 
#define HPM_GET_COEFF					{0x68, 0x01, 0x10, 0x87}

/**
 * HPM Readout send state 
 */
typedef enum
{
	HPM_NORMAL,
	HPM_AUTOSEND
} HPM_Mode_t;


/**
 * HPM Sensor commands 
 */
typedef enum
{
	HPM_READ_C,
	HPM_START_C,
	HPM_STOP_C,
	HPM_START_AUTO_C,
	HPM_STOP_AUTO_C
} HPM_CMD_t;


/**
 * HPM Readout value type 
 */
typedef enum
{
	HPM_READOUT_PM10,
	HPM_READOUT_PM2_5
} HPM_Readout_Mode_t;


/**
 * HPM CMD Ack 
 */
typedef enum
{
	HPM_ACK_NONE,
	HPM_NACK,
	HPM_ACK
} HPM_Ack_t;




/**
 * HPM Sensor UART handle
 */
extern UART_HandleTypeDef HPM_UartHandle;

HAL_StatusTypeDef HPM_Init(void);
HAL_StatusTypeDef HPM_get_auto(void);
HPM_Ack_t HPM_GetAck(void);
uint16_t HPM_LastReadout(HPM_Readout_Mode_t HPM_ReadoutType);
HAL_StatusTypeDef HPM_get(void);
HAL_StatusTypeDef HPM_Start(void);
HAL_StatusTypeDef HPM_Stop(void);
HAL_StatusTypeDef HPM_StartAuto(void);
HAL_StatusTypeDef HPM_StopAuto(void);
HAL_StatusTypeDef HPM_SetAdjCoeff(uint8_t Coeff);
HAL_StatusTypeDef HPM_GetAdjCoeff(uint8_t *Coeff);
void HPM_Power(uint8_t ppp);
void HPM_Reset(void);


#endif




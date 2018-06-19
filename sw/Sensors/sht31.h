 /***
 *      ____    ____  _  ____   ____    ___   __  __ 
 *     / ___|  / ___|(_)|  _ \ |  _ \  / _ \ |  \/  |
 *     \___ \ | |    | || | | || |_) || | | || |\/| |
 *      ___) || |___ | || |_| ||  _ < | |_| || |  | |
 *     |____/  \____||_||____/ |_| \_\ \___/ |_|  |_|
 *        (C)2018 Scidrom 
 
	Description: SHT31 driver
	Supported temperature and humiditz readout in non-repetitive mode
	License: GNU General Public License
	Maintainer: S54MTB
*/

/******************************************************************************
  * @file    SHT31.h
  * @author  S54MTB
  * @version V1.0.0
  * @date    14-January-2018
  * @brief   SHT31 driver header file
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




#ifndef __SHT3X_H__
#define __SHT3X_H__

#include "stm32l0xx_hal.h"

#define SHT31_ADDR 0x44


typedef enum
{
	SHT31_rep_High,
	SHT31_rep_Medium, 
	SHT31_rep_Low
}  sht31_repeatability_t;


// Status-Register
typedef union {
  uint16_t u16;
  struct{
    uint16_t CrcStatus     : 1; // write data checksum status
    uint16_t CmdStatus     : 1; // command status
    uint16_t Reserve0      : 2; // reserved
    uint16_t ResetDetected : 1; // system reset detected
    uint16_t Reserve1      : 5; // reserved
    uint16_t T_Alert       : 1; // temperature tracking alert
    uint16_t RH_Alert      : 1; // humidity tracking alert
    uint16_t Reserve2      : 1; // reserved
    uint16_t HeaterStatus  : 1; // heater status
    uint16_t Reserve3      : 1; // reserved
    uint16_t AlertPending  : 1; // alert pending status 
  }bit;
} sht_status_reg_t;


HAL_StatusTypeDef sht31_meas_oneshot(I2C_HandleTypeDef *hi2c, uint8_t adr,
								sht31_repeatability_t rep,
							  float *temperature, float *humidity);

HAL_StatusTypeDef sht31_softreset(I2C_HandleTypeDef *hi2c, uint8_t adr);

HAL_StatusTypeDef sht31_read_status(I2C_HandleTypeDef *hi2c, uint8_t adr,
								sht_status_reg_t *status);

HAL_StatusTypeDef sht31_meas_oneshot_int(I2C_HandleTypeDef *hi2c, uint8_t adr,
								sht31_repeatability_t rep,
							  int32_t *temperature, int32_t *humidity);


#endif

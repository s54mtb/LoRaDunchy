 /***
 *      ____    ____  _  ____   ____    ___   __  __ 
 *     / ___|  / ___|(_)|  _ \ |  _ \  / _ \ |  \/  |
 *     \___ \ | |    | || | | || |_) || | | || |\/| |
 *      ___) || |___ | || |_| ||  _ < | |_| || |  | |
 *     |____/  \____||_||____/ |_| \_\ \___/ |_|  |_|
 *        (C)2018 Scidrom 
 
	Description: LoraDunchy global configuration file
	License: GNU General Public License
	Maintainer: S54MTB
*/

/******************************************************************************
  * @file    config.h
  * @author  S54MTB
  * @version V1.0.0
  * @date    12-June-2018
  * @brief   LoraDunchy global configuration file
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LORADUNCHY_CONFIG_H__
#define __LORADUNCHY_CONFIG_H__


/** Enable measurement of battery voltage on port PA.4 */
#define BAT_LEVEL_MEASUREMENT 1

/** Enable honeywell HPM particle sensor */ 
#define HPM_SENSOR 1

/** Enable BME280 temperature, pressure and humidity sensor */
#define BME280_SENSOR 1

/** Enable TMP75 temperature sensor */
#define TMP75_SENSOR 1

/** Enable SI7013 RH/T Sensor */
#define SI7013_SENSOR 1

#endif







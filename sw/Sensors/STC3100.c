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
  * @file    stc3100.c
  * @author  S54MTB
  * @version V1.0.0
  * @date    22-June-2018
  * @brief   Driver for STC3100, Battery monitor IC with Coulomb counter/gas gauge
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


#include "stc3100.h"
#include "stm32l0xx.h"                  // Device header
#include "config.h"



/**
  * @brief  Write STC3100 register(s) in blocking mode to a specific register address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  reg STC3100 Internal register address
  * @param  buf Pointer to data buffer
  * @param  len Amount of data to be sent
  * @retval HAL status
  */
static HAL_StatusTypeDef stc3100_write_regs(I2C_HandleTypeDef *hi2c, 
						uint8_t reg, uint8_t *buf, uint8_t len)
{
	HAL_StatusTypeDef ret; 
	int i;
	uint8_t data[65];

	if (len>64) return HAL_ERROR;
  data[0] = reg;     // Register address
	for (i=0; i<len; i++) data[i+1]=buf[i]; // add data

	/** Target device address: The device 7 bits address must be shift at right before call interface	*/	
	ret = HAL_I2C_Master_Transmit(hi2c, STC3100_ADDRESS << 1, data, len+1, 100);

	return ret;
}


/**
  * @brief  Read STC3100 register(s) in blocking mode from a specific register address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  reg STC3100 Internal register address
  * @param  buf Pointer to data buffer
  * @param  len Amount of data to be read
  * @retval HAL status
  */
static HAL_StatusTypeDef stc3100_read_regs(I2C_HandleTypeDef *hi2c, 
						uint8_t reg, uint8_t *buf, uint8_t len)
{
	HAL_StatusTypeDef ret; 
		
	/** Target device address: The device 7 bits address must be shift at right before call interface	*/
	// first set the register pointer to the register wanted to be read
	ret = HAL_I2C_Master_Transmit(hi2c, STC3100_ADDRESS << 1, &reg, 1, 100);
	
	// receive the len x 8bit data into the receive buffer
	ret |= HAL_I2C_Master_Receive(hi2c, STC3100_ADDRESS << 1 | 0x01, buf, len, 100);
	return ret;
}


/**
 * Init the battery fuel gauge. 
 */
HAL_StatusTypeDef Battery_Init(stc3100_device_info_t *bat)
{
	HAL_StatusTypeDef ret;
	uint8_t regs[2];

	// read the REG_CTRL to reset the GG_EOC and VTM_EOC bits
	stc3100_read_regs(bat->hi2c, STC3100_REG_CTRL, &regs[0], 1);
	
	regs[0] = 0x02; 		// write CTRL
	stc3100_write_regs(bat->hi2c, STC3100_REG_CTRL, &regs[0], 1);
	regs[0] = 0x10;			// Write mode
	stc3100_write_regs(bat->hi2c, STC3100_REG_MODE, &regs[0], 1);

	return ret;	
}


void stc3100_battery_chargerinit(GPIO_TypeDef *port, uint32_t Pin)
{
	
	GPIO_InitTypeDef GPIO_InitStruct;

	switch (GPIO_GET_INDEX(port))
	{
		case 0 : __HAL_RCC_GPIOA_CLK_ENABLE(); break;
		case 1 : __HAL_RCC_GPIOB_CLK_ENABLE(); break;
		case 2 : __HAL_RCC_GPIOC_CLK_ENABLE(); break;
		case 5 : __HAL_RCC_GPIOH_CLK_ENABLE(); break;
	}
  
	GPIO_InitStruct.Pin = Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(port, &GPIO_InitStruct);
	
}


/*
 * Read the battery Voltage in milivolts, return 0 if something wrong
 */
static HAL_StatusTypeDef stc3100_battery_voltage(stc3100_device_info_t *bat)
{
	HAL_StatusTypeDef ret;
	uint8_t regs[2];
	
	float volt = 0;

	ret = stc3100_read_regs(bat->hi2c,STC3100_REG_VOLTL,regs,2);
	
	// The battery voltage is coded in binary format, and the LSB value is 2.44 mV.
	if (ret == HAL_OK) {
		volt = ((regs[1] & 0x0f)<<8) + regs[0];
		volt *= 2.44e-3f;
	}
	
	bat->voltage = volt;

	return ret;
}


/*
 * read the STC3100 unique ID, first byte must be 0x10
 */
static HAL_StatusTypeDef stc3100_read_ID(stc3100_device_info_t *bat)
{
	HAL_StatusTypeDef ret; 
  ret = stc3100_read_regs(bat->hi2c,STC3100_REG_ID0,bat->ids,8);
  return ret;	
}


/*
 * Return the battery average current
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static HAL_StatusTypeDef stc3100_battery_current(stc3100_device_info_t *bat)
{
	HAL_StatusTypeDef ret;
	uint8_t bytes[2];
	int16_t regval;
	float Acurrent = 0.0f;
/** The battery current is coded in 2’s complement 14 bit format, and the LSB value is 11.77 uV. */
	ret = stc3100_read_regs(bat->hi2c,STC3100_REG_AIL,bytes,2);
	regval = (bytes[1]<<10) | (bytes[0] << 2);  // 16 bit signed
	regval /= 4;  // 16 to 14 bit format to get correct value
	if (ret == HAL_OK)
	{
		///current (A) = current_code * 11.77e-6 / Rsense (Ohm)
		Acurrent = (float)regval * 11.77e-6f / STC3100_RSENSE_RESISTANCE;
	}
	
	bat->current = Acurrent;
	
	return ret; 
}

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static HAL_StatusTypeDef stc3100_battery_rsoc(stc3100_device_info_t *bat)
{
	HAL_StatusTypeDef ret;
	uint8_t bytes[2];
	int16_t regval;
	float Ah = -10000.0f;
// The charge data is coded in 2’s complement format, and the LSB value is 6.70 uV.h.
	ret = stc3100_read_regs(bat->hi2c,STC3100_REG_RSOCL,bytes,2);
	regval = (bytes[1]<<10) | (bytes[0] << 2);  // 16 bit signed
	regval /= 4;  // 16 to 14 bit format to get correct value
	
	if (ret == HAL_OK)
	{
		Ah = (float)regval * 6.70e-6f / STC3100_RSENSE_RESISTANCE;
	}
	
	bat->rsoc = Ah;

	return ret;
}


/*
 * Return the STC3100 chip temperature or -1K if something wrong
 */
static HAL_StatusTypeDef stc3100_battery_temperature(stc3100_device_info_t *bat)
{
	HAL_StatusTypeDef ret;
	uint8_t bytes[2];
	int16_t regval;
	float tempdegC = -274.0f;
// The temperature value is coded in 2’s complement format, and the LSB value is 0.125° C.
	ret = stc3100_read_regs(bat->hi2c,STC3100_REG_TEMPL,bytes,2);
	regval = (bytes[1]<<10) | (bytes[0] << 2);  // 16 bit signed
	regval /= 4;  // 16 to 14 bit format to get correct value
	
	if (ret == HAL_OK)
	{
		tempdegC = (float)regval * 0.125f;
	}
	
	bat->temperature = tempdegC;

	return ret;
}


/**
  * @brief Return current status of the battery charging
	* @param bat 		pointer to Battery status structure 
  * @retval 			POWER_STATUS_CHARGING | POWER_STATUS_NOT_CHARGING
	*/
static uint8_t dc_charge_status(stc3100_device_info_t *bat)
{
	// determine from charging pin or from current sign if on-board charger is not present
#ifdef BATTERY_CHARGER
	return ((HAL_GPIO_ReadPin(CHARGER_STATUS_PORT, CHARGER_STATUS_PIN) == GPIO_PIN_RESET)) ? POWER_STATUS_CHARGING : POWER_STATUS_NOT_CHARGING;
#else
	stc3100_battery_current(bat);
	return (bat->current < 0.0f) ? POWER_STATUS_CHARGING : POWER_STATUS_NOT_CHARGING;
#endif
}


/**
  * @brief Get and update battery monitoring property
  * @param bat 		pointer to Battery status structure
  * @param prop		specific property
  */
HAL_StatusTypeDef Battery_Get(stc3100_device_info_t *bat, stc3100_battery_prop_t prop)
{
	switch (prop)
	{
		case BATTERY_PROP_PRESENT: 
			
		break;

		case BATTERY_PROP_VOLTAGE_NOW:
			return stc3100_battery_voltage(bat);

		case BATTERY_PROP_CURRENT_NOW:
			return stc3100_battery_current(bat);

		case BATTERY_PROP_CAPACITY:
			return stc3100_battery_rsoc(bat);

		case BATTERY_PROP_TEMP:
			return stc3100_battery_temperature(bat);
		
		case BATTERY_PROP_UNIQUEID:
			return stc3100_read_ID(bat);
		
		case BATTERY_PROP_CHARGING:
			bat->charging = dc_charge_status(bat);
		break;
		
	}

	return HAL_ERROR;  // unknown property
	
}






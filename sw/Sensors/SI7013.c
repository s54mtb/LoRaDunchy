 /***
 *      ____    ____  _  ____   ____    ___   __  __ 
 *     / ___|  / ___|(_)|  _ \ |  _ \  / _ \ |  \/  |
 *     \___ \ | |    | || | | || |_) || | | || |\/| |
 *      ___) || |___ | || |_| ||  _ < | |_| || |  | |
 *     |____/  \____||_||____/ |_| \_\ \___/ |_|  |_|
 *        (C)2018 Scidrom 
 
	Description: Si7013 driver
	License: GNU General Public License
	Maintainer: S54MTB
*/

/******************************************************************************
  * @file    SI7013.c
  * @author  S54MTB
  * @version V1.0.0
  * @date    12-January-2018
  * @brief   MAG3110 driver
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
/**!
  * Uncoment to use osDelay() instead of HAL_Delay()
	*/
#define USE_OS_DELAY 1

#include "si7013.h"


/**!
  * Si7013 user registers
	*/
si7013_userReg_t si7013_userRegs;


/*
 * si7013_read_reg() - Read User register
 * @hi2c:  handle to I2C interface
 * @reg: Register read command
 * @val: 8-bit register value from the Si7013
 * Returns HAL status or HAL_ERROR for invalid parameters.
 */
HAL_StatusTypeDef si7013_read_reg(I2C_HandleTypeDef *hi2c, 
						uint8_t adr, uint8_t reg, uint8_t *val)
{
	uint8_t buf[2];
	HAL_StatusTypeDef  error;


	// Check argument
	if ((reg != SI7013_CMD_READ_REGISTER_1) &
		  (reg != SI7013_CMD_READ_REGISTER_2) &
	    (reg != SI7013_CMD_READ_REGISTER_3) )
		return HAL_ERROR;
	
	buf[0] = reg;
	/* Read user register */
	/* Send the command */
	error = HAL_I2C_Master_Transmit(hi2c,adr<<1,buf,1,100);
	if (error != HAL_OK)
		return error;

	HAL_Delay(5); 
	
	/* Receive a 1-byte result */
	error = HAL_I2C_Master_Receive(hi2c, adr<<1 | 0x01, buf, 1, 1000);
	if (error != HAL_OK)
		return error;
	
	/* Result */
	*val = buf[0]; 

	return HAL_OK;  /* Success */
	
}


/*
 * si7013_write_reg() - Read User register
 * @hi2c:  handle to I2C interface
 * @reg: Register read command
 * @val: 8-bit register value from the Si7013
 * Returns HAL status or HAL_ERROR for invalid arguments.
 */
HAL_StatusTypeDef si7013_write_reg(I2C_HandleTypeDef *hi2c, 
				uint8_t adr, uint8_t reg, uint8_t val)
{
	uint8_t buf[2];
	HAL_StatusTypeDef  error;

		// Check argument
	if ((reg != SI7013_CMD_WRITE_REGISTER_1) &
		  (reg != SI7013_CMD_WRITE_REGISTER_2) &
	    (reg != SI7013_CMD_WRITE_REGISTER_3) )
		return HAL_ERROR;

	buf[0] = reg;
	buf[1] = val;
	/* Write user register */
	/* Send the command and data */
	error = HAL_I2C_Master_Transmit(hi2c,adr<<1,buf,2,100);
	if (error != HAL_OK)
		return error;

	return HAL_OK;  /* Success */
	
}



/*
 * si7013_read_regs() - Read User registers
 * @hi2c:  handle to I2C interface
 * regs: register file
 * Returns HAL status 
 */
HAL_StatusTypeDef si7013_read_regs(I2C_HandleTypeDef *hi2c, 
								uint8_t adr, si7013_userReg_t *Regs)
{
	HAL_StatusTypeDef  error;
	uint8_t r;

	error = si7013_read_reg(hi2c, adr, SI7013_CMD_READ_REGISTER_1, &r);
	if (error != HAL_OK) return error;
	Regs->reg1.r = r;
	
	error = si7013_read_reg(hi2c, adr, SI7013_CMD_READ_REGISTER_2, &r);
	if (error != HAL_OK) return error;
	Regs->reg2.r = r;

	error = si7013_read_reg(hi2c, adr, SI7013_CMD_READ_REGISTER_3, &r);
	if (error != HAL_OK) return error;
	Regs->reg3.r = r;

	return HAL_OK;  /* Success */
	
}



/*
 * si7013_write_regs() - Write User registers
 * @hi2c:  handle to I2C interface
 * regs: register file
 * Returns HAL status 
 */
HAL_StatusTypeDef si7013_write_regs(I2C_HandleTypeDef *hi2c, 
								uint8_t adr, si7013_userReg_t *Regs)
{
	HAL_StatusTypeDef  error;
	uint8_t r;

	r = Regs->reg1.r;
	error = si7013_write_reg(hi2c, adr, SI7013_CMD_WRITE_REGISTER_1, r);
	if (error != HAL_OK) return error;

	r = Regs->reg2.r;
	error = si7013_write_reg(hi2c, adr, SI7013_CMD_WRITE_REGISTER_2, r);
	if (error != HAL_OK) return error;

	r = Regs->reg3.r;
	error = si7013_write_reg(hi2c, adr, SI7013_CMD_WRITE_REGISTER_3, r);
	if (error != HAL_OK) return error;
	

	return HAL_OK;  /* Success */
	
}




/*
 * si7013_measure() - Take a measurement
 * @hi2c:  handle to I2C interface
 * @command: Command for measuring temperature or humidity
 * @val: 16-bit value from the Si7013
 * Returns HAL status.
 */
HAL_StatusTypeDef si7013_measure(I2C_HandleTypeDef *hi2c, uint8_t adr, 
									uint8_t command, uint16_t *val)
{
	uint8_t buf[2];
	HAL_StatusTypeDef  error;


	buf[0] = command;
	/* Measure the humidity or temperature value */
	/* Send the command */
	error = HAL_I2C_Master_Transmit(hi2c,adr<<1,buf,1,100);
	if (error != HAL_OK)
		return error;

	HAL_Delay(15); 
	
	/* Receive the 2-byte result */
	error = HAL_I2C_Master_Receive(hi2c, adr<<1 | 0x01, buf, 2, 1000);
	if (error != HAL_OK)
		return error;

	
	/* Swap the bytes and clear the status bits */
	*val = buf[0] * 256 + buf[1] & ~3; 

	return HAL_OK;  /* Success */
	
}


/*
 * si7013_measure_intemperature() - internal temperature measurement 
 * @hi2c:  handle to I2C interface
 * @temperature:  (signed) 32 bit temperature result, unit is m°C
 * Returns HAL status.
 */
HAL_StatusTypeDef si7013_measure_intemperature(I2C_HandleTypeDef *hi2c,  
			uint8_t adr, 	int32_t *temperature)
{
	uint16_t value;
	HAL_StatusTypeDef  error;

	/* Measure the temperature value */
	error = si7013_measure(hi2c, adr, SI7013_CMD_MEASURE_TEMPERATURE_HOLD, &value);
	if (error != HAL_OK)
		return error;

	/* Convert the value to millidegrees Celsius */
	*temperature = ((value*21965)>>13)-46850;

	return HAL_OK;
}


/*
 * si7013_measure_thermistor() - external temperature measurement 
 * @hi2c:  handle to I2C interface
 * @value:  (signed) 16 bit conversion result
 * Returns HAL status.
 */
HAL_StatusTypeDef si7013_measure_thermistor(I2C_HandleTypeDef *hi2c,  
										uint8_t adr, int16_t *value)
{
	HAL_StatusTypeDef  error;
  uint16_t v;
	
	/* Measure the temperature value */
	error = si7013_measure(hi2c, adr, SI7013_CMD_MEASURE_THERMISTOR_HOLD, &v);
	if (error != HAL_OK)
		return error;

	*value = (int16_t)v;

	return HAL_OK;
}



/*
 * si7013_measure_humidity() - internal temperature measurement 
 * @hi2c:  handle to I2C interface
 * @temperature:  (signed) 32 bit humidity result, unit is mRH%
 * Returns HAL status.
 */
HAL_StatusTypeDef si7013_measure_humidity(I2C_HandleTypeDef *hi2c,  uint8_t adr,
	int32_t *humidity)
{
	uint16_t value;
	HAL_StatusTypeDef  error;
	int32_t rh;

	/* Measure the temperature value */
	error = si7013_measure(hi2c, adr, SI7013_CMD_MEASURE_HUMIDITY_HOLD, &value);
	if (error != HAL_OK)
		return error;

	/* Convert the value to milli-percent (pcm) relative humidity */
	rh = ((value*15625)>>13)-6000;

	/* Limit the humidity to valid values */
	if (rh < 0)
		*humidity = 0;
	else if (rh > 100000)
		*humidity = 100000;
	else
		*humidity = rh;		

	return HAL_OK;
}


/*
 * si7013_get_device_id() - Get the device ID from the device
 * @i2c: handle to I2C interface
 * @id: pointer to device ID
 *
 * Returns 0 on success.
 */
HAL_StatusTypeDef si7013_get_device_id(I2C_HandleTypeDef *hi2c,  uint8_t adr, 
									uint8_t *id)
{
	uint8_t buf[6];
	HAL_StatusTypeDef  error;

	/* Put the 2-byte command into the buffer */
	buf[0] = 0xFC;
	buf[1] = 0xC9;

	/* Send the command */
	error = HAL_I2C_Master_Transmit(hi2c,adr<<1,buf,2,100);
	if (error != HAL_OK)
	       return error;

	/* Receive the 6-byte result */
	error = HAL_I2C_Master_Receive(hi2c, adr<<1 | 0x01, buf, 6, 100);
	if (error != HAL_OK)
	       return error;

	/* Return the device ID */
	*id = buf[0];
	
	return HAL_OK;  /* Success */
}

/************ EOF *************************************************************/


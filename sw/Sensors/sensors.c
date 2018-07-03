 /***
 *      ____    ____  _  ____   ____    ___   __  __ 
 *     / ___|  / ___|(_)|  _ \ |  _ \  / _ \ |  \/  |
 *     \___ \ | |    | || | | || |_) || | | || |\/| |
 *      ___) || |___ | || |_| ||  _ < | |_| || |  | |
 *     |____/  \____||_||____/ |_| \_\ \___/ |_|  |_|
 *        (C)2018 Scidrom 
 
	Description: Sensors handler
	License: GNU General Public License
	Maintainer: S54MTB
*/

/******************************************************************************
  * @file    sensors.c
  * @author  S54MTB
  * @version V1.0.0
  * @date    15-June-2018
  * @brief   Sensors handler file
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

#include "sensors.h"

static HAL_StatusTypeDef SEN_I2C1_Init(void);

/**
 * I2C handle
 */
I2C_HandleTypeDef SEN_hi2c1;

#ifdef BATTERY_FUEL_GAUGE
	stc3100_device_info_t bat;
#endif

/**
  * @brief  This function initialize all sensors in the system.  
  * @param  None
  * @retval Initialization status: 0 = OK, >0 not OK
  */
HAL_StatusTypeDef SensorsInit(void)
{
	HAL_StatusTypeDef Status;  // HAL_OK = 0, can be or'd 
#ifdef HPM_SENSOR
	Status = HPM_Init();
#endif
	Status |= SEN_I2C1_Init();
	
#ifdef BME280_SENSOR 
	bme280_init_dev(&SEN_hi2c1);
#endif
	
#ifdef BATTERY_FUEL_GAUGE
	bat.hi2c = &SEN_hi2c1;
	Battery_Init(&bat);
#endif
	
#ifdef BATTERY_CHARGER
  stc3100_battery_chargerinit(CHARGER_STATUS_PORT, CHARGER_STATUS_PIN);
#endif	
	
	return Status;
}


/*** I2C is initialized and handled here... many sensors can be attached to I2C */

/**
  * @brief  This function initialize I2C subsystem.  
  * @param  None
  * @retval Initialization status HAL_OK when all OK
  * @Note: I2C pins fixed to PB8/PB9
  */
static HAL_StatusTypeDef SEN_I2C1_Init(void)
{
	
  GPIO_InitTypeDef GPIO_InitStruct;
  HAL_StatusTypeDef Status;
	
//	RCC_PeriphCLKInitTypeDef PeriphClkInit;

  __HAL_RCC_GPIOB_CLK_ENABLE();
	/**I2C1 GPIO Configuration    
	PB9     ------> I2C1_SDA
	PB8     ------> I2C1_SCL 
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	
	/* Peripheral clock enable */
	__HAL_RCC_I2C1_CLK_ENABLE();

//	__HAL_RCC_I2C1_FORCE_RESET();
//	HAL_Delay(2);
//	__HAL_RCC_I2C1_RELEASE_RESET();

//	RCC->APB1ENR |= 1U<<21;  // Enable I2C clock

  SEN_hi2c1.Instance = I2C1;
  SEN_hi2c1.Init.Timing = 0x00707CBB;
  SEN_hi2c1.Init.OwnAddress1 = 0;
  SEN_hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  SEN_hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  SEN_hi2c1.Init.OwnAddress2 = 0;
  SEN_hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  SEN_hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  SEN_hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	Status = HAL_I2C_Init(&SEN_hi2c1);
  if (Status != HAL_OK)
  {
    return Status;
  }

    /**Configure Analog filter 
    */
	Status = HAL_I2CEx_ConfigAnalogFilter(&SEN_hi2c1, I2C_ANALOGFILTER_ENABLE);
  if (Status != HAL_OK)
  {
    return Status;
  }

    /**Configure Digital filter 
    */
	return HAL_I2CEx_ConfigDigitalFilter(&SEN_hi2c1, 0);
}


void Sensor_readouts(sen_readout_t *readouts)
{
#ifdef HPM_SENSOR
	uint16_t pm10, pm2_5;
	HPM_Ack_t hpmack;
#endif
	
#ifdef SI7013_SENSOR
	int32_t si7013_t;
	int32_t si7013_rh;
#endif

#ifdef TMP75_SENSOR
	int32_t Tmp75_t;
#endif

#ifdef SHT31_SENSOR
  int32_t	sht31_t; 
	int32_t	sht31_rh;
#endif

#ifdef BME280_SENSOR
	int32_t	bme28_t;
	int32_t	bme280_rh;
	int32_t bme280_press;
#endif
	
#ifdef BATTERY_FUEL_GAUGE
  Battery_Get(&bat, BATTERY_PROP_UNIQUEID);
  Battery_Get(&bat, BATTERY_PROP_VOLTAGE_NOW);  
	Battery_Get(&bat, BATTERY_PROP_CURRENT_NOW);
	Battery_Get(&bat, BATTERY_PROP_CAPACITY);
	Battery_Get(&bat, BATTERY_PROP_TEMP);
	Battery_Get(&bat, BATTERY_PROP_CHARGING);
	readouts->battery = (uint16_t)(1000 * bat.rsoc);
	readouts->current = (int8_t)(1000 * bat.current);
	readouts->voltage = (uint8_t)(100 * (bat.voltage - 2.500f));
#endif

#ifdef HPM_SENSOR
	HPM_get();
	hpmack = HPM_GetAck();
	if (hpmack == HPM_ACK)
	{
		pm10 = HPM_LastReadout(HPM_READOUT_PM10);
		pm2_5 = HPM_LastReadout(HPM_READOUT_PM2_5);
	}
	readouts->pm2_5 = pm2_5;
	readouts->pm10 = pm10;
#endif
	

#ifdef SI7013_SENSOR
	si7013_measure_intemperature(&SEN_hi2c1, SI7013_ADDR, &si7013_t);  // return in m°C
	si7013_measure_humidity(&SEN_hi2c1, SI7013_ADDR, &si7013_rh); 
	readouts->si7013_T = (si7013_t) / 10;
	readouts->si7013_RH = si7013_rh / 1000;
#endif
	
#ifdef TMP75_SENSOR
	Tmp75_Read_Int_Teperature(&SEN_hi2c1, Tmp75_SlaveAddress(Tmp75Addr_Zero, Tmp75Addr_Zero, Tmp75Addr_One), &Tmp75_t);
	readouts->tmp75_T = (Tmp75_t) / 10;  // hundreds of deg. C
#endif
	
#ifdef SHT31_SENSOR	
	sht31_meas_oneshot_int(&SEN_hi2c1, SHT31_ADDR, SHT31_rep_High, &sht31_t, &sht31_rh);
	readouts->sht31_T = (sht31_t) / 10;
	readouts->sht31_RH = sht31_rh / 1000;
#endif
	
#ifdef BME280_SENSOR
	bme280_int_readout(&bme28_t, &bme280_rh, &bme280_press);
	readouts->bme280_T = (bme28_t ) / 10;
	readouts->bme280_RH = bme280_rh / 1000;
	readouts->bme280_p = bme280_press / 10000;
#endif
	
	
}

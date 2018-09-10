 /***
 *      ____    ____  _  ____   ____    ___   __  __ 
 *     / ___|  / ___|(_)|  _ \ |  _ \  / _ \ |  \/  |
 *     \___ \ | |    | || | | || |_) || | | || |\/| |
 *      ___) || |___ | || |_| ||  _ < | |_| || |  | |
 *     |____/  \____||_||____/ |_| \_\ \___/ |_|  |_|
 *        (C)2018 Scidrom 
 
	Description: Loradunchy I2C driver
	License: GNU General Public License
	Maintainer: S54MTB
*/

/******************************************************************************
  * @file    i2c.c
  * @author  S54MTB
  * @version V1.0.0
  * @date    12-August-2018
  * @brief   I2C driver
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



#include "stm32l0xx.h"                  // Device header


/* Fixed setup for I2C1 on pins PB8---SCL and PB9---SDA */
/* Add external pullups 10kOhm! */

#define DUNCHY_I2C_TIMING_100KHZ       0x10A13E56 /* Analog Filter ON, Rise Time 400ns, Fall Time 100ns */
#define DUNCHY_I2C_TIMING_400KHZ       0x00B1112E /* Analog Filter ON, Rise Time 250ns, Fall Time 100ns */
#define DUNCHY_I2C                            I2C1
#define DUNCHY_I2C_CLK_ENABLE()               __I2C1_CLK_ENABLE()
#define DUNCHY_I2C_SCL_SDA_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()
#define DUNCHY_I2C_SCL_SDA_AF                 GPIO_AF4_I2C1
#define DUNCHY_I2C_SCL_SDA_GPIO_PORT          GPIOB
#define DUNCHY_I2C_SCL_PIN                    GPIO_PIN_8
#define DUNCHY_I2C_SDA_PIN                    GPIO_PIN_9

#define DUNCHY_I2C_FORCE_RESET()              __I2C1_FORCE_RESET()
#define DUNCHY_I2C_RELEASE_RESET()            __I2C1_RELEASE_RESET()

#define DUNCHY_I2C_EV_IRQn                    I2C1_IRQn

#define I2C_Loradunchy_Timeout  100

/** Main I2C handle */
I2C_HandleTypeDef I2C_Loradunchy_Handle;


static void I2C_Loradunchy_MspInit( void )
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable I2C GPIO clocks */
  DUNCHY_I2C_SCL_SDA_GPIO_CLK_ENABLE();

  /* I2C_Loradunchy SCL and SDA pins configuration -------------------------------------*/
  GPIO_InitStruct.Pin        = DUNCHY_I2C_SCL_PIN | DUNCHY_I2C_SDA_PIN;
  GPIO_InitStruct.Mode       = GPIO_MODE_AF_OD;
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L0XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO))|| (defined (USE_B_L072Z_LRWAN1)))
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
#endif

#if (defined (USE_STM32L1XX_NUCLEO))
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
#endif
  GPIO_InitStruct.Pull       = GPIO_NOPULL;
  GPIO_InitStruct.Alternate  = DUNCHY_I2C_SCL_SDA_AF;

  HAL_GPIO_Init( DUNCHY_I2C_SCL_SDA_GPIO_PORT, &GPIO_InitStruct );

  /* Enable the I2C_Loradunchy peripheral clock */
  DUNCHY_I2C_CLK_ENABLE();

  /* Force the I2C peripheral clock reset */
  DUNCHY_I2C_FORCE_RESET();

  /* Release the I2C peripheral clock reset */
  DUNCHY_I2C_RELEASE_RESET();

  /* Enable and set I2C_Loradunchy Interrupt to the highest priority */
  HAL_NVIC_SetPriority(DUNCHY_I2C_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DUNCHY_I2C_EV_IRQn);

#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)))
  /* Enable and set I2C_Loradunchy Interrupt to the highest priority */
  HAL_NVIC_SetPriority(DUNCHY_I2C_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DUNCHY_I2C_ER_IRQn);
#endif

}




/**
 * @brief  Configures I2C interface.
 * @param  None
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t I2C_Loradunchy_Init( void )
{
  if(HAL_I2C_GetState( &I2C_Loradunchy_Handle) == HAL_I2C_STATE_RESET )
  {

    /* I2C_Loradunchy peripheral configuration */
    I2C_Loradunchy_Handle.Init.Timing = DUNCHY_I2C_TIMING_400KHZ;    /* 400KHz */
    I2C_Loradunchy_Handle.Init.OwnAddress1    = 0x33;
    I2C_Loradunchy_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    I2C_Loradunchy_Handle.Instance            = DUNCHY_I2C;

    /* Init the I2C */
    I2C_Loradunchy_MspInit();
    HAL_I2C_Init( &I2C_Loradunchy_Handle );
  }

  if( HAL_I2C_GetState( &I2C_Loradunchy_Handle) == HAL_I2C_STATE_READY )
  {
    return 0;
  }
  else
  {
    return 1;
  }
}



/**
 * @brief  Write data to the register of the device through I2C
 * @param  Addr Device address on I2C
 * @param  Reg The target register address to be written
 * @param  pBuffer The data to be written
 * @param  Size Number of bytes to be written
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t I2C_Loradunchy_WriteData( uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size )
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Write( &I2C_Loradunchy_Handle, Addr, ( uint16_t )Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Size,
                              I2C_Loradunchy_Timeout );

  /* Check the communication status */
  if( status != HAL_OK )
  {

    /* Execute user timeout callback */
    //I2C_Loradunchy_Error( Addr );
    return 1;
  }
  else
  {
    return 0;
  }

}



/**
 * @brief  Read a register of the device through I2C
 * @param  Addr Device address on I2C
 * @param  Reg The target register address to read
 * @param  pBuffer The data to be read
 * @param  Size Number of bytes to be read
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t I2C_Loradunchy_ReadData( uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size )
{

  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Read( &I2C_Loradunchy_Handle, Addr, ( uint16_t )Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Size,
                             I2C_Loradunchy_Timeout );

  /* Check the communication status */
  if( status != HAL_OK )
  {

    /* Execute user timeout callback */
    //I2C_Loradunchy_Error( Addr );
    return 1;
  }
  else
  {
    return 0;
  }
}




 /***
 *      ____    ____  _  ____   ____    ___   __  __ 
 *     / ___|  / ___|(_)|  _ \ |  _ \  / _ \ |  \/  |
 *     \___ \ | |    | || | | || |_) || | | || |\/| |
 *      ___) || |___ | || |_| ||  _ < | |_| || |  | |
 *     |____/  \____||_||____/ |_| \_\ \___/ |_|  |_|
 *        (C)2018 Scidrom 
 
	Description: Honeywell HPM sensor driver
	License: GNU General Public License
	Maintainer: S54MTB
*/

/******************************************************************************
  * @file    HPM.c
  * @author  S54MTB
  * @version V1.0.0
  * @date    12-January-2018
  * @brief   Honeywell HPM particle sensor driver, UART should be initialized
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


#include "HPM.h"
#include "stm32l0xx.h"                  // Device header
#include <string.h> 										// memcmp


/** Local functions */
static HAL_StatusTypeDef HPM_SendCmd(uint8_t *cmd, uint8_t len, uint8_t waitack);

/**
 * HPM Sensor UART handle
 */
UART_HandleTypeDef HPM_UartHandle;

///**
// * HPM Rx buffer
// */
#define HPM_MAX_BUF 64
//static uint8_t HPM_RxBuf[HPM_MAX_BUF];

static uint16_t HPM_LastPM10=10;
static uint16_t HPM_LastPM2_5=10;

__IO ITStatus HPM_UART_Ready = RESET;

static HPM_Ack_t HPM_LastAck = HPM_ACK_NONE;

const uint8_t HPM_READ_cmd[4] = HPM_READ;
const uint8_t HPM_START_cmd[4] = HPM_START;
const uint8_t HPM_STOP_cmd[4] = HPM_STOP;
const uint8_t HPM_START_AUTO_cmd[4] = HPM_START_AUTO;
const uint8_t HPM_STOP_AUTO_cmd[4] = HPM_STOP_AUTO;
const uint8_t HPM_AUTOSEND_SIG_cmd[4] = HPM_AUTO_SEND_SIG;
const uint8_t HPM_GET_COEFF_cmd[4] = HPM_GET_COEFF;
const uint8_t HPM_NACK_cmd[2] = {0x96, 0x96};
const uint8_t HPM_ACK_cmd[2] = {0xa5, 0xa5};



/***************** UART low level functions ************/


/**
 * Init HPM PM sensor: 
 *   Init GPIO
 *     PA2 ... Tx
 *     PA3 ... Rx
 *   Init USART2: 
 *	   9600 baud, 8, N, 1
 *  	 Init ISR
 *  When HPM sensor is used, only this sensor is connected to uart2
 */
HAL_StatusTypeDef HPM_Init(void)
{
	HAL_StatusTypeDef HPM_Init_Status;
  GPIO_InitTypeDef  GPIO_InitStruct;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
	__HAL_RCC_GPIOA_CLK_ENABLE();

  /* Enable USARTx clock */
	__HAL_RCC_USART2_CLK_ENABLE();
	
  /*##-2- Configure peripheral GPIO ##########################################*/  
  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = GPIO_PIN_2;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_USART2;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Alternate = GPIO_AF4_USART2;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*##-3- Configure the NVIC for UART ########################################*/
  /* NVIC for USART */
	
	HPM_UartHandle.Instance        = USART2;
  HPM_UartHandle.Init.BaudRate   = 9600;
  HPM_UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  HPM_UartHandle.Init.StopBits   = UART_STOPBITS_1;
  HPM_UartHandle.Init.Parity     = UART_PARITY_NONE;
  HPM_UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  HPM_UartHandle.Init.Mode       = UART_MODE_TX_RX;
  
	HPM_Init_Status = HAL_UART_Init(&HPM_UartHandle);
  if(HPM_Init_Status != HAL_OK)
  {
    /* Initialization Error */
    return HPM_Init_Status;
  }
  
	//HPM_UartHandle.Instance->CR1 |= USART_CR1_RXNEIE;  // enable RXNE IRQ
	
  HAL_NVIC_SetPriority(USART2_IRQn, 0x1, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
	
	/*  Init PA0 for driving the supply pin for HPM sensor */
  GPIO_InitStruct.Pin       = GPIO_PIN_0;
  GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);	
	
	return HAL_OK;

}


/**
  * @brief  This function handles UART interrupt request.  
  * @param  None
  * @retval None
  */
void USART2_IRQHandler( void )
{
	HAL_UART_IRQHandler(&HPM_UartHandle);
}


/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This function reports end of HPM UART Rx transfer
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  HPM_UART_Ready = SET;
}


/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This function reports end of HPM UART Tx transfer 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  HPM_UART_Ready = SET; 
}



void HPM_Power(uint8_t ppp)
{
	if (ppp>0)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);		
	}
	
}


/**************** End of UART Low level functions ******/



void HPM_Reset(void)
{
	HPM_Power(0);
	HAL_Delay(10);
	HPM_Power(0);
  HAL_Delay(100);
}

/**
  * Claculate checksum for automatic response mode 
  * Return 1 if calculated and received match
  */
static uint8_t HPM_checksum_check(uint8_t *buf, uint16_t len, uint16_t check)
{
	uint16_t i;
	uint16_t checkx = 0;
	
	for (i=0; i<len; i++)
	{
		checkx += buf[i];
	}
	
	return checkx == check ? 1 : 0;
}
	
/**
  * Calculate normal response checksum and compare with received 
  * Return 1 if calculated and received match
  */
static uint8_t HPM_CS_Check(uint8_t *buf, uint8_t len, uint8_t check)
{
	uint32_t sum = 0;
	uint8_t i, chkx;
	
	for (i=0; i<len; i++)
	{
		sum+=buf[i];
	}
	sum = 65536-sum;
	chkx = (uint8_t)(sum % 0x100);
	return chkx == check ? 1 : 0;
}


/**
  * @brief  Return last readouts  
  * @param  HPM_ReadoutType: 	HPM_READOUT_PM10 for PM10 or HPM_READOUT_PM2_5 for PM2,5 value
  * @retval PM readout 
  */
uint16_t HPM_LastReadout(HPM_Readout_Mode_t HPM_ReadoutType)
{
	switch (HPM_ReadoutType)
	{
		case HPM_READOUT_PM10 : return HPM_LastPM10;
		case HPM_READOUT_PM2_5 : return HPM_LastPM2_5;
	}
  return 0xffff;
}


/**
  * @brief Return last ACK, clear flags after read
  */
HPM_Ack_t HPM_GetAck(void)
{
	HPM_Ack_t lack = HPM_LastAck;
	HPM_LastAck = HPM_ACK_NONE;
	return lack;
}


/**
  * @brief  This function Reads valuse from auto response  
  * @param  none
  * @retval UART transfer status
  * @sideeffects last ack status changed when valid response with ACK/NACK
  */
HAL_StatusTypeDef HPM_get_auto(void)
{
	uint16_t i = 0;
	HAL_StatusTypeDef Status;
	uint8_t HPM_RxBuf[HPM_MAX_BUF];
		
	Status = HAL_UART_Receive_IT(&HPM_UartHandle, HPM_RxBuf, 32);
	if (Status != HAL_OK)
	{
		HPM_Reset();
		return Status;
	}
	
	HPM_UART_Ready = RESET;
	while ((HPM_UART_Ready != SET) & (i < 120))  // wait for rx or timeout 
	{
		HAL_Delay(10);
		i++;
	}
	
	if (memcmp(HPM_AUTOSEND_SIG_cmd, HPM_RxBuf, 4) == 0)  // is start of the buffer correct ?
	{
		if (HPM_checksum_check(HPM_RxBuf, 29, HPM_RxBuf[30] * 256 + HPM_RxBuf[31]) == 1)  // is checsum OK ?
		{
			HPM_LastPM10 = HPM_RxBuf[8] * 256 + HPM_RxBuf[9];
			HPM_LastPM2_5 = HPM_RxBuf[6] * 256 + HPM_RxBuf[7];
			HPM_LastAck = HPM_ACK;
		}	
		else // checksum not OK --- NACK! 
		{
			HPM_LastAck = HPM_NACK;
		}
	}
	else // start of the buffer not recognised... start over
	{
		HPM_LastAck = HPM_NACK;
	}
	for (i=0; i<HPM_MAX_BUF; i++) HPM_RxBuf[i] = 0;
	return Status;
}


/**
  * @brief  This function sends command to HPM sensor. If command has no response it checks the ACK from sensor.  
  * @param  cmd: command array
  * @param  len: command length (including CS)
  * @param  waitack: 1 - wait for ack, 0 - don't wait for ack
  * @retval UART transfer status
  * @sideeffects last ack status changed when valid response with ACK/NACK
  */
static HAL_StatusTypeDef HPM_SendCmd(uint8_t *cmd, uint8_t len, uint8_t waitack)
{
	uint16_t i = 0;
	uint8_t HPM_RxBuf[HPM_MAX_BUF];
	HAL_StatusTypeDef Status = HAL_UART_Transmit_IT(&HPM_UartHandle, (uint8_t *)cmd, 4);
	if (Status == HAL_OK)
	{
		HPM_UART_Ready = RESET;
		while ((HPM_UART_Ready != SET) & (i < 800))  // wait for rx or timeout 
		{
			HAL_Delay(1);
			i++;
		}
  }
	else
	{
		return Status;
	}
	
//	if ((cmd[2]==0x01) | (cmd[2]==0x02) | (cmd[2]==0x08) | (cmd[2]==0x20) | (cmd[2]==0x40)) // commands without response, just ack/nack
	if (waitack == 1)
	{
		Status = HAL_UART_Receive_IT(&HPM_UartHandle, HPM_RxBuf, 2);
		if (Status == HAL_OK)
		{
			HPM_UART_Ready = RESET;
			while ((HPM_UART_Ready != SET) & (i < 120))  // wait for rx or timeout 
			{
				HAL_Delay(10);
				i++;
			}
		}
		else
		{
			HPM_Reset();
			return Status;
		}
		
		if (HPM_RxBuf[0] == 0x96) HPM_LastAck = HPM_NACK;
		if (HPM_RxBuf[0] == 0xA5) HPM_LastAck = HPM_ACK;
	}
  return Status;
}


/**
  * @brief  This function Reads valuse from HPM sensor with normal query and response  
  * @param  none
  * @retval UART transfer status
  * @sideeffects readout values and last ack status changed when valid response with ACK/NACK
  */
HAL_StatusTypeDef HPM_get(void)
{
	uint8_t HPM_RxBuf[HPM_MAX_BUF];
	uint16_t i = 0;
	HAL_StatusTypeDef Status;
	
	for (i=0; i<HPM_MAX_BUF; i++) HPM_RxBuf[i] = 0;
	
	Status = HPM_SendCmd((uint8_t *)HPM_STOP_AUTO_cmd, 4, 1);
	if (Status != HAL_OK) return Status;
	
	Status = HPM_SendCmd((uint8_t *)HPM_READ_cmd, 4, 0);
	if (Status != HAL_OK) return Status;
	
	
	Status = HAL_UART_Receive_IT(&HPM_UartHandle, HPM_RxBuf, 8);
	if (Status == HAL_OK)
	{
		HPM_UART_Ready = RESET;
		while ((HPM_UART_Ready != SET) & (i < 800))  // wait for rx or timeout 
		{
			HAL_Delay(1);
			i++;
		}
		if (HPM_CS_Check(HPM_RxBuf, 7, HPM_RxBuf[7]) == 1) // check received data signature
		{
			HPM_LastPM10 = HPM_RxBuf[5] * 256 + HPM_RxBuf[6];
			HPM_LastPM2_5 = HPM_RxBuf[3] * 256 + HPM_RxBuf[4];
			HPM_LastAck = HPM_ACK;
		}
		else // received signature not ok
		{
			HPM_LastAck = HPM_NACK;
		}

  } 
	else // receive error
	{
		HPM_Reset();
		return Status;
	}
	
	
	return Status;
	
}


/**
  * @brief  This function send command to Start Particle Measurement  
  * @param  none
  * @retval UART transfer status
  */
HAL_StatusTypeDef HPM_Start(void)
{
	return HPM_SendCmd((uint8_t *)HPM_START_cmd, 4, 1);
}


/**
  * @brief  This function send command to Stop Particle Measurement  
  * @param  none
  * @retval UART transfer status
  */
HAL_StatusTypeDef HPM_Stop(void)
{
	return HPM_SendCmd((uint8_t *)HPM_STOP_cmd, 4, 1);
}

/**
  * @brief  This function send command to enable Auto Send 
  * @param  none
  * @retval UART transfer status
  */
HAL_StatusTypeDef HPM_StartAuto(void)
{
	return HPM_SendCmd((uint8_t *)HPM_START_AUTO_cmd, 4, 1);
}


/**
  * @brief  This function send command to disable Auto Send 
  * @param  none
  * @retval UART transfer status
  */
HAL_StatusTypeDef HPM_StopAuto(void)
{
	return HPM_SendCmd((uint8_t *)HPM_STOP_AUTO_cmd, 4, 1);
}


/**
  * @brief  This function send command to Set Customer Adjustment Coefficient
  * @param  Coeff: 30 ~ 200
  * @retval UART transfer status
  */
HAL_StatusTypeDef HPM_SetAdjCoeff(uint8_t Coeff)
{
	uint8_t buf[5] = {0x68, 0x02, 0x08, 0x00, 0x00};
	
	buf[3] = Coeff;
	buf[4] = (uint8_t)((65536-(buf[0]+buf[1]+buf[2]+buf[3])) % 256);
		
	return HPM_SendCmd(buf, 5, 1);
}

/**
  * @brief  This function sends command to Read Customer Adjustment Coefficient
  * @param[out]  Coeff: Adjustment coefficient
  * @retval UART transfer status
  */
HAL_StatusTypeDef HPM_GetAdjCoeff(uint8_t *Coeff)
{
	uint8_t HPM_RxBuf[HPM_MAX_BUF];
	uint16_t i = 0;
	HAL_StatusTypeDef Status;
	
	// Stop auto send first to avoid errors in Rx
	Status = HPM_SendCmd((uint8_t *)HPM_STOP_AUTO_cmd, 4, 1);
	if (Status != HAL_OK) return Status;
	
	Status = HPM_SendCmd((uint8_t *)HPM_GET_COEFF_cmd, 4, 0);
	if (Status != HAL_OK) return Status;
	
	
	Status = HAL_UART_Receive_IT(&HPM_UartHandle, HPM_RxBuf, 5);
	if (Status == HAL_OK)
	{
		HPM_UART_Ready = RESET;
		while ((HPM_UART_Ready != SET) & (i < 120))  // wait for rx or timeout 
		{
			HAL_Delay(10);
			i++;
		}
		if (HPM_CS_Check(HPM_RxBuf, 4, HPM_RxBuf[4]) == 1) // check received data signature
		{
			*Coeff = HPM_RxBuf[3];
			HPM_LastAck = HPM_ACK;
		}
		else // received signature not ok
		{
			HPM_LastAck = HPM_NACK;
		}

  } 
	else // receive error
	{
		HPM_Reset();
		return Status;
	}
	
	return Status;
	
}






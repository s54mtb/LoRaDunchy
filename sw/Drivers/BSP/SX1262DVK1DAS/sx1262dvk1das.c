/*!
 * \file      sx1262dvk1das-board.c
 *
 * \brief     Target board SX1261DVK1BAS shield driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
 /*******************************************************************************
  * @file    sx1262dvk1das.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    10-July-2018
  * @brief   driver sx1262dvk1das board
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/

#include "hw.h"
#include "radio.h"
#include "sx126x_board.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BOARD_TCXO_WAKEUP_TIME  5 // no TCXO

#ifndef USE_SX126X_DVK
#error "please define USE_SX126X_DVK"
#endif

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

void SX126xIoInit( void )
{
  GPIO_InitTypeDef initStruct={0};

//   Set DioIrqHandler port in IT_RISING mode
  initStruct.Mode = GPIO_MODE_IT_RISING;
  initStruct.Pull = GPIO_NOPULL;
  initStruct.Speed = GPIO_SPEED_HIGH;
  HW_GPIO_Init( RADIO_DIO_1_PORT, RADIO_DIO_1_PIN, &initStruct);

  initStruct.Mode = GPIO_MODE_INPUT;
  initStruct.Pull = GPIO_NOPULL;
  HW_GPIO_Init( DEVICE_SEL_PORT, DEVICE_SEL_PIN, &initStruct);
  HW_GPIO_Init( RADIO_BUSY_PORT, RADIO_BUSY_PIN, &initStruct);
  
  initStruct.Pull = GPIO_NOPULL;
  initStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HW_GPIO_Init( RADIO_NSS_PORT, RADIO_NSS_PIN,  &initStruct );
  HW_GPIO_Write( RADIO_NSS_PORT, RADIO_NSS_PIN,  1 );

  HW_GPIO_Init( RADIO_LEDTX_PORT, RADIO_LEDTX_PIN,  &initStruct );
  HW_GPIO_Write( RADIO_LEDTX_PORT, RADIO_LEDTX_PIN,  0 );
  
  HW_GPIO_Init( RADIO_LEDRX_PORT, RADIO_LEDRX_PIN,  &initStruct );
  HW_GPIO_Write( RADIO_LEDRX_PORT, RADIO_LEDRX_PIN,  0 );
  
  HW_GPIO_Init( RADIO_ANT_SWITCH_POWER_PORT, RADIO_ANT_SWITCH_POWER_PIN, &initStruct); 
  HW_GPIO_Write( RADIO_ANT_SWITCH_POWER_PORT, RADIO_ANT_SWITCH_POWER_PIN, 0);
}

void SX126xIoIrqInit( DioIrqHandler dioIrq )
{
  HW_GPIO_SetIrq( RADIO_DIO_1_PORT, RADIO_DIO_1_PIN, IRQ_HIGH_PRIORITY, dioIrq );
}

void SX126xIoDeInit( void )
{
}

uint32_t SX126xGetBoardTcxoWakeupTime( void )
{
    return BOARD_TCXO_WAKEUP_TIME;
}

void SX126xReset( void )
{
  GPIO_InitTypeDef initStruct = { 0 };

  initStruct.Mode =GPIO_MODE_OUTPUT_PP;
  initStruct.Pull = GPIO_NOPULL;
  initStruct.Speed = GPIO_SPEED_HIGH;

  // Wait 10 ms
  DelayMs( 10 );
  
  // Set RESET pin to 0
  HW_GPIO_Init( RADIO_RESET_PORT, RADIO_RESET_PIN, &initStruct );
  HW_GPIO_Write( RADIO_RESET_PORT, RADIO_RESET_PIN, 0 );
  
  // Wait 20 ms
  DelayMs( 20 );
  // Configure RESET as input
  initStruct.Mode = GPIO_MODE_ANALOG;
  HW_GPIO_Init( RADIO_RESET_PORT, RADIO_RESET_PIN, &initStruct );  
  HW_GPIO_Write( RADIO_RESET_PORT, RADIO_RESET_PIN, 0); // internal pull-up
  
  // Wait 10 ms
  DelayMs( 10 );
}

void SX126xWaitOnBusy( void )
{
    while( HW_GPIO_Read(RADIO_BUSY_PORT, RADIO_BUSY_PIN) == 1 );
}

void SX126xWakeup( void )
{
    //NSS = 0;
    HW_GPIO_Write( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );
    
    HW_SPI_InOut(RADIO_GET_STATUS);
    HW_SPI_InOut(0);
  
    //NSS = 1;
    HW_GPIO_Write( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );

    // Wait for chip to be ready.
    SX126xWaitOnBusy( );
}

void SX126xWriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    //NSS = 0;
    HW_GPIO_Write( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );

    HW_SPI_InOut(command);

    for( uint16_t i = 0; i < size; i++ )
    {
        HW_SPI_InOut( buffer[i] );
    }
    //NSS = 1;
    HW_GPIO_Write( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );

    if( command != RADIO_SET_SLEEP )
    {
        SX126xWaitOnBusy( );
    }
}

void SX126xReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    //NSS = 0;
    HW_GPIO_Write( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );

    HW_SPI_InOut(command);
    HW_SPI_InOut(0x00 );
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = HW_SPI_InOut( 0 );
    }

    //NSS = 1;
    HW_GPIO_Write( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );

    SX126xWaitOnBusy( );
}

void SX126xWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    //NSS = 0;
    HW_GPIO_Write( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );
    
    HW_SPI_InOut(RADIO_WRITE_REGISTER );
    HW_SPI_InOut(( address & 0xFF00 ) >> 8 );
    HW_SPI_InOut(address & 0x00FF );
    
    for( uint16_t i = 0; i < size; i++ )
    {
        HW_SPI_InOut(buffer[i] );
    }

    //NSS = 1;
    HW_GPIO_Write( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );

    SX126xWaitOnBusy( );
}

void SX126xWriteRegister( uint16_t address, uint8_t value )
{
    SX126xWriteRegisters( address, &value, 1 );
}

void SX126xReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    //NSS = 0;
    HW_GPIO_Write( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );

    HW_SPI_InOut(RADIO_READ_REGISTER);
    HW_SPI_InOut(( address & 0xFF00) >> 8 );
    HW_SPI_InOut(address & 0x00FF);
    HW_SPI_InOut(0);
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = HW_SPI_InOut( 0 );
    }
    //NSS = 1;
    HW_GPIO_Write( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );

    SX126xWaitOnBusy( );
}

uint8_t SX126xReadRegister( uint16_t address )
{
    uint8_t data;
    SX126xReadRegisters( address, &data, 1 );
    return data;
}

void SX126xWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    SX126xCheckDeviceReady( );

    //NSS = 0;
    HW_GPIO_Write( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );

    HW_SPI_InOut( RADIO_WRITE_BUFFER );
    HW_SPI_InOut( offset );
    for( uint16_t i = 0; i < size; i++ )
    {
        HW_SPI_InOut( buffer[i] );
    }
    //NSS = 1;
    HW_GPIO_Write( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );

    SX126xWaitOnBusy( );
}


void SX126xReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    SX126xCheckDeviceReady( );

    //NSS = 0;
    HW_GPIO_Write( RADIO_NSS_PORT, RADIO_NSS_PIN, 0 );

    HW_SPI_InOut( RADIO_READ_BUFFER );
    HW_SPI_InOut( offset );
    HW_SPI_InOut( 0 );
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = HW_SPI_InOut( 0 );
    }
    
    //NSS = 1;
    HW_GPIO_Write( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );

    SX126xWaitOnBusy( );
}
void SX126xSetRfTxPower( int8_t power )
{
    SX126xSetTxParams( power, RADIO_RAMP_40_US );
}



uint8_t SX126xGetPaSelect( uint32_t channel )
{
    if( HW_GPIO_Read( DEVICE_SEL_PORT, DEVICE_SEL_PIN) ==  1 )
    {
        return SX1261;
    }
    else
    {
        return SX1262;
    }
}

void SX126xAntSwOn( void )
{
  HW_GPIO_Write( RADIO_ANT_SWITCH_POWER_PORT, RADIO_ANT_SWITCH_POWER_PIN, 1);
}

void SX126xAntSwOff( void )
{
  HW_GPIO_Write( RADIO_ANT_SWITCH_POWER_PORT, RADIO_ANT_SWITCH_POWER_PIN, 0);
}



bool SX126xCheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}

bool SX126xBoardIsTcxoPresent(void)
{ /*no TCXO on board*/
  return true;
}

void SX126xBoardSetLedTx( bool state )
{
  if (state==false)
  {
    HW_GPIO_Write( RADIO_LEDTX_PORT, RADIO_LEDTX_PIN,  0 );
  }
  else
  {
    HW_GPIO_Write( RADIO_LEDTX_PORT, RADIO_LEDTX_PIN, 1 );
  }
}

void SX126xBoardSetLedRx( bool state )
{
  if (state==false)
  {
    HW_GPIO_Write( RADIO_LEDRX_PORT, RADIO_LEDRX_PIN, 0 );
  }
  else
  {
    HW_GPIO_Write( RADIO_LEDRX_PORT, RADIO_LEDRX_PIN, 1 );
  }
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

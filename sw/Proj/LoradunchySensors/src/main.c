/******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    10-July-2018
  * @brief   this is the main!
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "low_power_manager.h"
#include "lora.h"
//#include "bsp.h"
#include "timeServer.h"
#include "vcom.h"
#include "version.h"
#include "loradunchy_sensors.h"
#include "HPM.h"
#include "LoRaMacFCntHandler.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            10000
/*!
 * LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE LORAWAN_ADR_OFF
/*!
 * LoRaWAN Default data Rate Data Rate
 * @note Please note that LORAWAN_DEFAULT_DATA_RATE is used only when ADR is disabled 
 */
#define LORAWAN_DEFAULT_DATA_RATE DR_5
/*!
 * LoRaWAN application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_APP_PORT                            2
/*!
 * LoRaWAN default endNode class
 */
#define LORAWAN_DEFAULT_CLASS                       CLASS_B
/*!
 * LoRaWAN default confirm state
 */
#define LORAWAN_DEFAULT_CONFIRM_MSG_STATE           LORAWAN_UNCONFIRMED_MSG
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFF_SIZE                           64
/*!
 * User application data
 */
static uint8_t AppDataBuff[LORAWAN_APP_DATA_BUFF_SIZE];

/*!
 * User application data structure
 */
//static lora_AppData_t AppData={ AppDataBuff,  0 ,0 };
lora_AppData_t AppData={ AppDataBuff,  0 ,0 };

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* call back when LoRa endNode has received a frame*/
static void LORA_RxData( lora_AppData_t *AppData);

/* call back when LoRa endNode has just joined*/
static void LORA_HasJoined( void );

/* call back when LoRa endNode has just switch the class*/
static void LORA_ConfirmClass ( DeviceClass_t Class );

/* call back when server needs endNode to send a frame*/
static void LORA_TxNeeded ( void );

/* LoRa endNode send request*/
static void Send( void );

/* start the tx process*/
static void LoraStartTx(TxEventType_t EventType);

/* tx timer callback function*/
static void OnTxTimerEvent( void );

/* Private variables ---------------------------------------------------------*/
/* load Main call backs structure*/
static LoRaMainCallback_t LoRaMainCallbacks = { HW_GetBatteryLevel,
                                                HW_GetTemperatureLevel,
                                                HW_GetUniqueId,
                                                HW_GetRandomSeed,
                                                LORA_RxData,
                                                LORA_HasJoined,
                                                LORA_ConfirmClass,
                                                LORA_TxNeeded};

                                               
static TimerEvent_t TxTimer;


/* !
 *Initialises the Lora Parameters
 */
static  LoRaParam_t LoRaParamInit= {LORAWAN_ADR_STATE,
                                    LORAWAN_DEFAULT_DATA_RATE,  
                                    LORAWAN_PUBLIC_NETWORK};

                                    
/* ! 
    UART traffic process 
*/
extern void HPM_Process(void);


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main( void )
{
  /* HPM sensor readout in main thread */
  HPM_Ack_t hpmack;
	static uint16_t pm10, pm2_5;


  /* STM32 HAL library initialization*/
  HAL_Init();
  
  /* Configure the system clock*/
  SystemClock_Config();
  
  /* Configure the debug mode*/
  DBG_Init();
  
  /* Configure the hardware*/
  HW_Init();
  
  /* USER CODE BEGIN 1 */
  ENABLE_IRQ();
	/* USER CODE END 1 */
  
  /*Disbale Stand-by mode*/
  LPM_SetOffMode(LPM_APPLI_Id , LPM_Disable );
  
  PRINTF("VERSION: %X\n\r", VERSION);
  
  /* Configure the Lora Stack*/
  LORA_Init( &LoRaMainCallbacks, &LoRaParamInit);
  
  LORA_Join();
  
  LoraStartTx( TX_ON_TIMER) ;
  
  while( 1 )
  {
//    /* Handle UART traffic */
//    HPM_Process();

    /* Handle LoRa process */
    LoRaMacProcess( );
		//DelayMs( 2 );
    DISABLE_IRQ( );
    /* if an interrupt has occurred after DISABLE_IRQ, it is kept pending 
     * and cortex will not enter low power anyway  */

#ifndef LOW_POWER_DISABLE
    LPM_EnterLowPower( );
#endif

    ENABLE_IRQ();
		DelayMs( 2 );
		
		HPM_get();
		hpmack = HPM_GetAck();
		if (hpmack == HPM_ACK)
		{
			if (pm10==0) pm10 = 10;
			if (pm2_5==0) pm2_5 = 10;
			pm10 = HPM_LastReadout(HPM_READOUT_PM10);
			pm2_5 = HPM_LastReadout(HPM_READOUT_PM2_5);
			Loradunchy_sensor_SetPM(pm2_5, pm10);
		}
		else
		{
			HPM_Reset();
		}

    //Loradunchy_ReadHPMsensor();
		
  }
}

//extern LoRaMacFCntHandlerStatus_t LoRaMacSetFCntUp( uint32_t currentUp );


static void LORA_HasJoined( void )
{
	// Uncomment for testing frame counter rollover
	 // LoRaMacSetFCntUp(0x0000fffa);

#if( OVER_THE_AIR_ACTIVATION != 0 )
  PRINTF("JOINED\n\r");
#endif
  LORA_RequestClass( LORAWAN_DEFAULT_CLASS );
}

extern uint32_t GetUplinkCounter(void);
//extern uint16_t si7013_test(void);



static void Send( void )
{
  
  //uint8_t buf[4];
  sen_readout_t readouts;
	uint32_t uplinkcounter;
	
//  int32_t temperature=0, humidity=0; 
//  
// uint16_t pm2_5=0;		// PM 2.5 
//	uint16_t pm10=0;		// PM 10
//	
//	int16_t tmp75_T=0;	// Temperature from TMP75 in 1/100 deg. C

//	int16_t si7013_T=0; // Temperature from SI7013 in 1/100 deg. C
//	uint8_t si7013_RH=0;// Relative humidity from SI7013 in %RH

  
  if ( LORA_JoinStatus () != LORA_SET)
  {
    /*Not joined, try again later*/
    LORA_Join();
    return;
  }
  
  TVL1(PRINTF("SEND REQUEST\n\r");)
  //I2C_Loradunchy_ReadData(0x80,0xE3, buf, 2);
  //si7013_test();
  
//  si7013_measure_intemperature(&I2C_Loradunchy_Handle, 0x40, &temperature); 
//  si7013_T = temperature / 10;
//  si7013_measure_humidity(&I2C_Loradunchy_Handle, 0x40, &humidity);
//  si7013_RH = humidity/1000;
  
  Loradunchy_Read_sensors(&readouts);
  
  uint32_t i = 0;

  AppData.Port = LORAWAN_APP_PORT;
//  AppData.Buff[i++] = 0x10;
//  AppData.Buff[i++] = 0x21;
//  AppData.Buff[i++] = 0x32;
//  AppData.Buff[i++] = 0x43;
//  AppData.Buff[i++] = 0x54;
//  AppData.Buff[i++] = 0x65;
  AppData.Buff[i++] = readouts.pm2_5 & 0xff;
  AppData.Buff[i++] = (readouts.pm2_5>>8) & 0xff;
  AppData.Buff[i++] = readouts.pm10 & 0xff;
  AppData.Buff[i++] = (readouts.pm10>>8) & 0xff;
  AppData.Buff[i++] = readouts.si7013_T & 0xff;
  AppData.Buff[i++] = (readouts.si7013_T>>8) & 0xff;
  AppData.Buff[i++] = readouts.si7013_RH;
  
    uplinkcounter = GetUplinkCounter();
    if (uplinkcounter >= 0x0000ffff) {
       NVIC_SystemReset();   // Reset everything
    }
	AppData.Buff[i++]= uplinkcounter & 0xff;
  
  AppData.BuffSize = i;
  
  LORA_send( &AppData, LORAWAN_DEFAULT_CONFIRM_MSG_STATE);
	
 
	
}


static void LORA_RxData( lora_AppData_t *AppData )
{
  /* USER CODE BEGIN 4 */
  PRINTF("PACKET RECEIVED ON PORT %d\n\r", AppData->Port);

  switch (AppData->Port)
  {
    case 3:
    /*this port switches the class*/
    if( AppData->BuffSize == 1 )
    {
      switch (  AppData->Buff[0] )
      {
        case 0:
        {
          LORA_RequestClass(CLASS_A);
          break;
        }
        case 1:
        {
          LORA_RequestClass(CLASS_B);
          break;
        }
        case 2:
        {
          LORA_RequestClass(CLASS_C);
          break;
        }
        default:
          break;
      }
    }
    break;
    case LORAWAN_APP_PORT:
    if( AppData->BuffSize == 1 )
    {
//      AppLedStateOn = AppData->Buff[0] & 0x01;
//      if ( AppLedStateOn == RESET )
//      {
//        PRINTF("LED OFF\n\r");
//        LED_Off( LED_BLUE ) ; 
//      }
//      else
//      {
//        PRINTF("LED ON\n\r");
//        LED_On( LED_BLUE ) ; 
//      }
    }
    break;
		
  default:
    break;
  }
  /* USER CODE END 4 */
}

static void OnTxTimerEvent( void )
{
  /*Wait for next tx slot*/
  TimerStart( &TxTimer);
  /*Send*/
  Send( );
}

static void LoraStartTx(TxEventType_t EventType)
{
  if (EventType == TX_ON_TIMER)
  {
    /* send everytime timer elapses */
    TimerInit( &TxTimer, OnTxTimerEvent );
    TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE); 
    OnTxTimerEvent();
  }
  else
  {
    /* send everytime button is pushed */
    GPIO_InitTypeDef initStruct={0};
  
    initStruct.Mode =GPIO_MODE_IT_RISING;
    initStruct.Pull = GPIO_PULLUP;
    initStruct.Speed = GPIO_SPEED_HIGH;

    HW_GPIO_Init( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, &initStruct );
    HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 0, Send );
  }
}

static void LORA_ConfirmClass ( DeviceClass_t Class )
{
  PRINTF("switch to class %c done\n\r","ABC"[Class] );

  /*Optionnal*/
  /*informs the server that switch has occurred ASAP*/
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;
  
  LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
}

static void LORA_TxNeeded ( void )
{
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;
  
  LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

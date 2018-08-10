/*******************************************************************************
 * @file    at.c
 * @author  MCD Application Team
 * @version V1.2.0
 * @date    10-July-2018
 * @brief   at command API
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
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "at.h"
#include "utilities.h"
#include "lora.h"
#include "LoRaMacTest.h"
#include "radio.h"
#include "vcom.h"
#include "tiny_sscanf.h"
#include "version.h"
#include "hw_msp.h"
#include "test_rf.h"

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/**
 * @brief Max size of the data that can be received
 */
#define MAX_RECEIVED_DATA 255

/* Private macro -------------------------------------------------------------*/
/**
 * @brief Macro to return when an error occurs
 */
#define CHECK_STATUS(status) do {                    \
    ATEerror_t at_status = translate_status(status); \
    if (at_status != AT_OK) { return at_status; }    \
  } while (0)

/* Private variables ---------------------------------------------------------*/
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
static lora_AppData_t AppData={ AppDataBuff,  0 ,0 };
/**
 * @brief Buffer that contains the last received data
 */
static char ReceivedData[MAX_RECEIVED_DATA];

/**
 * @brief Size if the buffer that contains the last received data
 */
static unsigned ReceivedDataSize = 0;

/**
 * @brief Application port the last received data were on
 */
static uint8_t ReceivedDataPort;

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief  Translate a LoRaMacStatus_t into an ATEerror_t
 * @param  the LoRaMacStatus_t status
 * @retval the corresponding ATEerror_t code
 */
static ATEerror_t translate_status(LoRaMacStatus_t status);

/**
 * @brief  Get 16 bytes values in hexa
 * @param  The string containing the 16 bytes, something like ab:cd:01:...
 * @param  The buffer that will contain the bytes read
 * @retval The number of bytes read
 */
static int sscanf_16_hhx(const char *from, uint8_t *pt);

/**
 * @brief  Print 16 bytes as %02x
 * @param  the pointer containing the 16 bytes to print
 * @retval None
 */
static void print_16_02x(uint8_t *pt);

/**
 * @brief  Get 4 bytes values in hexa
 * @param  The string containing the 16 bytes, something like ab:cd:01:21
 * @param  The buffer that will contain the bytes read
 * @retval The number of bytes read
 */
static int sscanf_uint32_as_hhx(const char *from, uint32_t *value);

/**
 * @brief  Print 4 bytes as %02x
 * @param  the value containing the 4 bytes to print
 * @retval None
 */
static void print_uint32_as_02x(uint32_t value);

/**
 * @brief  Print 8 bytes as %02x
 * @param  the pointer containing the 8 bytes to print
 * @retval None
 */
static void print_8_02x(uint8_t *pt);

/**
 * @brief  Print an int
 * @param  the value to print
 * @retval None
 */
static void print_d(int value);

/**
 * @brief  Print an unsigned int
 * @param  the value to print
 * @retval None
 */
static void print_u(unsigned int value);

/* Exported functions ------------------------------------------------------- */

void set_at_receive(uint8_t AppPort, uint8_t* Buff, uint8_t BuffSize)
{
  if (MAX_RECEIVED_DATA <= BuffSize)
    BuffSize = MAX_RECEIVED_DATA;
  memcpy1((uint8_t *)ReceivedData, Buff, BuffSize);
  ReceivedDataSize = BuffSize;
  ReceivedDataPort = AppPort;
}

ATEerror_t at_return_ok(const char *param)
{
  return AT_OK;
}

ATEerror_t at_return_error(const char *param)
{
  return AT_ERROR;
}

ATEerror_t at_reset(const char *param)
{
  NVIC_SystemReset();
  return AT_OK;
}

ATEerror_t at_DevEUI_get(const char *param)
{
  print_8_02x(lora_config_deveui_get());
  return AT_OK;
}

ATEerror_t at_JoinEUI_get(const char *param)
{
  print_8_02x(lora_config_joineui_get());
  return AT_OK;
}

ATEerror_t at_JoinEUI_set(const char *param)
{
  uint8_t JoinEui[8];
  if (tiny_sscanf(param, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                  &JoinEui[0], &JoinEui[1], &JoinEui[2], &JoinEui[3],
                  &JoinEui[4], &JoinEui[5], &JoinEui[6], &JoinEui[7]) != 8)
  {
    return AT_PARAM_ERROR;
  }
  
  lora_config_joineui_set(JoinEui);
  return AT_OK;
}

ATEerror_t at_DevAddr_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_DEV_ADDR;
  if (sscanf_uint32_as_hhx(param, &mib.Param.DevAddr) != 4)
  {
    return AT_PARAM_ERROR;
  }
  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);
  return AT_OK;
}

ATEerror_t at_DevAddr_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_DEV_ADDR;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_uint32_as_02x(mib.Param.DevAddr);
  return AT_OK;
}

ATEerror_t at_AppKey_get(const char *param)
{
  print_16_02x(lora_config_appkey_get());
  return AT_OK;
}

ATEerror_t at_AppKey_set(const char *param)
{
  uint8_t AppKey[16];
  if (sscanf_16_hhx(param, AppKey) != 16)
  {
    return AT_PARAM_ERROR;
  }
  
  lora_config_appkey_set(AppKey);
  return AT_OK;
}

ATEerror_t at_NwkSKey_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_NWK_S_ENC_KEY;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_16_02x(mib.Param.NwkSEncKey);

  return AT_OK;
}

ATEerror_t at_NwkSKey_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;
  uint8_t NwkSKey[16];

  mib.Type = MIB_NWK_S_ENC_KEY;

  if (sscanf_16_hhx(param, NwkSKey) != 16)
  {
    return AT_PARAM_ERROR;
  }

  mib.Param.NwkSEncKey = NwkSKey;
  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

  return AT_OK;
}

ATEerror_t at_AppSKey_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_APP_S_KEY;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_16_02x(mib.Param.AppSKey);

  return AT_OK;
}

ATEerror_t at_AppSKey_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;
  uint8_t AppSKey[16];

  mib.Type = MIB_APP_S_KEY;
  if (sscanf_16_hhx(param, AppSKey) != 16)
  {
    return AT_PARAM_ERROR;
  }
  mib.Param.AppSKey = AppSKey;
  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

  return AT_OK;
}

ATEerror_t at_Certif( const char *param )
{
  lora_wan_certif( );
  return AT_OK;
}

ATEerror_t at_ADR_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_ADR;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_d(mib.Param.AdrEnable);

  return AT_OK;
}

ATEerror_t at_ADR_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_ADR;

  switch (param[0])
  {
    case '0':
    case '1':
      mib.Param.AdrEnable = param[0] - '0';
      status = LoRaMacMibSetRequestConfirm(&mib);
      CHECK_STATUS(status);
      break;
    default:
      return AT_PARAM_ERROR;
  }

  return AT_OK;
}

ATEerror_t at_TransmitPower_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_CHANNELS_TX_POWER;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_d(mib.Param.ChannelsTxPower);

  return AT_OK;
}

ATEerror_t at_TransmitPower_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_CHANNELS_TX_POWER;
  if (tiny_sscanf(param, "%hhu", &mib.Param.ChannelsTxPower) != 1)
  {
    return AT_PARAM_ERROR;
  }
  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

  return AT_OK;
}

ATEerror_t at_DataRate_get(const char *param)
{

  print_d(lora_config_tx_datarate_get() );

  return AT_OK;
}

ATEerror_t at_DataRate_set(const char *param)
{
  int8_t datarate;

  if (tiny_sscanf(param, "%hhu", &datarate) != 1)
  {
    return AT_PARAM_ERROR;
  }
  
  lora_config_tx_datarate_set(datarate) ;

  return AT_OK;
}

ATEerror_t at_DutyCycle_set(const char *param)
{
  switch (param[0])
  {
    case '0':
      lora_config_duty_cycle_set(LORA_DISABLE);
      break;
    case '1':
      lora_config_duty_cycle_set(LORA_ENABLE);
      break;
    default:
      return AT_PARAM_ERROR;
  }

  return AT_OK;
}

ATEerror_t at_DutyCycle_get(const char *param)
{
  if (lora_config_duty_cycle_get() == LORA_ENABLE)
    AT_PRINTF("1\r\n");
  else
    AT_PRINTF("0\r\n");

  return AT_OK;
}


ATEerror_t at_PublicNetwork_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_PUBLIC_NETWORK;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_d(mib.Param.EnablePublicNetwork);

  return AT_OK;
}

ATEerror_t at_PublicNetwork_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_PUBLIC_NETWORK;
  switch (param[0])
  {
    case '0':
    case '1':
      mib.Param.EnablePublicNetwork = param[0] - '0';
      status = LoRaMacMibSetRequestConfirm(&mib);
      CHECK_STATUS(status);
      break;
    default:
      return AT_PARAM_ERROR;
  }

  return AT_OK;
}

ATEerror_t at_Rx2Frequency_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_RX2_CHANNEL;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_d(mib.Param.Rx2Channel.Frequency);

  return AT_OK;
}

ATEerror_t at_Rx2Frequency_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_RX2_CHANNEL;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);

  if (tiny_sscanf(param, "%lu", &mib.Param.Rx2Channel.Frequency) != 1)
  {
    return AT_PARAM_ERROR;
  }

  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

  return AT_OK;
}

ATEerror_t at_Rx2DataRate_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_RX2_CHANNEL;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_d(mib.Param.Rx2Channel.Datarate);

  return AT_OK;
}

ATEerror_t at_Rx2DataRate_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_RX2_CHANNEL;

  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);

  if (tiny_sscanf(param, "%hhu", &mib.Param.Rx2Channel.Datarate) != 1)
  {
    return AT_PARAM_ERROR;
  }

  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

  return AT_OK;
}

ATEerror_t at_Rx1Delay_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_RECEIVE_DELAY_1;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_u(mib.Param.ReceiveDelay1);

  return AT_OK;
}

ATEerror_t at_Rx1Delay_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_RECEIVE_DELAY_1;
  if (tiny_sscanf(param, "%lu", &mib.Param.ReceiveDelay1) != 1)
  {
    return AT_PARAM_ERROR;
  }
  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

  return AT_OK;
}

ATEerror_t at_Rx2Delay_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_RECEIVE_DELAY_2;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_u(mib.Param.ReceiveDelay2);

  return AT_OK;
}

ATEerror_t at_Rx2Delay_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_RECEIVE_DELAY_2;
  if (tiny_sscanf(param, "%lu", &mib.Param.ReceiveDelay2) != 1)
  {
    return AT_PARAM_ERROR;
  }
  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

  return AT_OK;
}

ATEerror_t at_JoinAcceptDelay1_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_JOIN_ACCEPT_DELAY_1;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_u(mib.Param.JoinAcceptDelay1);

  return AT_OK;
}

ATEerror_t at_JoinAcceptDelay1_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_JOIN_ACCEPT_DELAY_1;
  if (tiny_sscanf(param, "%lu", &mib.Param.JoinAcceptDelay1) != 1)
  {
    return AT_PARAM_ERROR;
  }
  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

  return AT_OK;
}

ATEerror_t at_JoinAcceptDelay2_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_JOIN_ACCEPT_DELAY_2;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_u(mib.Param.JoinAcceptDelay2);

  return AT_OK;
}

ATEerror_t at_JoinAcceptDelay2_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_JOIN_ACCEPT_DELAY_2;
  if (tiny_sscanf(param, "%lu", &mib.Param.JoinAcceptDelay2) != 1)
  {
    return AT_PARAM_ERROR;
  }
  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

  return AT_OK;
}

ATEerror_t at_NetworkJoinMode_get(const char *param)
{
  print_d((lora_config_otaa_get() == LORA_ENABLE ? 1 : 0));
  return AT_OK;
}

ATEerror_t at_NetworkJoinMode_set(const char *param)
{
  LoraState_t status;

  switch (param[0])
  {
    case '0':
      status = LORA_DISABLE;
      break;
    case '1':
      status = LORA_ENABLE;
      break;
    default:
      return AT_PARAM_ERROR;
  }

  lora_config_otaa_set(status);
  return AT_OK;
}

ATEerror_t at_NetworkID_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_NET_ID;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_uint32_as_02x(mib.Param.NetID);

  return AT_OK;
}

ATEerror_t at_NetworkID_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_NET_ID;
  if (sscanf_uint32_as_hhx(param, &mib.Param.NetID) != 4)
  {
    return AT_PARAM_ERROR;
  }
  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

  return AT_OK;
}



ATEerror_t at_DeviceClass_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_DEVICE_CLASS;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  AT_PRINTF("%c\r\n", 'A' + mib.Param.Class);

  return AT_OK;
}

ATEerror_t at_DeviceClass_set(const char *param)
{
  switch (param[0])
  {
    case 'A':
      LORA_RequestClass(CLASS_A);
      break;
    case 'B':
      return AT_ERROR;
      /*LORA_RequestClass(CLASS_B);*/  /*Class B AT cmd switch Class B not supported cf.[UM2073]*/
      /*break;*/
    case 'C':
      LORA_RequestClass(CLASS_C);
      break;
    default:
      return AT_PARAM_ERROR;
  }
  return AT_OK;
}

ATEerror_t at_Join(const char *param)
{
  
  LoraErrorStatus status= LORA_Join();
  if (status == LORA_SUCCESS)
  {
    return AT_OK;
  }
  else
  {
    return AT_ERROR;
  }
}

ATEerror_t at_NetworkJoinStatus(const char *param)
{
  if( LORA_JoinStatus() == LORA_RESET )
  {
    print_d(0);
  }
  else
  {
    print_d(1);
  }
  
  return AT_OK;
}

ATEerror_t at_SendBinary(const char *param)
{
  LoraErrorStatus status;
  const char *buf= param;
  unsigned char bufSize= strlen(param);
  uint32_t appPort;
  unsigned size=0;
  char hex[3];
  
    /* read and set the application port */
  if (1 != tiny_sscanf(buf, "%u:", &appPort))
  {
    AT_PRINTF("AT+SEND without the application port");
    return AT_PARAM_ERROR;
  }
  
  /* skip the application port */
  while (('0' <= buf[0]) && (buf[0] <= '9'))
  {
    buf ++;
    bufSize --;
  };
  
  if (buf[0] != ':')
  {
    AT_PRINTF("AT+SEND without the application port");
    return AT_PARAM_ERROR;
  }
  else
  {
    /*ok skip the char ':' */
    buf ++;
    bufSize --;
  }

  hex[2] = 0;
  while ((size < LORAWAN_APP_DATA_BUFF_SIZE) && (bufSize > 1))
  {
    hex[0] = buf[size*2];
    hex[1] = buf[size*2+1];
    if (tiny_sscanf(hex, "%hhx", &AppData.Buff[size]) != 1)
    {
      return AT_PARAM_ERROR;
    }
    size++;
    bufSize -= 2;
  }
  if (bufSize != 0)
  {
    return AT_PARAM_ERROR;
  }
  
  AppData.BuffSize = size;
  AppData.Port= appPort;

  status = LORA_send( &AppData, lora_config_reqack_get() );
  
  if (status == LORA_SUCCESS)
  {
    return AT_OK;
  }
  else
  {
    return AT_ERROR;
  }
}

ATEerror_t at_Send(const char *param)
{
  LoraErrorStatus status;
  const char *buf= param;
  unsigned char bufSize= strlen(param);
  uint32_t appPort;
  
    /* read and set the application port */
  if (1 != tiny_sscanf(buf, "%u:", &appPort))
  {
    AT_PRINTF("AT+SEND without the application port");
    return AT_PARAM_ERROR;
  }
  
  /* skip the application port */
  while (('0' <= buf[0]) && (buf[0] <= '9'))
  {
    buf ++;
    bufSize --;
  };
  
  if (buf[0] != ':')
  {
    AT_PRINTF("AT+SEND without the application port");
    return AT_PARAM_ERROR;
  }
  else
  {
    /*ok skip the char ':' */
    buf ++;
    bufSize --;
  }
  
  if (bufSize > LORAWAN_APP_DATA_BUFF_SIZE)
  {
    bufSize = LORAWAN_APP_DATA_BUFF_SIZE;
  }
  memcpy1(AppData.Buff, (uint8_t *)buf, bufSize);
  AppData.BuffSize = bufSize;
  AppData.Port= appPort;
  
  status = LORA_send( &AppData, lora_config_reqack_get() );
  
  if (status == LORA_SUCCESS)
  {
    return AT_OK;
  }
  else
  {
    return AT_ERROR;
  }
}

ATEerror_t at_ReceiveBinary(const char *param)
{
  unsigned i;
  
  AT_PRINTF("%d:", ReceivedDataPort);
  for (i = 0; i < ReceivedDataSize; i++)
  {
    AT_PRINTF("%02x", ReceivedData[i]);
  }
  AT_PRINTF("\r\n");
  ReceivedDataSize = 0;

  return AT_OK;
}

ATEerror_t at_Receive(const char *param)
{
  AT_PRINTF("%d:", ReceivedDataPort);
  if (ReceivedDataSize)
  {
    AT_PRINTF("%s", ReceivedData);
    ReceivedDataSize = 0;
  }
  AT_PRINTF("\r\n");

  return AT_OK;
}

ATEerror_t at_version_get(const char *param)
{
  AT_PRINTF(AT_VERSION_STRING"\r\n");
  return AT_OK;
}

ATEerror_t at_ack_set(const char *param)
{
  switch (param[0])
  {
    case '0':
      lora_config_reqack_set(LORAWAN_UNCONFIRMED_MSG);
      break;
    case '1':
      lora_config_reqack_set(LORAWAN_CONFIRMED_MSG);
      break;
    default:
      return AT_PARAM_ERROR;
  }

  return AT_OK;
}

ATEerror_t at_ack_get(const char *param)
{
  print_d (((lora_config_reqack_get() == LORAWAN_CONFIRMED_MSG) ? 1 : 0));
  return AT_OK;
}

ATEerror_t at_isack_get(const char *param)
{
  print_d(((lora_config_isack_get() == LORA_ENABLE) ? 1 : 0));
  return AT_OK;
}

ATEerror_t at_snr_get(const char *param)
{
  print_u(lora_config_snr_get());
  return AT_OK;
}

ATEerror_t at_rssi_get(const char *param)
{
  print_d(lora_config_rssi_get());
  return AT_OK;
}

ATEerror_t at_bat_get(const char *param)
{
  print_u(HW_GetBatteryLevel());
  return AT_OK;
}

ATEerror_t at_test_txTone(const char *param)
{
  return TST_TxTone(param, strlen(param));
}

ATEerror_t at_test_rxTone(const char *param)
{
  return TST_RxTone(param, strlen(param));
}

ATEerror_t at_test_txlora(const char *param)
{
  TST_TX_LoraStart( param, strlen(param) );
  return AT_OK;
}

ATEerror_t at_test_rxlora(const char *param)
{
  TST_RX_LoraStart( );
  return AT_OK;
}

ATEerror_t at_test_get_lora_config(const char *param)
{
  return TST_get_lora_config( param, strlen(param));
}

ATEerror_t at_test_set_lora_config(const char *param)
{
  return TST_SET_lora_config( param, strlen(param));
}

ATEerror_t at_test_stop(const char *param)
{
  return TST_stop( );
}
/* Private functions ---------------------------------------------------------*/

static ATEerror_t translate_status(LoRaMacStatus_t status)
{
  if (status == LORAMAC_STATUS_BUSY)
  {
    return AT_BUSY_ERROR;
  }
  if (status == LORAMAC_STATUS_PARAMETER_INVALID)
  {
    return AT_PARAM_ERROR;
  }
  if (status == LORAMAC_STATUS_NO_NETWORK_JOINED)
  {
    return AT_NO_NET_JOINED;
  }
  if (status != LORAMAC_STATUS_OK)
  {
    return AT_ERROR;
  }
  return AT_OK;
}

static int sscanf_16_hhx(const char *from, uint8_t *pt)
{
  return tiny_sscanf(from, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                     &pt[0], &pt[1], &pt[2], &pt[3], &pt[4], &pt[5], &pt[6],
                     &pt[7], &pt[8], &pt[9], &pt[10], &pt[11], &pt[12], &pt[13],
                     &pt[14], &pt[15]);
}

static void print_16_02x(uint8_t *pt)
{
  AT_PRINTF("%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\r\n",
            pt[0], pt[1], pt[2], pt[3],
            pt[4], pt[5], pt[6], pt[7],
            pt[8], pt[9], pt[10], pt[11],
            pt[12], pt[13], pt[14], pt[15]);
}

static int sscanf_uint32_as_hhx(const char *from, uint32_t *value)
{
  return tiny_sscanf(from, "%hhx:%hhx:%hhx:%hhx",
                     &((unsigned char *)(value))[0],
                     &((unsigned char *)(value))[1],
                     &((unsigned char *)(value))[2],
                     &((unsigned char *)(value))[3]);
}

static void print_uint32_as_02x(uint32_t value)
{
  AT_PRINTF("%02x:%02x:%02x:%02x\r\n",
            (unsigned)((unsigned char *)(&value))[0],
            (unsigned)((unsigned char *)(&value))[1],
            (unsigned)((unsigned char *)(&value))[2],
            (unsigned)((unsigned char *)(&value))[3]);
}

static void print_8_02x(uint8_t *pt)
{
  AT_PRINTF("%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\r\n",
            pt[0], pt[1], pt[2], pt[3], pt[4], pt[5], pt[6], pt[7]);
}

static void print_d(int value)
{
  AT_PRINTF("%d\r\n", value);
}

static void print_u(unsigned int value)
{
  AT_PRINTF("%u\r\n", value);
}

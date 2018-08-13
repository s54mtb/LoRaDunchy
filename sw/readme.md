## How to use LoraDunchy software

After installation of free Keil IDE (uVision for STM32 L0/F0) clone the repository and copy content of the [sw/Proj/LoraDunchy](https://github.com/s54mtb/LoRaDunchy/tree/master/sw/Proj/LoraDunchy) folder and that's your starting point. 

Next, register on the TheThings Network and create application. Within your application [register new device](https://www.thethingsnetwork.org/docs/devices/registration.html). 

Copy [template comissioning file (sw/Proj/{your_project}/inc/](https://github.com/s54mtb/LoRaDunchy/blob/master/sw/Proj/LoraDunchy/inc/Commissioning_template.h) to new file. 

Use application key and device ids to fill the missing data in the missing fields in the comissioning file. 
Build project, upload to the LoraDunchy hardware. Activate Reset and if your device is within the TTN connected LoraWAN Gateway, you should observe packet with couple of bytes payload (0x01 0x02 0x03 0x04 0x05). 

## Other configuration options: 

inc/utilities_conf.h 		--- Change verbose level for trace via serial port

inc/hw_conf.h				--- Debug and low power disable switch

inc/mlm32l0xx_hw_conf.h		--- Define which USART is used for debug IO (USART1: PB7 --- Rx, PB6 --- Tx, USART2: PA2, PA3)


### Check defines in main.c: 

APP_TX_DUTYCYCLE			Defines the application data transmission duty cycle in ms

LORAWAN_ADR_STATE			LoRaWAN Adaptive Data Rate on/off

LORAWAN_DEFAULT_DATA_RATE	LoRaWAN Default Data Rate

LORAWAN_APP_PORT			LoRaWAN application port for sending data

LORAWAN_DEFAULT_CLASS		LoRaWAN default endNode class

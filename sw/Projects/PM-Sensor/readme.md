#LoRa multisensor board

Check hardware project HW/MultiSen

Connect board to I2C1: 
*	PB9     ------> I2C1_SDA
*	PB8     ------> I2C1_SCL 

* 	PM sensor to UART

and power supply 3,3V

Payload structure: 

uint16 pm2_5		// PM2.5 value
uint16 pm10			// PM10 value
int16 T				// 1/100 deg. C + 300
uint8 RH			// % rh

test: 
pm2_5 = 0x1234
pm10 = 0x5678
T = 0xAABB
RH = 0xcc

payload:34127856BBAACC00


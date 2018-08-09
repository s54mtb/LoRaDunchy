## How to use LoraDunchy software

After installation of free Keil IDE (uVision for STM32 L0/F0) clone the repository and copy content of the [sw/projects/LoraDunchyTemplate](https://github.com/s54mtb/LoRaDunchy/tree/master/sw/Projects/LoraDunchyTemplate) folder and that's your starting point. 

Next, register on the TheThings Network and create application. Within your application [register new device](https://www.thethingsnetwork.org/docs/devices/registration.html). 

Copy [template comissioning file (sw/Projects/{your_project}/inc/](https://github.com/s54mtb/LoRaDunchy/blob/master/sw/Projects/LoraDunchy/inc/Commissioning_template.h) to new file.  
Use application key and device ids to fill the missing data in the missing fields in the comissioning file. 
Edit the [include directive in file comissioning.h, line 68](https://github.com/s54mtb/LoRaDunchy/blob/6230a39a3e301befb14c109ed16b3d87734aaac0/sw/Projects/LoraDunchy/inc/Commissioning.h#L67). 
Build project, upload to the LoraDunchy hardware. Activate Reset and if your device is within the TTN connected LoraWAN Gateway, you should observe packet with couple of bytes payload (0x01 0x02 0x03 0x04 0x05). 

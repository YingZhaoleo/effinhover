# effinhover arduino code

This code runs on the onboard arduino ESP32 and echos commands to/from the betaflight board over WIFI.

## Set up Arduino IDE to program ESP32

- Download the latest Arduino IDE from the [official website](https://www.arduino.cc/en/Main/Software).
- Install the ESP32 platform package as described [here](https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md). This will allow you to select *Adafruit ESP32 Feather* in the dropdown menu *Tools > Board* of the Arduino IDE. 
- Finally you have to install the SiLabs CP2104 Driver from [this website](https://www.silabs.com/products/development-tools/software/usb-to-uart-bridge-vcp-drivers). On Mac OSX you might have to go to *System Preferences > Security & Privacy > General* and allow the driver on your system. Otherwise the serial communication with the arduino will not work. 

All these steps are also explained in the [Adafruit HUZZAH32 documentation](https://learn.adafruit.com/adafruit-huzzah32-esp32-feather).
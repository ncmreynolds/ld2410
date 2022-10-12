# LD2410
## Introduction

An Arduino library for the Hi-Link LD2410 24Ghz FMCW radar sensor. This sensor is a Frequency Modulated Continuous Wave radar, which makes it good for presence detection and its sensitivity at different ranges to both static and moving targets can be configured.

![](ld2410andbreakout.jpg)

The code in this library is based heavily off this [piece of work for ESPHome](https://github.com/rain931215/ESPHome-LD2410) and the manufacturer datasheet.

The LD2410, as sold for configuration with the common breakout boards shown above communicates over serial at 256000 baud by default. This library allows you to configure and use the sensor over this serial connection. As the LD2410 is a device that uses a high baud rate, a microcontroller with a spare hardware UART for this communication is preferable to bit-banged software serial.

The modules also provide an 'active high' output on the 'OUT' pin which can be used to signal presence, based off the settings configured. Once the modules are configured it is not necessary to use their UART, but it provides much more information.

## Connections

The module must be powered by 5V or higher, but does 3.3v I/O, please consider this when working with the module.

![](ld2410pinout.jpg)

The default header pitch is 1.27mm, half the size of the usual 0.1" hobbyist pin headers so you may need to buy some, or carefully de-solder the existing pins to use the module.

![](ld2410pcb.jpg)

If you have the breakout board, you can use the VCC, GND, TX and RX pins to work with the module.






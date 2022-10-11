# LD2410
## Introduction

An Arduino library for the Hi-Link LD2410 24Ghz FMCW radar sensor.

![](ld2410andbreakout.jpg)

The code in this library is based heavily off this [piece of work for ESPHome](https://github.com/rain931215/ESPHome-LD2410) and the datasheet.

The LD2410, as sold for configuration with the common breakout boards shown above communicates over serial at 256000 baud by default. This library allows you to configure and use the sensor over this serial connection. As the LD2410 is a device that uses a high baud rate, a microcontroller with a spare hardware UART for this communication is preferable.




# SX1276Generic Driver
/*
* (c) 2017 Helmut Tschemernjak (Helmut64 on mbed).
* 30826 Garbsen (Hannover) Germany
*/

This library represents a common SX1276 module driver supporting SX1276
based modules. The approach is to support multiple OS versions including
mbed, Arduino and Linux using the same driver code and little
adjustments for the different OS version. The SX1276 driver is based on
the Semtech 1276 code which can be seen in the revisions of this library
repository.

## Supported LoRa Modules

The following Lora modules are supported:
- HopeRF RFM95
- Murata MURATA_SX1276 (CMWX1ZZABZ-078, used the STM B_L072Z_LRWAN1 board)
- SX1276MB1MAS (433, 868 MHz version)
- SX1276MB1LAS (433, 915 MHz version)
- SX1276 Heltec (868 MHz version)
- SX1278 Heltec (433 MHz version)


## Getting Started for Developers
Import the mbed sample project:
http://developer.mbed.org/users/Helmut64/code/STM32L0_LoRa
- It includes a PingPong sample code
- It includes a PinMap.h which allows to define the LoRa SPI,
DIO interrupt, reset and antenna pins.
The STM32L0_LoRa is a turnkey sample application for the STM B_L072Z_LRWAN1,
however it will work with all other mbed based boards by adjusting the PinMap.h

## Developers help needed
A list of tasks is documented in the file: LoRa_TODO.txt
I (Helmut Tschemernjak) spend a very significant time to complete the
initial version of the SX1276Generic packet driver. Enhancements,
further module support and tuning is more than welcome. Please send me
your patches via mbed. Also questions can be submitted in the mbed
“Questions” area or a personal message via mbed.

## Future developments
I work in a advanced private protocol using basic LoRa modules to
communicate between simple nodes (battery powered) and stations
(permanent power). The station should support thousands of nodes running
on an Linux based OS using this 1276Generic driver or the LoRa
concentrator module. The station should also work on mbed or Arduino
assuming sufficient memory is provided. I believe there is an
opportunity to do a better protocol compared to the official LoRa
protocol which requires an Concentrator, a LoRa server and an
application server. The idea is to over only efficient, reliable and
secure communication between the nodes and the stations. Further
forwarding to MQTT and other network services can be handled separately
on the station.



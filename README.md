# AIROC&#8482; BTSDK - CYW43022

## Overview

The Infineon CYW43022 is a 28nm ultra-low power dual-mode Bluetooth&#174; 5.2 wireless MCU Wi-Fi + Bluetooth&#174; Combo device. It has a stand-alone baseband processor with an integrated 2.4 GHz transceiver supporting BR, EDR, and Bluetooth&#174; LE.

## SDK Software Features
- Dual mode Bluetooth&#174; stack included in the ROM (BR/EDR and LE).
- Bluetooth&#174; stack and profile level APIs for embedded Bluetooth&#174; application development.
- AIROC&#8482; HCI protocol to simplify host/MCU application development.
- APIs and drivers to access on board peripherals.
- Bluetooth&#174; protocols include GAP, GATT, SMP, HID.
- LE profile APIs, libraries, and sample apps.
- Support for Over-The-Air (OTA) upgrade.
- Device Configurator for creating custom pin mapping.
- Bluetooth&#174; Configurator for creating LE GATT Database.
- Documentation for APIs, datasheet, profiles, and features.

## Kits Supported
#### CYW943022BTEVK-01
- 106 ball WLBGA package, Audio Arduino shield with on-board microphones,
  audio codec chip, headphone output, expansion header (J5) for direct access
  to BT_GPIO, i2S, UART, user switches and LEDs, and a USB connector for
  power, programming, and USB-UART bridge.
  Max UART baud rate is 3M

## Downloading application to the kit
This kit does not have SFLASH, so image download and write to FLASH is not possible
as with other AIROC&#8482; boards.  Instead, the DIRECT_LOAD feature is the default/only
option for downloading new application FW.  The application 'make build' target
will produce a .hcd image file, and the application 'make program' target can be used to download
to RAM on the device and instruct the device to reboot directly from RAM instead of the read-only
ROM boot image.

If you have issues downloading to the kit, power cycle the kit to boot from the ROM
image.

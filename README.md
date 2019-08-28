# Azure Sphere WiFi Control using a Bluetooth Low Energy (BLE) Link

## Overview

This repository contains a modified version of the Sample code found in the [Azure Sphere Samples repository](https://github.com/Azure/azure-sphere-samples/tree/master/Samples/WifiSetupAndDeviceControlViaBle).  The code in this repository has been ported to the [Avnet Azure Sphere MT3620 Starter Kit]([http://cloudconnectkits.org/product/azure-sphere-starter-kit](http://cloudconnectkits.org/product/azure-sphere-starter-kit)). Although the Avnet Starter Kit contains the same MT3620 processor as the Seed Development Board, the hardware platform itself is different. A summary of these differences are:

1. Different number and configuration of LEDs

2. The UART interface in this repository has been minimized to run with a minimum hardware interface from the mikroBUS sockets.

## **Using the Sample**

See the [Azure Sphere Getting Started](https://www.microsoft.com/en-us/azure-sphere/get-started/) page for details on setting up your PC for development. You should also complete the Azure Sphere [Installation](https://docs.microsoft.com/azure-sphere/install/overview) and [Quickstarts](https://docs.microsoft.com/azure-sphere/quickstarts/qs-overview) to validate that your environment is configured properly.

After the environment is established, clone this repository locally:

    git clone https://github.com/Avnet/WifiBLE

This solution utilizes a Nordic nRF52 Development Kit similar to the original Microsoft Sample along with a Windows 10 App to illustrate the companion user experience.

Overall, the sample code operation is the same as described in the [Microsoft Sample](https://github.com/Azure/azure-sphere-samples/tree/master/Samples/WifiSetupAndDeviceControlViaBle) but the LED operation is different.  For the Avnet Starter Kit it there is a single tri-colored LED which flashes at a half second interval between Status and User LED values:

| LED | Description |
|------|------------|
|Yellow| Uninitialized |
|Blue| Advertising to bonded devices only |
|Red | Advertising to all devices |
|Green | Connected to a central device |
|Magenta | Error|
|White |flashes WHITE when the User LED is on|

## Hardware setup

Jumpers run from the Avnet Azure Sphere MT3620 Starter Kit to the nRF52 board using a using a MikroElectronika click [PROTO click]([https://www.mikroe.com/proto-click](https://www.mikroe.com/proto-click)).  The Click Board pin used are :
* GPIO2_PWM2
* GPI28_MISC0_RX0_DATA0
* GPIO26_SCLK0_TX0
* GND

See: [Schematic](http://cloudconnectkits.org/sites/default/files/AES-MS-MT3620-SK-G_SCH_2019-03-06.PDF)

|NRF52 Board | Mikroe Proto click |
|------------|--------------------|
|P0.21 (RESET) | GPIO2_PWM2 |
| P0.12 (TX) | GPIO26_SCLK0_TX0 |
| P0.11 (RX) | GPIO28_MISC0_RX0_DATA0|
| GND | GND|

NOTE: Not all the connections present in the Microsoft Sample code are needed. 

With the exception of the above, operation is the same as described in the READ.me located [here](https://github.com/Azure/azure-sphere-samples/tree/master/Samples/WifiSetupAndDeviceControlViaBle)).



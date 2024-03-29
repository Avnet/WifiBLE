/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

// This file defines the mapping from the Seeed MT3620 Mini Development Board (MDB) to the
// 'sample hardware' abstraction used by the samples at https://github.com/Azure/azure-sphere-samples.
// Some peripherals are on-board on the Seeed MT3620 MDB, while other peripherals must be attached externally if needed.

// This file is autogenerated from ..\..\sample_hardware.json.  Do not edit it directly.

#pragma once
#include "seeed_mt3620_mdb.h"

// MT3620 MDB: Connect external button using J1, pin 9.
#define SAMPLE_BUTTON_1 SEEED_MT3620_MDB_J1_PIN9_GPIO30

// MT3620 MDB: Connect external button using J2, pin 15.
#define SAMPLE_BUTTON_2 SEEED_MT3620_MDB_J2_PIN15_GPIO35

// MT3620 MDB: User LED.
#define SAMPLE_LED SEEED_MT3620_MDB_USER_LED

// MT3620 MDB: Connect external LED using J1, pin 1.
#define SAMPLE_RGBLED_RED SEEED_MT3620_MDB_J1_PIN1_GPIO4

// MT3620 MDB: Connect external LED using J1, pin 2.
#define SAMPLE_RGBLED_GREEN SEEED_MT3620_MDB_J1_PIN2_GPIO5

// MT3620 MDB: Connect external LED using J1, pin 3.
#define SAMPLE_RGBLED_BLUE SEEED_MT3620_MDB_J1_PIN3_GPIO6

// MT3620 MDB: Connect J1 pin 7 (RX) to J1 pin 5 (TX).
#define SAMPLE_UART SEEED_MT3620_MDB_J1_ISU0_UART

// MT3620 MDB: Connect external LSM6DS3 to I2C using J1 and J2, pin 15 (SDA) and pin 10 (SCL).
#define SAMPLE_LSM6DS3_I2C SEEED_MT3620_MDB_J1J2_ISU1_I2C

// MT3620 MDB: Connect external LSM6DS3 to SPI using J1, pin 7 (MISO), pin 5 (SCLK), pin 8 (CSA), pin 6 (MOSI).
#define SAMPLE_LSM6DS3_SPI SEEED_MT3620_MDB_J1_ISU0_SPI

// MT3620 SPI Chip Select (CS) value "A". This is not a peripheral identifier, and so has no meaning in an app manifest.
#define SAMPLE_LSM6DS3_SPI_CS MT3620_SPI_CS_A

// MT3620 MDB: Connect external reset signal using J1, pin 1.
#define SAMPLE_NRF52_RESET SEEED_MT3620_MDB_J1_PIN11_GPIO34

// MT3620 MDB: Connect external dfu signal using J2, pin 13.
#define SAMPLE_NRF52_DFU SEEED_MT3620_MDB_J2_PIN13_GPIO31

// MT3620 MDB: Connect external NRF52 UART using J1, pin 7 (RX), pin 5 (TX), pin 8 (CTS), pin 6 (RTS).
#define SAMPLE_NRF52_UART SEEED_MT3620_MDB_J1_ISU0_UART


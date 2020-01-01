/*
 * Copyright (c) 2016-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RF_BLE_CONNECT_CONFIG_H
#define RF_BLE_CONNECT_CONFIG_H

#include <zephyr/types.h>

#ifndef Board_CC1352R1_LAUNCHXL
#define Board_CC1352R1_LAUNCHXL
#endif

#include "Board.h"

/* This file defines data types and variables for the application configuration */

/* PER version */
#define BLEC_VERSION "Ver 0.1"

/*
RF basic settings as found in the typical settings section of Smart RF Studio.
Each one defines a test case in this application.
*/
typedef enum
{
    RfSetup_Ble = 0,     // BLE
    RfSetup_Ble5,        // BLE5
    NrOfRfSetups
} RfSetup;

/* Whether the application works as sender or receiver */
typedef enum
{
    TestMode_Rx = 0,
    TestMode_Tx,
    NrOfTestModes
} TestMode;

/* Contains a pre-defined setting for frequency selection */
typedef struct
{
    const char* const label;
    const u16_t frequency;
    const u16_t fractFreq;
    const u8_t whitening; // BLE has freq dependent whitening settings
} FrequencyTableEntry;

/*
Holds the application config that is prepared in the menu and
used in the rx and tx functions.
*/
typedef struct
{
    RfSetup rfSetup;                     // Test case index
    TestMode testMode;                   // TX/RX mode index
    u32_t packetCount;
    u32_t  payloadLength;             // Desired payload length (bytes)
    FrequencyTableEntry* frequencyTable; // FrequencyTable for this test case
    u8_t frequency;                   // Index in config_frequencyTable
} ApplicationConfig;

extern FrequencyTableEntry*  config_frequencyTable_Lut[]; // Lookup table for freq table
extern const char* const config_testmodeLabels[];         // Lookup table for operation mode labels
extern const char* const config_rfSetupLabels[];          // Lookup table for RfSetup labels
extern const u32_t config_payloadLengthTable[];        // Lookup table for different payload length options

#endif /* RF_BLE_CONNECT_CONFIG_H */

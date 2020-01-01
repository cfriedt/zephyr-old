/*
 * Copyright (c) 2016-2018, Texas Instruments Incorporated
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


/***** Includes *****/
/* Standard C Libraries */
//#include <stdio.h>

#include <sys/printk.h>

#define Display_clear(handle)
#define Display_printf(handle, row, col, fmt, args...) printk( fmt, ##args )

/* TI Drivers */
//#include <ti/display/Display.h>
#include <ti/drivers/Power.h>
//#include <ti/drivers/pin/PINCC26XX.h>

/* Application specific Header files */
#include "menu.h"
#include "config.h"

#include "Board.h"

#ifndef DeviceFamily_CC13X2
#define DeviceFamily_CC13X2
#endif
#ifdef Board_SYSCONFIG_PREVIEW
#include "smartrf_settings/smartrf_settings.h"
#else
#include "smartrf_settings/smartrf_settings_predefined.h"
#endif //Board_SYSCONFIG_PREVIEW

/***** Defines *****/


#if 0
#if (defined Board_CC1350_LAUNCHXL)   || (defined Board_CC1350_LAUNCHXL_433) || \
    (defined Board_CC1310_LAUNCHXL)   || defined (Board_CC1352R1_LAUNCHXL)   || \
    (defined Board_CC1352P1_LAUNCHXL) || (defined Board_CC1352P_2_LAUNCHXL)  || \
    (defined Board_CC1352P_4_LAUNCHXL)
extern PIN_Handle buttonPinHandle;
#endif
#endif

/***** Variable declarations *****/

/* Events used in the application */
typedef enum
{
    MenuEvent_Navigate = 2,
    MenuEvent_Select = 1,
    MenuEvent_AnyButtonPushed = MenuEvent_Navigate + MenuEvent_Select,
} MenuEvent;

/* Menu row indices */
typedef enum
{
    TitleRow = 0,
    TestModeRow,
    ModulationRow,
    FrequencyRow,
    PacketCountRow,
    IntervalRow,
    PayloadLengthRow,
#if (defined Board_CC1310_LAUNCHXL)
    RangeExtenderRow,
#elif ((defined Board_CC1352P1_LAUNCHXL)   || (defined Board_CC1352P_2_LAUNCHXL) || \
       (defined Board_CC1352P_4_LAUNCHXL))
    HighPaRow,
#endif
    StartRow,
    NrOfMainMenuRows,
} MenuIndex;

/* String constants for different boards */
#if (defined Board_CC1310DK_7XD) || (defined Board_CC2650DK_7ID)
    static const char* const button0Text = "UP";
    static const char* const button1Text = "DOWN";
#elif (defined Board_CC1310_LAUNCHXL)     || (defined Board_CC1350_LAUNCHXL)    || \
      (defined Board_CC1350_LAUNCHXL_433) || (defined Board_CC1352R1_LAUNCHXL)  || \
      (defined Board_CC1352P1_LAUNCHXL)   || (defined Board_CC1352P_2_LAUNCHXL) || \
      (defined Board_CC1352P_4_LAUNCHXL)  || (defined Board_CC1312R1_LAUNCHXL)  || \
      (defined Board_CC2640R2_LAUNCHXL)   || (defined Board_CC2650_LAUNCHXL)    || \
      (defined Board_CC26X2R1_LAUNCHXL)
    static const char* const button0Text = "BTN-1";
    static const char* const button1Text = "BTN-2";
#elif defined Board_CC1350STK
    static const char* const button0Text = "LEFT";
    static const char* const button1Text = "RIGHT";
#else
    #error This board is not supported.
#endif

/* Convenience macros for printing on a vt100 terminal via UART */
#define vt100_print0(handle, row, col, text) \
    Display_printf(handle, 0, 0, "\x1b[%d;%df" text, row+1, col+1)

#define vt100_print1(handle, row, col, formatString, arg1) \
    Display_printf(handle, 0, 0, "\x1b[%d;%df" formatString, row+1, col+1, arg1)

#define vt100_print2(handle, row, col, formatString, arg1, arg2) \
    Display_printf(handle, 0, 0, "\x1b[%d;%df" formatString, row+1, col+1, arg1, arg2)

#define vt100_clear(handle) \
    Display_printf(handle, 0, 0, "\x1b[2J\x1b[H")

#define vt100_setCursorVisible(handle, visible) \
    Display_printf(handle, 0, 0, "\x1b[?25%c", ((visible) == true) ? 'h' : 'l')

/* Holds the configuration for the current test case */
static ApplicationConfig config =
{
    RfSetup_Ble,
    IntervalMode_Yes,
    TestMode_Tx,
    10,
    30,
    NULL,
    0,
#if (defined Board_CC1310_LAUNCHXL)
    RangeExtender_Dis
#elif ((defined Board_CC1352P1_LAUNCHXL)   || (defined Board_CC1352P_2_LAUNCHXL) || \
       (defined Board_CC1352P_4_LAUNCHXL))
    HighPa_Dis
#endif
};

//static Display_Handle lcdDisplay;
//static Display_Handle uartDisplay;
static volatile uint8_t eventFlag = 0;

/***** Prototypes *****/
void menu_runTask();

bool menu_isButtonPressed()
{
    return (eventFlag != 0);
}

/*
Menu task function.

This task contains the main application logic. It prints the menu on both,
LCD and UART and starts the RX and TX test cases.
The setup code is generated from the .cfg file.
*/
void menu_init()
{
    config.frequencyTable = config_frequencyTable_Lut[config.rfSetup];

    /* Init displays */
//    Display_Params params;
//    Display_Params_init(&params);
//    params.lineClearMode = DISPLAY_CLEAR_NONE;

//    lcdDisplay = Display_open(Display_Type_LCD, &params);
//    if(lcdDisplay == NULL)
//    {
//        while(1);
//    }

//    Display_clear(lcdDisplay);

//    uartDisplay = Display_open(Display_Type_UART, &params);
//    if(uartDisplay == NULL)
//    {
//        while(1);
//    }
}

void menu_runTask()
{

    uint8_t cursorRow = TestModeRow;
    uint8_t packetIndex = 0;
    uint8_t payloadIndex = 0;
    vt100_clear(uartDisplay);
    vt100_setCursorVisible(uartDisplay, false);

    /* Splash screen */
//    Display_printf(lcdDisplay, 0, 0, "PER TEST");
//    Display_printf(lcdDisplay, 1, 0, PER_VERSION);
//    Display_printf(lcdDisplay, 3, 0, "Select:   %s", button0Text);
//    Display_printf(lcdDisplay, 4, 0, "Navigate: %s", button1Text);
//    Display_printf(lcdDisplay, 6, 0, "Push a button");
//    Display_printf(lcdDisplay, 7, 0, "to proceed...");

    vt100_print0(uartDisplay, 0, 0, "PER TEST");
    vt100_print0(uartDisplay, 1, 0, PER_VERSION);
    vt100_print1(uartDisplay, 3, 0, "Select:   %s", button0Text);
    vt100_print1(uartDisplay, 4, 0, "Navigate: %s", button1Text);
    vt100_print0(uartDisplay, 6, 0, "Push a button");
    vt100_print0(uartDisplay, 7, 0, "to proceed...");

    bool previousHwiState = IntMasterDisable();
    // Tricky IntMasterDisable():
    //true  : Interrupts were already disabled when the function was called.
    //false : Interrupts were enabled and are now disabled.
    while(!eventFlag){
        IntMasterEnable();
        Power_idleFunc();
        IntMasterDisable();

    }
    if(!previousHwiState)
    {
        IntMasterEnable();
    }
    eventFlag = 0;
    //Display_clear(lcdDisplay);
    vt100_clear(uartDisplay);

    while(true)
    {
#if 0
        /* Main Menu */
        Display_printf(lcdDisplay, 0, 0, "Main Menu");
        Display_printf(lcdDisplay, TestModeRow, 0,      " Test: %s", config_testmodeLabels[config.testMode]);
        Display_printf(lcdDisplay, ModulationRow, 0,    " Mode: %s", config_rfSetupLabels[config.rfSetup]);
        Display_printf(lcdDisplay, FrequencyRow, 0,     " Freq: %s", config.frequencyTable[config.frequency].label);
        Display_printf(lcdDisplay, PacketCountRow, 0,   " Pkts: %-5d", config.packetCount);
        if(config.testMode == TestMode_Rx)
        {
            Display_printf(lcdDisplay, IntervalRow, 0,      " Interval: -- ");
            Display_printf(lcdDisplay, PayloadLengthRow, 0, " Length: -- ");
#if (defined Board_CC1310_LAUNCHXL)
            Display_printf(lcdDisplay, RangeExtenderRow, 0, " CC1190: --");
#elif ((defined Board_CC1352P1_LAUNCHXL)   || (defined Board_CC1352P_2_LAUNCHXL) || \
       (defined Board_CC1352P_4_LAUNCHXL))
            Display_printf(lcdDisplay, HighPaRow, 0,        " HighPA: --      ");
#endif
        }
        else
        {
            Display_printf(lcdDisplay, IntervalRow, 0,      " Interval: %s", config_intervalLabels[config.intervalMode]);
            Display_printf(lcdDisplay, PayloadLengthRow, 0, " Length: %-3d", config.payloadLength);
#if (defined Board_CC1310_LAUNCHXL)
            Display_printf(lcdDisplay, RangeExtenderRow, 0, " CC1190: %s", config_rangeExtenderLabels[config.rangeExtender]);
#elif ((defined Board_CC1352P1_LAUNCHXL)   || (defined Board_CC1352P_2_LAUNCHXL) || \
       (defined Board_CC1352P_4_LAUNCHXL))
            Display_printf(lcdDisplay, HighPaRow, 0,        " HighPA: %s", config_highPaLabels[config.highPa]);
#endif
        }
        Display_printf(lcdDisplay, StartRow, 0, " Start...");
#endif

        vt100_print0(uartDisplay, 0, 0, "Main Menu");
        vt100_print1(uartDisplay, TestModeRow, 0,      " Test: %s", config_testmodeLabels[config.testMode]);
        vt100_print1(uartDisplay, ModulationRow, 0,    " Mode: %s", config_rfSetupLabels[config.rfSetup]);
        vt100_print1(uartDisplay, FrequencyRow, 0,     " Freq: %s", config.frequencyTable[config.frequency].label);
        vt100_print1(uartDisplay, PacketCountRow, 0,   " Pkts: %-5d", config.packetCount);
        if(config.testMode == TestMode_Rx)
        {
            vt100_print0(uartDisplay, IntervalRow, 0,      " Interval: -- ");
            vt100_print0(uartDisplay, PayloadLengthRow, 0, " Length: -- ");
#if (defined Board_CC1310_LAUNCHXL)
            vt100_print0(uartDisplay, RangeExtenderRow, 0, " CC1190: --");
#elif ((defined Board_CC1352P1_LAUNCHXL)   || (defined Board_CC1352P_2_LAUNCHXL) || \
       (defined Board_CC1352P_4_LAUNCHXL))
            vt100_print0(uartDisplay, HighPaRow, 0,        " HighPA: --      ");
#endif
        }
        else
        {
            vt100_print1(uartDisplay, IntervalRow, 0,      " Interval: %s", config_intervalLabels[config.intervalMode]);
            vt100_print1(uartDisplay, PayloadLengthRow, 0, " Length: %-3d", config.payloadLength);
#if (defined Board_CC1310_LAUNCHXL)
            vt100_print1(uartDisplay, RangeExtenderRow, 0, " CC1190: %s", config_rangeExtenderLabels[config.rangeExtender]);
#elif ((defined Board_CC1352P1_LAUNCHXL)   || (defined Board_CC1352P_2_LAUNCHXL) || \
        (defined Board_CC1352P_4_LAUNCHXL))
            vt100_print1(uartDisplay, HighPaRow, 0,        " HighPA: %s", config_highPaLabels[config.highPa]);
#endif
        }

        vt100_print0(uartDisplay, StartRow, 0, " Start...");


        /* Print the selector */
        //Display_printf(lcdDisplay, cursorRow, 0, ">");
        vt100_print0(uartDisplay, cursorRow, 0, ">" "\x1b[1A"); // Overlay selector and cursor

        previousHwiState = IntMasterDisable();
        /* Navigation is done event based. Events are created from button interrupts */
        while(!eventFlag)
        {
            IntMasterEnable();
            Power_idleFunc();
            IntMasterDisable();
        }
        if(!previousHwiState)
        {
            IntMasterEnable();
        }
        if (eventFlag & MenuEvent_Navigate)
        {
            eventFlag &= !MenuEvent_Navigate;
            cursorRow++;
            if (cursorRow >= NrOfMainMenuRows)
            {
                cursorRow = TestModeRow;
            }
        }
        if (eventFlag & MenuEvent_Select)
        {
            eventFlag &= !MenuEvent_Select;
            switch(cursorRow)
            {
                case TestModeRow:
                    config.testMode = (TestMode)((config.testMode + 1) % NrOfTestModes);
                    break;

                case ModulationRow:
                    config.rfSetup = (RfSetup)((config.rfSetup + 1) % NrOfRfSetups);
                    config.frequencyTable = config_frequencyTable_Lut[config.rfSetup];
                    config.frequency = 0;
#if ((defined Board_CC1352P1_LAUNCHXL)   || (defined Board_CC1352P_2_LAUNCHXL) || \
    (defined Board_CC1352P_4_LAUNCHXL))
                    config.highPa = HighPa_Dis;
#endif
#if (defined Board_CC2650DK_7ID)        || (defined Board_CC2650_LAUNCHXL)    || \
    (defined Board_CC2640R2_LAUNCHXL)   || (defined Board_CC1350_LAUNCHXL)    || \
    (defined Board_CC1350_LAUNCHXL_433) || (defined Board_CC1350STK)          || \
    (defined Board_CC26X2R1_LAUNCHXL)   || (defined Board_CC1352R1_LAUNCHXL)  || \
    (defined Board_CC1352P1_LAUNCHXL)   || (defined Board_CC1352P_2_LAUNCHXL) || \
    (defined Board_CC1352P_4_LAUNCHXL)
                    if ((config.rfSetup == RfSetup_Ble)
#if (defined Board_CC2640R2_LAUNCHXL)  || (defined Board_CC1352R1_LAUNCHXL)  || \
    (defined Board_CC1352P1_LAUNCHXL)  || (defined Board_CC1352P_2_LAUNCHXL) || \
    (defined Board_CC1352P_4_LAUNCHXL) || (defined Board_CC26X2R1_LAUNCHXL)
                    || (config.rfSetup == RfSetup_Ble5)
#endif
                       )
                    {
                        // Fixed payload length of 30
                        payloadIndex = 0;
                        config.payloadLength = config_payloadLengthTable[payloadIndex];
                    }
#endif

#if (defined Board_CC1352P_4_LAUNCHXL)
                    if(config.rfSetup == RfSetup_Sl_lr)
                    {

                        // SLR mode only works in the 433 MHz band irrespective of the PA
                        // setting.
                        config.frequency = 0;
                    }
#endif
                    break;

                case FrequencyRow:
#if (defined Board_CC1352P_4_LAUNCHXL)
                    // Frequency is fixed for the SLR mode. Custom inherits from
                    // smartrf_settings.c
                    if ((config.rfSetup != RfSetup_Sl_lr) && (config.rfSetup != RfSetup_Custom))
#else
                    // Custom settings only. Use the freq from smartrf_settings.c
                    if (config.rfSetup != RfSetup_Custom)
#endif
                    {
                        config.frequency = (config.frequency + 1);
                        if(config.frequencyTable[config.frequency].frequency == 0xFFFF)
                        {
                            config.frequency = 0;
                        }
                    }
                    break;

            case PacketCountRow:
                packetIndex = (packetIndex + 1) % config_NrOfPacketCounts;
                config.packetCount = config_packetCountTable[packetIndex];
                break;

            case IntervalRow:
                config.intervalMode = (IntervalMode)((config.intervalMode + 1) % NrOfIntervalModes);
                if(config.intervalMode == IntervalMode_Yes)
                {
                    /* Fixed payload length of 30 */
                    payloadIndex = 0;
                    config.payloadLength = config_payloadLengthTable[payloadIndex];
                }
                break;

            case PayloadLengthRow:
                if((config.intervalMode == IntervalMode_No)
#if (defined Board_CC2650DK_7ID)        || (defined Board_CC2650_LAUNCHXL)    || \
    (defined Board_CC2640R2_LAUNCHXL)   || (defined Board_CC1350_LAUNCHXL)    || \
    (defined Board_CC1350_LAUNCHXL_433) || (defined Board_CC1350STK)          || \
    (defined Board_CC26X2R1_LAUNCHXL)   || (defined Board_CC1352R1_LAUNCHXL)  || \
    (defined Board_CC1352P1_LAUNCHXL)   || (defined Board_CC1352P_2_LAUNCHXL) || \
    (defined Board_CC1352P_4_LAUNCHXL)
                    && (config.rfSetup != RfSetup_Ble)
#if (defined Board_CC2640R2_LAUNCHXL)  || (defined Board_CC1352R1_LAUNCHXL)  || \
    (defined Board_CC1352P1_LAUNCHXL)  || (defined Board_CC1352P_2_LAUNCHXL) || \
    (defined Board_CC1352P_4_LAUNCHXL) || (defined Board_CC26X2R1_LAUNCHXL)
                    && (config.rfSetup != RfSetup_Ble5)
#endif
#endif
                  )
                {
                    payloadIndex = (payloadIndex + 1) % config_NrOfPayloadLengths;
                }
                config.payloadLength = config_payloadLengthTable[payloadIndex];
                break;


#if (defined Board_CC1310_LAUNCHXL)
            case RangeExtenderRow:

                config.rangeExtender = (RangeExtender)((config.rangeExtender + 1) % NrOfRangeExtender);
                if (config.rangeExtender == RangeExtender_Dis)
                {
                    /* Reverse the pins to GPIO mapping */
                    PINCC26XX_setMux(buttonPinHandle, Board_LNA_EN, -1);
                    PINCC26XX_setMux(buttonPinHandle, Board_PA_EN, -1);
                }
                else
                {
                    /* Configure the pins tp control the range extender */
                    PINCC26XX_setMux(buttonPinHandle, Board_LNA_EN, PINCC26XX_MUX_RFC_GPO0);
                    PINCC26XX_setMux(buttonPinHandle, Board_PA_EN, PINCC26XX_MUX_RFC_GPO1);
                }
                break;
#elif ((defined Board_CC1352P1_LAUNCHXL)   || (defined Board_CC1352P_2_LAUNCHXL) || \
       (defined Board_CC1352P_4_LAUNCHXL))
            case HighPaRow:
#if ((defined Board_CC1352P1_LAUNCHXL)   || (defined Board_CC1352P_4_LAUNCHXL))
                if((config.rfSetup == RfSetup_Ble) || (config.rfSetup == RfSetup_Ble5))
                {
                    // CC1352P1 and CC1352P-4 do not support high PA BLE Tx
                    config.highPa = HighPa_Dis;
                }
#elif (defined Board_CC1352P_2_LAUNCHXL)
                if(!(config.rfSetup == RfSetup_Ble) && !(config.rfSetup == RfSetup_Ble5))
                {
                    // CC1352P-2 does not support high PA Sub-1 Tx
                    config.highPa = HighPa_Dis;
                }
#endif
                else
                {
                    config.highPa = (HighPa)((config.highPa + 1) % NrOfHighPa);
                }
                break;
#endif

            case StartRow:

                if (config.testMode == TestMode_Rx)
                {
#if 0
                    /* Prepare RX display */
                    Display_clear(lcdDisplay);
                    Display_printf(lcdDisplay, 0, 0, "Receiving...");
                    Display_printf(lcdDisplay, 1, 0, "%s %s",
                            config_rfSetupLabels[config.rfSetup],
                            config.frequencyTable[config.frequency].label);
                    Display_printf(lcdDisplay, 2, 0, "Pkts ok   :%-5d", 0);
                    Display_printf(lcdDisplay, 3, 0, "RSSI [dBm]:n/a");
                    Display_printf(lcdDisplay, 4, 0, "TP[bps]:n/a");
                    Display_printf(lcdDisplay, 5, 0, "PER  [%%] :n/a");
                    Display_printf(lcdDisplay, 7, 0, "Push a button");
                    Display_printf(lcdDisplay, 8, 0, "to abort.");
#endif
                    vt100_clear(uartDisplay);
                    vt100_print0(uartDisplay, 0, 0, "Receiving...");
                    vt100_print2(uartDisplay, 1, 0, "%s %s",
                            config_rfSetupLabels[config.rfSetup],
                            config.frequencyTable[config.frequency].label);
                    vt100_print1(uartDisplay, 2, 0, "Pkts ok   : %-5d", 0);
                    vt100_print0(uartDisplay, 3, 0, "RSSI [dBm]: n/a");
                    vt100_print0(uartDisplay, 4, 0, "TP[bps]: n/a");
                    vt100_print0(uartDisplay, 5, 0, "PER  [%%] : n/a");
                    vt100_print0(uartDisplay, 7, 0, "Push a button");
                    vt100_print0(uartDisplay, 8, 0, "to abort.");

                    /* Run the test. */
                    IntMasterEnable();
                    TestResult result = rx_runRxTest(&config);
                    if (result == TestResult_Finished)
                    {
                        //Display_printf(lcdDisplay, 7, 0, "...finished. ");
                        vt100_print0(uartDisplay, 7, 0, "...finished. ");

                        //Display_printf(lcdDisplay, 8, 0, "Push a button...");
                        vt100_print0(uartDisplay, 8, 0, "Push a button...");
                        previousHwiState = IntMasterDisable();
                        while(!eventFlag)
                        {
                            IntMasterEnable();
                            Power_idleFunc();
                            IntMasterDisable();
                        }
                        if(!previousHwiState)
                        {
                            IntMasterEnable();
                        }
                    }
                    eventFlag = 0;
                }
                else
                {

#if 0
                    /* Prepare TX display */
                    Display_clear(lcdDisplay);
                    Display_printf(lcdDisplay, 0, 0, "Sending...");
                    Display_printf(lcdDisplay, 1, 0, "%s %s",
                            config_rfSetupLabels[config.rfSetup],
                            config.frequencyTable[config.frequency].label);
                    Display_printf(lcdDisplay, 3, 0, "Pkts sent: %-5d", 0);
#endif
                    vt100_clear(uartDisplay);
                    vt100_print0(uartDisplay, 0, 0, "Sending...");
                    vt100_print2(uartDisplay, 1, 0, "%s %s",
                            config_rfSetupLabels[config.rfSetup],
                            config.frequencyTable[config.frequency].label);
                    vt100_print1(uartDisplay, 3, 0, "Pkts sent: %-5d", 0);

                    /* Run the test. */
                    IntMasterEnable();
                    TestResult result = tx_runTxTest(&config);
                    if (result == TestResult_Aborted)
                    {
                        //Display_printf(lcdDisplay, 8, 0, "...aborted.");
                        vt100_print0(uartDisplay, 8, 0, "...aborted.");
                        eventFlag = 0;
                    }
                    else if (result == TestResult_Finished)
                    {
                        //Display_printf(lcdDisplay, 8, 0, "...finished.");
                        vt100_print0(uartDisplay, 8, 0, "...finished.");
                    }
                    //Display_printf(lcdDisplay, 9, 0, "Push a button...");
                    vt100_print0(uartDisplay, 9, 0, "Push a button...");

                    previousHwiState = IntMasterDisable();
                    while(!eventFlag)
                    {
                        IntMasterEnable();
                        Power_idleFunc();
                        IntMasterDisable();
                    }
                    if(!previousHwiState)
                    {
                        IntMasterEnable();
                    }
                    eventFlag = 0;
                }
                //Display_clear(lcdDisplay);
                vt100_clear(uartDisplay);
                break;
            }
        }
    }
}

/*
Callback for button interrupts.

This function is supposed to be called asynchronously from within an interrupt
handler and signals a button press event to the application logic.
*/
void menu_notifyButtonPressed(Button button)
{
    if (button == Button_Navigate)
    {
        eventFlag |= MenuEvent_Navigate;
    }
    else
    {
        eventFlag |= MenuEvent_Select;
    }
}

/*
Updates the screen content during an ongoing receive.

Call this function from any other task to refresh the menu with
updated parameters.
*/
void menu_updateRxScreen(rx_metrics *metrics)
{
#if 0
    Display_printf(lcdDisplay, 2, 11, "%-5d", metrics->packetsReceived);
    Display_printf(lcdDisplay, 3, 11, "%-5i", metrics->rssi);
    Display_printf(lcdDisplay, 4, 8, "%-7d", metrics->throughput);
    Display_printf(lcdDisplay, 5, 0, "PER  [%%] :");
#endif

    vt100_print1(uartDisplay, 2, 0, "Pkts ok   : %-5d", metrics->packetsReceived);
    vt100_print1(uartDisplay, 3, 0, "RSSI [dBm]: %-5i", metrics->rssi);
    vt100_print1(uartDisplay, 4, 0, "TP[bps]: %-7d", metrics->throughput);
    vt100_print0(uartDisplay, 5, 0, "PER  [%%] :");


    /* Convert float to string buffer */
    if ((metrics->packetsReceived <= config.packetCount) &&
        (metrics->packetsReceived <= metrics->packetsExpected))
    {
        /* Avoid a 0.0/0.0 (NaN) or a x/0.0 (+Inf) condition */
        float per =  0.0f;
        if(metrics->packetsExpected > 0)
        {
            per = ((float)(metrics->packetsMissed)/(float)(metrics->packetsExpected))*100.0f;
        }
        int characteristic = (int)per;
        int mantissa = (int)((per - characteristic)*100);

        //Display_printf(lcdDisplay, 5, 10, "%3d", characteristic);
        vt100_print1(uartDisplay, 5, 10, "%3d", characteristic);

        //Display_printf(lcdDisplay, 5, 13, "%c", '.');
        vt100_print1(uartDisplay, 5, 13, "%c", '.');

        //Display_printf(lcdDisplay, 5, 14, "%-2d", mantissa);
        vt100_print1(uartDisplay, 5, 14, "%-2d", mantissa);
    }
    else
    {
        char buffer[6] = "n/a  ";
        //Display_printf(lcdDisplay, 5, 11, "%s", (char *)&buffer);
        vt100_print1(uartDisplay, 5, 0, "PER  [%%] : %s", (char *)&buffer);
    }
}

/*
Updates the screen content during an ongoing transmission.

Call this function from any other task to refresh the menu with
updated parameters.
 */
void menu_updateTxScreen(uint32_t packetsSent)
{
    //Display_printf(lcdDisplay, 3, 11, "%-5d", packetsSent);
    vt100_print1(uartDisplay, 3, 11, "%-5d", packetsSent);
}

/*
Updates the screen content during an ongoing transmission. This includes
TX metrics like Transmission Power (dBm), Data Rate (bps) and Packet Interval
(ms)

Call this function from any other task to refresh the menu with
updated parameters.
 */
void menu_updateTxMetricScreen(tx_metrics *metrics)
{
    //Display_printf(lcdDisplay, 4, 0, "Pwr[dBm]: %-4d", metrics->transmitPowerDbm);
    vt100_print1(uartDisplay, 4, 0,  "Pwr[dBm]: %-4d", metrics->transmitPowerDbm);

    //Display_printf(lcdDisplay, 5, 0, "DR[bps]: %-7d", metrics->dataRateBps);
    vt100_print1(uartDisplay, 5, 0,  "DR[bps]: %-7d", metrics->dataRateBps);

    //Display_printf(lcdDisplay, 6, 0, "Interval: %-3d ms", metrics->packetIntervalMs);
    vt100_print1(uartDisplay, 6, 0,  "Interval: %-3d ms", metrics->packetIntervalMs);
}

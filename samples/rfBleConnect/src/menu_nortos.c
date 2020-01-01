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
#include <stddef.h>

#include <sys/printk.h>

#define Display_clear(handle) printk("\x1b[2J\x1b[H")
#define Display_printf(handle, row, col, fmt, args...) printk( fmt, ##args )

/* TI Drivers */
//#include <ti/display/Display.h>
#include <ti/drivers/Power.h>
//#include <ti/drivers/pin/PINCC26XX.h>

/* Application specific Header files */
#include "config.h"
#include "menu.h"

#include "Board.h"

#ifndef DeviceFamily_CC13X2
#define DeviceFamily_CC13X2
#endif
#include "smartrf_settings/smartrf_settings_predefined.h"

/***** Defines *****/


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
    StartRow,
    NrOfMainMenuRows,
} MenuIndex;

/* String constants for different boards */
    static const char* const button0Text = "BTN-1";
    static const char* const button1Text = "BTN-2";

/* Convenience macros for printing on a vt100 terminal via UART */
#define vt100_print0(handle, row, col, text) \
    Display_printf(handle, 0, 0, "\x1b[%d;%df" text, row+1, col+1)

#define vt100_print1(handle, row, col, formatString, arg1) \
    Display_printf(handle, 0, 0, "\x1b[%d;%df" formatString, row+1, col+1, arg1)

#define vt100_print2(handle, row, col, formatString, arg1, arg2) \
    Display_printf(handle, 0, 0, "\x1b[%d;%df" formatString, row+1, col+1, arg1, arg2)

#define vt100_clear(handle) \
    Display_printf(handle, 0, 0, "\x1b[2J\x1b[H")

void menu_clear(void) {
	vt100_clear( Hufflepuff! );
}

#define vt100_setCursorVisible(handle, visible) \
    Display_printf(handle, 0, 0, "\x1b[?25%c", ((visible) == true) ? 'h' : 'l')

/* Holds the configuration for the current test case */
static ApplicationConfig config =
{
    RfSetup_Ble,
    TestMode_Tx,
    10,
	30,
    NULL,
    0,
};

//static Display_Handle lcdDisplay;
//static Display_Handle uartDisplay;
static volatile u8_t eventFlag = 0;

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
}

void menu_runTask()
{

    u8_t cursorRow = TestModeRow;
    u8_t payloadIndex = 0;
    vt100_clear(uartDisplay);
    vt100_setCursorVisible(uartDisplay, false);

    /* Splash screen */
    vt100_print0(uartDisplay, 0, 0, "BLE Connect Test");
    vt100_print0(uartDisplay, 1, 0, BLEC_VERSION);
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
        vt100_print0(uartDisplay, 0, 0, "Main Menu");
        vt100_print1(uartDisplay, TestModeRow, 0,      " Test: %s", config_testmodeLabels[config.testMode]);
        vt100_print1(uartDisplay, ModulationRow, 0,    " Mode: %s", config_rfSetupLabels[config.rfSetup]);
        if(config.testMode == TestMode_Rx)
        {
//            vt100_print0(uartDisplay, IntervalRow, 0,      " Interval: -- ");
//            vt100_print0(uartDisplay, PayloadLengthRow, 0, " Length: -- ");
        }
        else
        {
//            vt100_print1(uartDisplay, IntervalRow, 0,      " Interval: %s", config_intervalLabels[config.intervalMode]);
//            vt100_print1(uartDisplay, PayloadLengthRow, 0, " Length: %-3d", config.payloadLength);
        }

        vt100_print0(uartDisplay, StartRow, 0, " Start...");

        /* Print the selector */
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
                    if ( config.rfSetup == RfSetup_Ble || config.rfSetup == RfSetup_Ble5 )
                    {
                        // Fixed payload length of 30
                        payloadIndex = 0;
                        config.payloadLength = config_payloadLengthTable[payloadIndex];
                    }
                    break;

            case StartRow:

                if (config.testMode == TestMode_Rx)
                {
                    /* Prepare RX display */
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
                        vt100_print0(uartDisplay, 13, 0, "...finished. ");

                        //Display_printf(lcdDisplay, 8, 0, "Push a button...");
                        vt100_print0(uartDisplay, 14, 0, "Push a button...");
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

                    /* Prepare TX display */
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
                        vt100_print0(uartDisplay, 13, 0, "...aborted.");
                        eventFlag = 0;
                    }
                    else if (result == TestResult_Finished)
                    {
                        //Display_printf(lcdDisplay, 8, 0, "...finished.");
                        vt100_print0(uartDisplay, 13, 0, "...finished.");
                    }
                    //Display_printf(lcdDisplay, 9, 0, "Push a button...");
                    vt100_print0(uartDisplay, 14, 0, "Push a button...");

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
#endif
}

/*
Updates the screen content during an ongoing transmission.

Call this function from any other task to refresh the menu with
updated parameters.
 */
void menu_updateTxScreen(u32_t packetsSent)
{
    //vt100_print1(uartDisplay, 3, 11, "%-5d", packetsSent);
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
	extern RF_EventMask ble_adv_event_mask;
	rfc_bleAdvOutput_t *x = (rfc_bleAdvOutput_t *) metrics;

	vt100_print1(uartDisplay,  0, 0, "EventMask:     %08x\x1b[K", (unsigned)ble_adv_event_mask );
	vt100_print1(uartDisplay,  1, 0, "nTxAdvInd:     %u\x1b[K", x->nTxAdvInd );
	vt100_print1(uartDisplay,  2, 0, "nTxScanRsp:    %u\x1b[K", x->nTxScanRsp );
	vt100_print1(uartDisplay,  3, 0, "nRxScanReq:    %u\x1b[K", x->nRxScanReq );
	vt100_print1(uartDisplay,  4, 0, "nRxConnectReq: %u\x1b[K", x->nRxConnectReq );
	vt100_print1(uartDisplay,  5, 0, "nTxConnectRsp: %u\x1b[K", x->nTxConnectRsp );
	vt100_print1(uartDisplay,  6, 0, "nRxNok:        %u\x1b[K", x->nRxNok );
	vt100_print1(uartDisplay,  7, 0, "nRxIgnored:    %u\x1b[K", x->nRxIgnored );
	vt100_print1(uartDisplay,  8, 0, "nRxBufFull:    %u\x1b[K", x->nRxBufFull );
	vt100_print1(uartDisplay,  9, 0, "lastRssi:      %d\x1b[K", x->lastRssi );
	vt100_print1(uartDisplay, 10, 0, "timeStamp:     %u\x1b[K", x->timeStamp );
}

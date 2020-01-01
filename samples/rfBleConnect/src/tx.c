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
#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>

#include <zephyr.h>

#ifndef CONFIG_NET_BUF_USER_DATA_SIZE
#define CONFIG_NET_BUF_USER_DATA_SIZE 0
#endif

#include <bluetooth/gap.h>
#include <bluetooth/uuid.h>
#include <bluetooth/bluetooth.h>
#include "../../../subsys/bluetooth/controller/ll_sw/pdu.h"

#define Board_CC1352R1_LAUNCHXL
#ifndef DeviceFamily_CC13X2
#define DeviceFamily_CC13X2
#endif

/* TI-RTOS Header files */
#include <ti/drivers/rf/RF.h>
//#include <ti/drivers/PIN.h>
#include <driverlib/rf_mailbox.h>

/* Board Header files */
#include "Board.h"

/* Application specific Header files */
#include "config.h"
#include "menu.h"
#include "RFQueue.h"

#include "smartrf_settings/smartrf_settings.h"
#include "smartrf_settings/smartrf_settings_predefined.h"
#include "smartrf_settings/smartrf_settings_ble.h"

/***** Defines *****/
#define MAX_PAYLOAD_LENGTH      254 // Maximum length of the packet to send (Even due to HS requirement)
#define DATA_ENTRY_HEADER_SIZE  8   // Constant header size of a Generic Data Entry
#define MAX_LENGTH              MAX_PAYLOAD_LENGTH // Set the length of the data entry
#define NUM_DATA_ENTRIES        4
#define NUM_APPENDED_BYTES      0

#define EXTENDED_HEADER_LENGTH  9
#define BLE_BASE_FREQUENCY      2300 // When programming the channel in the BLE TX command it is the
                                     // offset from 2300 MHz

#define MAX_BLE_PWR_LEVEL_DBM   5
#define MAX_SUB1_PWR_LEVEL_DBM  13

#define ABORT_GRACEFUL          1   // Option for the RF cancel command
#define ABORT_ABRUPT            0   // Option for the RF cancel command

/* Inter-packet intervals for each phy mode in ms*/
#define PKT_INTERVAL_MS_BLE     100

#define BLE_SLAVE_CONN_INTERVAL_MIN 0x0150
#define BLE_SLAVE_CONN_INTERVAL_MAX 0x01F0

/***** Prototypes *****/
static void tx_callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);

/***** Variable declarations *****/
static RF_Object rfObject;
static RF_Handle rfHandle;

static ApplicationConfig localConfig;
static volatile u32_t ble_adv_cmd_start;
static volatile u32_t ble_adv_cmd_end;
static volatile RF_EventMask ble_adv_events_notified;
static volatile RF_CmdHandle current_rf_cmd_handle = -1;

static const u8_t ble_adv_channels[] = { 37, 38, 39 };

typedef enum {
	BLE_STATE_ADV_STBY,
	BLE_STATE_ADV_SCAN,
	BLE_STATE_ADV_SYNC,
	BLE_STATE_ADV_INIT,
	BLE_STATE_ADV_CONN,
	BLE_STATE_ADV_ADVT,
} BleStateAdv_t;

static BleStateAdv_t ble_adv_state;

/*
This interval is dependent on data rate and packet length, and might need to be changed
if any of these parameter changes
*/
static u32_t packetInterval;

__attribute__ ((aligned (4)))
static u8_t rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                                 MAX_LENGTH,
                                                                 NUM_APPENDED_BYTES)];

/* TX queue or RF Core to read data from */
static dataQueue_t rxDataQueue;
rfc_ble5ExtAdvEntry_t ble5ExtAdvPacket;

static u16_t device_address[] = {
	0x4455,
	0x2233,
	0x0011,
};
static u8_t adv_data[] = {
	2, // length
	BT_DATA_FLAGS, // key
	BT_LE_AD_LIMITED | BT_LE_AD_NO_BREDR, // value
	3, // length
	BT_DATA_UUID16_ALL, // key
	// BT_UUID_HRS, // value
	0x18, 0x0d, // value it would be better if this was available as a macro..
};
static u8_t scan_rsp_data[] = {
	14, // length
	BT_DATA_NAME_COMPLETE, // key
	'L','i','s','b','o','a',' ','(','B','e','t','a',')', // value
#if 0
	5, // length
	0x12, // key = slave connect interval range
	BLE_SLAVE_CONN_INTERVAL_MIN & 0xff,
	(BLE_SLAVE_CONN_INTERVAL_MIN >> 8) & 0xff,
	BLE_SLAVE_CONN_INTERVAL_MAX & 0xff,
	(BLE_SLAVE_CONN_INTERVAL_MAX >> 8) & 0xff
#endif
};
static rfc_bleAdvOutput_t adv_output;

// least amount of time for largest advertisement packet (using 1MHz phy)
#define BLE_ADV_TIME_MIN_US (LL_HEADER_SIZE(1) + sizeof(struct pdu_adv_adv_ind))
// least amount of time for largest scan req packet (using 1MHz phy)
#define BLE_SCAN_REQ_TIME_MIN_US (LL_HEADER_SIZE(1) + sizeof(struct pdu_adv_scan_req))
// least amount of time for largest scan rsp packet (using 1MHz phy)
#define BLE_SCAN_RSP_TIME_MIN_US (LL_HEADER_SIZE(1) + sizeof(struct pdu_adv_scan_rsp))
// least amount of time for largest connect req packet (using 1MHz phy)
#define BLE_CONNECT_TIME_MIN_US (LL_HEADER_SIZE(1) + sizeof(struct pdu_adv_connect_ind))

// least amount of time for a single slot to transmit an advertisement and receive a response
#define BLE_ADV_SLOT_TIME_MIN_US \
	(\
		0 \
		+ BLE_ADV_TIME_MIN_US \
		+ EVENT_MAFS_MAX_US \
		+ MIN( \
			BLE_SCAN_REQ_TIME_MIN_US + EVENT_MAFS_MAX_US + BLE_SCAN_RSP_TIME_MIN_US, \
			BLE_CONNECT_TIME_MIN_US \
		) \
	)
#define BLE_ADV_SLOT_TIME_MIN RF_convertUsToRatTicks( BLE_ADV_SLOT_TIME_MIN_US )

// the chosen amount of time to spend advertising on all selected channels
#define BLE_ADV_INTERVAL_US 4000000
#define BLE_ADV_INTERVAL RF_convertUsToRatTicks( BLE_ADV_INTERVAL_US )

static u32_t origin;
static inline void split_sec_usec( u32_t usec, u32_t *sec, u32_t *remainder ) {
	if ( sec ) {
		*sec = usec / 1000000;
	}
	if ( remainder ) {
		*remainder = usec % 1000000;
	}
}
#define D(fmt, args...) \
	do { \
		u32_t _now_ ## __LINE__, _sec_ ## __LINE__, _usec_ ## __LINE__; \
		_now_ ## __LINE__ = RF_getCurrentTime(); \
		_now_ ## __LINE__ -= origin; \
		_now_ ## __LINE__ = RF_convertRatTicksToUs( _now_ ## __LINE__ ); \
		split_sec_usec( _now_ ## __LINE__, & _sec_ ## __LINE__, & _usec_ ## __LINE__ ); \
		printk( "[%3u.%06u]: " fmt "\n", _sec_ ## __LINE__ % 1000, _usec_ ## __LINE__, ##args ); \
	} while( 0 )


static ratmr_t ble_adv_interval_start_time;
static ratmr_t ble_adv_interval_end_time;

extern const FrequencyTableEntry config_frequencyTable_ble[];
static const FrequencyTableEntry* chan_to_fte[] = {
	[ 37 ] = & config_frequencyTable_ble[ 0 ],
	[ 0 ] = & config_frequencyTable_ble[ 1 ],
	[ 1 ] = & config_frequencyTable_ble[ 2 ],
	[ 2 ] = & config_frequencyTable_ble[ 3 ],
	[ 3 ] = & config_frequencyTable_ble[ 4 ],
	[ 4 ] = & config_frequencyTable_ble[ 5 ],
	[ 5 ] = & config_frequencyTable_ble[ 6 ],
	[ 6 ] = & config_frequencyTable_ble[ 7 ],
	[ 7 ] = & config_frequencyTable_ble[ 8 ],
	[ 8 ] = & config_frequencyTable_ble[ 9 ],
	[ 9 ] = & config_frequencyTable_ble[ 10 ],
	[ 10 ] = & config_frequencyTable_ble[ 11 ],
	[ 38 ] = & config_frequencyTable_ble[ 12 ],
	[ 11 ] = & config_frequencyTable_ble[ 13 ],
	[ 12 ] = & config_frequencyTable_ble[ 14 ],
	[ 13 ] = & config_frequencyTable_ble[ 15 ],
	[ 14 ] = & config_frequencyTable_ble[ 16 ],
	[ 15 ] = & config_frequencyTable_ble[ 17 ],
	[ 16 ] = & config_frequencyTable_ble[ 18 ],
	[ 17 ] = & config_frequencyTable_ble[ 19 ],
	[ 18 ] = & config_frequencyTable_ble[ 20 ],
	[ 19 ] = & config_frequencyTable_ble[ 21 ],
	[ 20 ] = & config_frequencyTable_ble[ 22 ],
	[ 21 ] = & config_frequencyTable_ble[ 23 ],
	[ 22 ] = & config_frequencyTable_ble[ 24 ],
	[ 23 ] = & config_frequencyTable_ble[ 25 ],
	[ 24 ] = & config_frequencyTable_ble[ 26 ],
	[ 25 ] = & config_frequencyTable_ble[ 27 ],
	[ 26 ] = & config_frequencyTable_ble[ 28 ],
	[ 27 ] = & config_frequencyTable_ble[ 29 ],
	[ 28 ] = & config_frequencyTable_ble[ 30 ],
	[ 29 ] = & config_frequencyTable_ble[ 31 ],
	[ 30 ] = & config_frequencyTable_ble[ 32 ],
	[ 31 ] = & config_frequencyTable_ble[ 33 ],
	[ 32 ] = & config_frequencyTable_ble[ 34 ],
	[ 33 ] = & config_frequencyTable_ble[ 35 ],
	[ 34 ] = & config_frequencyTable_ble[ 36 ],
	[ 35 ] = & config_frequencyTable_ble[ 37 ],
	[ 36 ] = & config_frequencyTable_ble[ 38 ],
	[ 39 ] = & config_frequencyTable_ble[ 39 ],
};

enum {
	//TXMASK = RF_EventLastCmdDone,
	TXMASK = -1,
};

static rfc_CMD_BLE_ADV_t RF_ble_cmdBleAdv = {
	.commandNo = CMD_BLE_ADV,
	.pParams = & bleAdvPar,
	.pOutput = & adv_output,
	.startTrigger = {
		.triggerType = TRIG_ABSTIME,
		.pastTrig = true,
	},
	.condition = {
		.rule = COND_NEVER,
	},
};

static struct pdu_adv_connect_ind connect_req;

/* Runs the transmitting part of the test application and returns a result. */
TestResult tx_runTxTest(const ApplicationConfig* config)
{
    menu_clear();

    if( ARRAY_SIZE( ble_adv_channels ) * BLE_ADV_SLOT_TIME_MIN >= BLE_ADV_INTERVAL ) {
    	printk(
			"\x1b[%d;%df"
			"%u * slot time (%u us) >= advertising interval (%u us)\n",
			13, 1,
			(unsigned)ARRAY_SIZE( ble_adv_channels ),
			BLE_ADV_SLOT_TIME_MIN_US,
			BLE_ADV_INTERVAL_US
		);
    	while(1);
    }

    if(config == NULL)
    {
        while(1);
    }
    memcpy((void *)&localConfig, config, sizeof(ApplicationConfig));

	origin = RF_getCurrentTime();
	D( "Theoretical values for 1MHz phy.." );
    D( "BLE_ADV_TIME_MIN_US: %u us", BLE_ADV_TIME_MIN_US );
    D( "BLE_SCAN_REQ_TIME_MIN_US: %u us", BLE_SCAN_REQ_TIME_MIN_US );
    D( "BLE_SCAN_RSP_TIME_MIN_US: %u us", BLE_SCAN_RSP_TIME_MIN_US );
    D( "BLE_CONNECT_TIME_MIN_US: %u us", BLE_CONNECT_TIME_MIN_US );
    D( "BLE_ADV_SLOT_TIME_MIN_US: %u us", BLE_ADV_SLOT_TIME_MIN_US );

    RF_Params rfParams;
    RF_Params_init(&rfParams);

    if( RFQueue_defineQueue(&rxDataQueue,
                            rxDataEntryBuffer,
                            sizeof(rxDataEntryBuffer),
                            NUM_DATA_ENTRIES,
                            MAX_LENGTH + NUM_APPENDED_BYTES))
    {
        /* Failed to allocate space for all data entries */
        while(true);
    }

    const FrequencyTableEntry *fte = chan_to_fte[ ble_adv_channels[ 0 ] ];

    RF_ble_cmdBle5AdvAux.pParams->pAdvPkt = (u8_t *)&ble5ExtAdvPacket;
    ble5ExtAdvPacket.extHdrInfo.length = EXTENDED_HEADER_LENGTH;
    ble5ExtAdvPacket.advDataLen = sizeof( adv_data );
    ble5ExtAdvPacket.pAdvData = (u8_t *)adv_data;
    RF_ble_cmdBle5AdvAux.startTrigger.triggerType  = TRIG_NOW;
    RF_ble_cmdBle5AdvAux.startTrigger.pastTrig = 1;
    RF_ble_cmdBle5AdvAux.channel = 0xFF;
    RF_ble_cmdBle5AdvAux.whitening.bOverride = 1;
    RF_ble_cmdBle5AdvAux.whitening.init = config->frequencyTable[config->frequency].whitening;
    RF_ble_cmdBle5AdvAux.startTime = 0;

    bleAdvPar.pDeviceAddress = (u16_t *)device_address;
    bleAdvPar.pRxQ = & rxDataQueue;
    bleAdvPar.pAdvData = (u8_t *)adv_data;
    bleAdvPar.advLen = sizeof( adv_data );
    bleAdvPar.pScanRspData = (u8_t *)scan_rsp_data;
    bleAdvPar.scanRspLen = sizeof( scan_rsp_data );

    D( "opening RF object" );
    /* Request access to the radio based on test case*/
	rfHandle = RF_open(&rfObject, &RF_modeBle, (RF_RadioSetup*)&RF_ble_cmdRadioSetup, &rfParams);
	packetInterval = (u32_t)(RF_convertMsToRatTicks(PKT_INTERVAL_MS_BLE)); // Set packet interval to 100 ms

	RF_ble_cmdFs.frequency = fte->frequency;
	RF_ble_cmdFs.fractFreq = fte->fractFreq;
	D( "Setting frequency" );
	RF_runCmd(rfHandle, (RF_Op*)&RF_ble_cmdFs, RF_PriorityNormal, NULL, 0);

	/*
	RF_setTxPower(
		rfHandle,
		RF_TxPowerTable_findValue(
			(RF_TxPowerTable_Entry *)RF_BLE_txPowerTable,
			RF_TxPowerTable_MAX_DBM
		)
	);
	*/

	TestResult r = TestResult_Finished;

    //menu_updateTxMetricScreen( (tx_metrics *) & adv_output );

	bool once = false;

	D( "Entering loop.." );
    for( ;; ) {

        /* Check, whether a button has been pressed */
        if (menu_isButtonPressed()) {
            /* If there is an ongoing Tx command, cancel it */
            (void)RF_cancelCmd(rfHandle, current_rf_cmd_handle, ABORT_GRACEFUL);
            RF_pendCmd(rfHandle, current_rf_cmd_handle, (RF_EventCmdCancelled | RF_EventCmdStopped | RF_EventCmdAborted));
            RF_close(rfHandle);

            r = TestResult_Aborted;

            current_rf_cmd_handle = -1;
            break;
        }

	    /* Get current time */
        ratmr_t now = RF_getCurrentTime();

        if ( false ) {
        } else if ( BLE_STATE_ADV_STBY == ble_adv_state ) {

        	ble_adv_state = BLE_STATE_ADV_ADVT;

        } else if ( BLE_STATE_ADV_ADVT == ble_adv_state ) {

            //menu_updateTxMetricScreen( (tx_metrics *) & adv_output );

    		if ( now >= ble_adv_interval_end_time ) {

    			ble_adv_interval_start_time = now;
    			ble_adv_interval_end_time = now + BLE_ADV_INTERVAL;

        		for( u8_t i = 0; i < ARRAY_SIZE( ble_adv_channels ); ++i ) {
    				while( -1 != current_rf_cmd_handle );
        			ble_adv_events_notified = 0;
    				RF_ble_cmdBleAdv.startTime = now;
    				RF_ble_cmdBleAdv.channel = ble_adv_channels[ i ];
    				if ( ! once ) {
    					D( "TX'ing first packet" );
    					once = true;
    				}
    				ble_adv_cmd_start = RF_getCurrentTime();
    				current_rf_cmd_handle = RF_postCmd( rfHandle, (RF_Op*) & RF_ble_cmdBleAdv, RF_PriorityNormal, &tx_callback, TXMASK );
        		}
    		}

        } else if ( BLE_STATE_ADV_CONN == ble_adv_state ) {

        	//menu_clear();
        	break;
        }
    }
    while( -1 != current_rf_cmd_handle );

    ble_adv_state = BLE_STATE_ADV_STBY;

    memset( & connect_req, 0, sizeof( connect_req ) );
    memset( & adv_output, 0, sizeof( adv_output ) );

    RF_close(rfHandle);
    return r;
}

static void describe_rf_event( RF_EventMask e, RF_EventMask already, u32_t usec, u8_t chan ) {

	if ( ( e & RF_EventCmdDone ) && ! ( already & RF_EventCmdDone ) ) D( "Ch%u A radio operation command in a chain finished. (%u us)", chan, usec );
	if ( ( e & RF_EventLastCmdDone ) && ! ( already & RF_EventLastCmdDone ) ) D( "Ch%u The last radio operation command in a chain finished. (%u us)", chan, usec );
	if ( ( e & RF_EventFGCmdDone ) && ! ( already & RF_EventFGCmdDone ) ) D( "Ch%u A IEEE-mode radio operation command in a chain finished. (%u us)", chan, usec );
	if ( ( e & RF_EventLastFGCmdDone ) && ! ( already & RF_EventLastFGCmdDone ) ) D( "Ch%u A stand-alone IEEE-mode radio operation command or the last command in a chain finished. (%u us)", chan, usec );
	if ( ( e & RF_EventTxDone ) && ! ( already & RF_EventTxDone ) ) D( "Ch%u Packet transmitted (%u us)", chan, usec );
	if ( ( e & RF_EventTXAck ) && ! ( already & RF_EventTXAck ) ) D( "Ch%u ACK packet transmitted (%u us)", chan, usec );
	if ( ( e & RF_EventTxCtrl ) && ! ( already & RF_EventTxCtrl ) ) D( "Ch%u Control packet transmitted (%u us)", chan, usec );
	if ( ( e & RF_EventTxCtrlAck ) && ! ( already & RF_EventTxCtrlAck ) ) D( "Ch%u Acknowledgement received on a transmitted control packet (%u us)", chan, usec );
	if ( ( e & RF_EventTxCtrlAckAck ) && ! ( already & RF_EventTxCtrlAckAck ) ) D( "Ch%u Acknowledgement received on a transmitted control packet, and acknowledgement transmitted for that packet (%u us)", chan, usec );
	if ( ( e & RF_EventTxRetrans ) && ! ( already & RF_EventTxRetrans ) ) D( "Ch%u Packet retransmitted (%u us)", chan, usec );
	if ( ( e & RF_EventTxEntryDone ) && ! ( already & RF_EventTxEntryDone ) ) D( "Ch%u Tx queue data entry state changed to Finished (%u us)", chan, usec );
	if ( ( e & RF_EventTxBufferChange ) && ! ( already & RF_EventTxBufferChange ) ) D( "Ch%u A buffer change is complete (%u us)", chan, usec );
	if ( ( e & RF_EventPaChanged ) && ! ( already & RF_EventPaChanged ) ) D( "Ch%u The PA was reconfigured on the fly. (%u us)", chan, usec );
	if ( ( e & RF_EventRxOk ) && ! ( already & RF_EventRxOk ) ) D( "Ch%u Packet received with CRC OK, payload, and not to be ignored (%u us)", chan, usec );
	if ( ( e & RF_EventRxNOk ) && ! ( already & RF_EventRxNOk ) ) D( "Ch%u Packet received with CRC error (%u us)", chan, usec );
	if ( ( e & RF_EventRxIgnored ) && ! ( already & RF_EventRxIgnored ) ) D( "Ch%u Packet received with CRC OK, but to be ignored (%u us)", chan, usec );
	if ( ( e & RF_EventRxEmpty ) && ! ( already & RF_EventRxEmpty ) ) D( "Ch%u Packet received with CRC OK, not to be ignored, no payload (%u us)", chan, usec );
	if ( ( e & RF_EventRxCtrl ) && ! ( already & RF_EventRxCtrl ) ) D( "Ch%u Control packet received with CRC OK, not to be ignored (%u us)", chan, usec );
	if ( ( e & RF_EventRxCtrlAck ) && ! ( already & RF_EventRxCtrlAck ) ) D( "Ch%u Control packet received with CRC OK, not to be ignored, then ACK sent (%u us)", chan, usec );
	if ( ( e & RF_EventRxBufFull ) && ! ( already & RF_EventRxBufFull ) ) D( "Ch%u Packet received that did not fit in the Rx queue (%u us)", chan, usec );
	if ( ( e & RF_EventRxEntryDone ) && ! ( already & RF_EventRxEntryDone ) ) D( "Ch%u Rx queue data entry changing state to Finished (%u us)", chan, usec );
	if ( ( e & RF_EventDataWritten ) && ! ( already & RF_EventDataWritten ) ) D( "Ch%u Data written to partial read Rx buffer (%u us)", chan, usec );
	if ( ( e & RF_EventNDataWritten ) && ! ( already & RF_EventNDataWritten ) ) D( "Ch%u Specified number of bytes written to partial read Rx buffer (%u us)", chan, usec );
	if ( ( e & RF_EventRxAborted ) && ! ( already & RF_EventRxAborted ) ) D( "Ch%u Packet reception stopped before packet was done (%u us)", chan, usec );
	if ( ( e & RF_EventRxCollisionDetected ) && ! ( already & RF_EventRxCollisionDetected ) ) D( "Ch%u A collision was indicated during packet reception (%u us)", chan, usec );
	if ( ( e & RF_EventModulesUnlocked ) && ! ( already & RF_EventModulesUnlocked ) ) D( "Ch%u As part of the boot process, the CM0 has opened access to RF core modules and memories (%u us)", chan, usec );
	if ( ( e & RF_EventInternalError ) && ! ( already & RF_EventInternalError ) ) D( "Ch%u Internal error observed (%u us)", chan, usec );
	if ( ( e & RF_EventMdmSoft ) && ! ( already & RF_EventMdmSoft ) ) D( "Ch%u Synchronization word detected (MDMSOFT interrupt flag) (%u us)", chan, usec );
	if ( ( e & RF_EventCmdCancelled ) && ! ( already & RF_EventCmdCancelled ) ) D( "Ch%u Command canceled before it was started. (%u us)", chan, usec );
	if ( ( e & RF_EventCmdAborted ) && ! ( already & RF_EventCmdAborted ) ) D( "Ch%u Abrupt command termination caused by RF_cancelCmd() or RF_flushCmd(). (%u us)", chan, usec );
	if ( ( e & RF_EventCmdStopped ) && ! ( already & RF_EventCmdStopped ) ) D( "Ch%u Graceful command termination caused by RF_cancelCmd() or RF_flushCmd(). (%u us)", chan, usec );
	if ( ( e & RF_EventRatCh ) && ! ( already & RF_EventRatCh ) ) D( "Ch%u A user-programmable RAT channel triggered an event. (%u us)", chan, usec );
	//if ( ( e & RF_EventPowerUp ) && ! ( already & RF_EventPowerUp ) ) D( "Ch%u RF power up event. deprecated This event is deprecated. Use RF_ClientEventPowerUpFinished instead. (%u us)", chan, usec );
	if ( ( e & RF_EventError ) && ! ( already & RF_EventError ) ) D( "Ch%u Event flag used for error callback functions to indicate an error. See RF_Params::pErrCb. (%u us)", chan, usec );
	if ( ( e & RF_EventCmdPreempted ) && ! ( already & RF_EventCmdPreempted ) ) D( "Ch%u Command preempted by another command with higher priority. Applies only to multi-client applications. (%u us)", chan, usec );

}

// used in menu-driven UI to display Adv stats
RF_EventMask ble_adv_event_mask;

void tx_callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
	static u8_t lastNRxScanReq;

	u32_t then;
	u32_t now;
	u32_t delta;
	u32_t sec;
	u32_t usec;

	// used in menu-driven UI to display Adv stats
	ble_adv_event_mask = e;

	then = ble_adv_cmd_start;
	now = RF_getCurrentTime();
	delta = now - then;
	delta = RF_convertRatTicksToUs( delta );
	split_sec_usec( delta, & sec, & usec );

	rfc_bleRadioOp_t *op = (rfc_bleRadioOp_t *) RF_getCmdOp( h, ch );

	describe_rf_event( e, ble_adv_events_notified, usec, op->channel );
	ble_adv_events_notified |= e;

	if ( e & RF_EventLastCmdDone ) {
		current_rf_cmd_handle = -1;
		ble_adv_cmd_end = now;
	}

	if ( e & RF_EventRxEntryDone ) {
		for( ;; ) {
			rfc_dataEntryGeneral_t *it = RFQueue_getDataEntry();

			u8_t packet_length = it->length;
			// for some reason, the actual data is always preceeded by the same value in it->status
			u8_t *packet_data = &(& it->data)[ 1 ];

			if ( false ) {
			} else if ( BLE_STATE_ADV_ADVT == ble_adv_state ) {

				const u8_t invalid_channel_map[ 5 ] = {0};

				if (
					true
					&& adv_output.nRxConnectReq > 0
					&& packet_length >= sizeof( connect_req )
					&& 0 != memcmp(
								invalid_channel_map,
								& packet_data[ offsetof( struct pdu_adv_connect_ind, chan_map ) ],
								sizeof( invalid_channel_map )
							)
				) {

					memcpy( & connect_req, packet_data, MIN( packet_length, sizeof( connect_req ) ) );
					ble_adv_state = BLE_STATE_ADV_CONN;

		        	struct pdu_adv_connect_ind *x = & connect_req;
		        	D( "Ch%u received a CONNECT_REQ:\n\t"
		        		"init_addr:   %02x:%02x:%02x:%02x:%02x:%02x\n\t"
		        		"adv_addr:    %02x:%02x:%02x:%02x:%02x:%02x\n\t"
		        		"access_addr: %02x:%02x:%02x:%02x\n\t"
		        		"crc_init:    %02x:%02x:%02x\n\t"
		        		"win_size:    %u\n\t"
		        		"win_offset:  %u\n\t"
		        		"interval:    %u\n\t"
		        		"latency:     %u\n\t"
		        		"timeout:     %u\n\t"
		        		"chan_map:    %02x%02x%02x%02x%02x\n\t"
		        		"hop:         %u\n\t"
		        		"sca:         %u"
		        		,
						op->channel,
						x->init_addr[ 5 ], x->init_addr[ 4 ], x->init_addr[ 3 ], x->init_addr[ 2 ], x->init_addr[ 1 ], x->init_addr[ 0 ],
						x->adv_addr[ 5 ], x->adv_addr[ 4 ], x->adv_addr[ 3 ], x->adv_addr[ 2 ], x->adv_addr[ 1 ], x->adv_addr[ 0 ],
						x->access_addr[ 3 ], x->access_addr[ 2 ], x->access_addr[ 1 ], x->access_addr[ 0 ],
						x->crc_init[ 2 ], x->crc_init[ 1 ], x->crc_init[ 0 ],
						x->win_size,
						x->win_offset,
						x->interval,
						x->latency,
						x->timeout,
						x->chan_map[ 4 ], x->chan_map[ 3 ], x->chan_map[ 2 ], x->chan_map[ 1 ], x->chan_map[ 0 ],
						x->hop,
						x->sca
					);

					break;
				}

				if ( adv_output.nRxScanReq != lastNRxScanReq ) {

					lastNRxScanReq = adv_output.nRxScanReq;

					struct pdu_adv_scan_req *x = (struct pdu_adv_scan_req *) packet_data;

					D(
						"Ch%u received a SCAN_REQ: "
						"scan_addr: %02x:%02x:%02x:%02x:%02x:%02x "
						"adv_addr: %02x:%02x:%02x:%02x:%02x:%02x"
						,
						op->channel,
						x->scan_addr[ 5 ], x->scan_addr[ 4 ], x->scan_addr[ 3 ], x->scan_addr[ 2 ], x->scan_addr[ 1 ], x->scan_addr[ 0 ],
						x->adv_addr[ 5 ], x->adv_addr[ 4 ], x->adv_addr[ 3 ], x->adv_addr[ 2 ], x->adv_addr[ 1 ], x->adv_addr[ 0 ]
					);
				}

			}

			u8_t rx_status = RFQueue_nextEntry();
			if ( DATA_ENTRY_PENDING == rx_status ) {
				break;
			}
		}
	}
}

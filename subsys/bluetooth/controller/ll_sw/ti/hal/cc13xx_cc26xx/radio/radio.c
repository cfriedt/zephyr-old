/*
 * Copyright (c) 2016 - 2019 Nordic Semiconductor ASA
 * Copyright (c) 2016 Vinayak Kariappa Chettimada
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sys/dlist.h>
#include <sys/mempool_base.h>
#include <toolchain.h>

#include "util/mem.h"
#include "hal/ccm.h"
#include "hal/radio.h"
#include "hal/ticker.h"
#include "ll_sw/pdu.h"


#include <sys/dlist.h>
#include <sys/mempool_base.h>
#include <toolchain.h>
#include <bluetooth/gap.h>

#define DeviceFamily_CC13X2
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/power/PowerCC26X2.h>
#include <driverlib/rf_data_entry.h>
#include <driverlib/aon_rtc.h>
#include <driverlib/osc.h>
#include <driverlib/prcm.h>
#include <driverlib/rf_mailbox.h>
#include <driverlib/rf_common_cmd.h>

#include <driverlib/rf_ble_mailbox.h>
#include <driverlib/rfc.h>
#include <rf_patches/rf_patch_cpe_bt5.h>
#include <inc/hw_ccfg.h>
#include <inc/hw_fcfg1.h>

#include "irq.h"
#include "hal/cntr.h"
#include "hal/ticker.h"
#include "hal/swi.h"

#define BT_DBG_ENABLED 1
#define LOG_MODULE_NAME bt_ctlr_hal_ti_radio
#include "common/log.h"
#include "hal/debug.h"

#define RADIO_PDU_LEN_MAX (BIT(8) - 1)

/* Modifying these values could open a portal to The Upside-down */
#define CC13XX_CC26XX_INCLUDE_LEN_BYTE 1
#define CC13XX_CC26XX_INCLUDE_CRC 1
#define CC13XX_CC26XX_APPEND_RSSI 1
#define CC13XX_CC26XX_APPEND_TIMESTAMP 4
#define CC13XX_CC26XX_ADDITIONAL_DATA_BYTES                                    \
	(0 + CC13XX_CC26XX_INCLUDE_LEN_BYTE + CC13XX_CC26XX_INCLUDE_CRC +      \
	 CC13XX_CC26XX_APPEND_RSSI + CC13XX_CC26XX_APPEND_TIMESTAMP)

#define CC13XX_CC26XX_NUM_RX_BUF 2
#define CC13XX_CC26XX_RX_BUF_SIZE                                              \
	(RADIO_PDU_LEN_MAX + CC13XX_CC26XX_ADDITIONAL_DATA_BYTES)

#define CC13XX_CC26XX_NUM_TX_BUF 2
#define CC13XX_CC26XX_TX_BUF_SIZE RADIO_PDU_LEN_MAX

struct ble_cc13xx_cc26xx_data {

	RF_Params rfParams;
	RF_Object rfObject;
	RF_Handle rfHandle;
	RF_EventMask rfStatus;
	RF_Mode RF_modeBle;

	u16_t device_address[3];
	u32_t access_address;
	u32_t polynomial;
	u32_t iv;
	u8_t whiten;
	u16_t chan;
	u16_t max_len;
#if IS_ENABLED(CONFIG_BT_CTLR_ADV_EXT)
	u8_t adv_data[sizeof(
		((struct pdu_adv_com_ext_adv *)0)->ext_hdr_adi_adv_data)];
#else
	u8_t adv_data[sizeof((struct pdu_adv_adv_ind *)0)->data];
#endif
	u8_t adv_data_len;

#if IS_ENABLED(CONFIG_BT_CTLR_ADV_EXT)
	u8_t scan_rsp_data[sizeof(
		((struct pdu_adv_com_ext_adv *)0)->ext_hdr_adi_adv_data)];
#else
	u8_t scan_rsp_data[sizeof((struct pdu_adv_scan_rsp *)0)->data];
#endif
	u8_t scan_rsp_data_len;

	RF_EventMask rx_mask;

	dataQueue_t rx_queue;
	rfc_dataEntryPointer_t rx_entry[CC13XX_CC26XX_NUM_RX_BUF];
	u8_t rx_data[CC13XX_CC26XX_NUM_RX_BUF]
		    [CC13XX_CC26XX_RX_BUF_SIZE] __aligned(4);

	dataQueue_t tx_queue;
	rfc_dataEntryPointer_t tx_entry[CC13XX_CC26XX_NUM_TX_BUF];
	u8_t tx_data[CC13XX_CC26XX_NUM_TX_BUF]
		    [CC13XX_CC26XX_TX_BUF_SIZE] __aligned(4);

	RF_CmdHandle active_command_handle;

	RF_RatConfigCompare rat_hcto_compare;
	RF_RatHandle rat_hcto_handle;

#if defined(CONFIG_BT_CTLR_DEBUG_PINS)
	u32_t window_begin_ticks;
	u32_t window_duration_ticks;
	u32_t window_interval_ticks;
#endif /* defined(CONFIG_BT_CTLR_DEBUG_PINS) */

	bool ignore_next_rx;
	bool ignore_next_tx;

	rfc_CMD_GET_FW_INFO_t cmd_get_fw_info;

	rfc_CMD_BLE5_RADIO_SETUP_t cmd_ble_radio_setup;

	rfc_CMD_FS_t cmd_set_frequency;

	rfc_CMD_BLE_ADV_t cmd_ble_adv;
	rfc_bleAdvPar_t cmd_ble_adv_param;
	rfc_bleAdvOutput_t cmd_ble_adv_output;

	rfc_CMD_BLE_GENERIC_RX_t cmd_ble_generic_rx;
	rfc_bleGenericRxPar_t cmd_ble_generic_rx_param;
	rfc_bleGenericRxOutput_t cmd_ble_generic_rx_output;

	rfc_CMD_NOP_t cmd_nop;
	rfc_CMD_CLEAR_RX_t cmd_clear_rx;
	rfc_CMD_BLE_ADV_PAYLOAD_t cmd_ble_adv_payload;

	rfc_CMD_BLE_SLAVE_t cmd_ble_slave;
	rfc_bleSlavePar_t cmd_ble_slave_param;
	rfc_bleMasterSlaveOutput_t cmd_ble_slave_output;
};

/* Contains a pre-defined setting for frequency selection */
struct FrequencyTableEntry {
	const char *const label;
	const u16_t frequency;
	const u16_t fractFreq;
	const u8_t whitening;
};

static radio_isr_cb_t isr_cb;
static void           *isr_cb_param;

void isr_radio(void)
{
	BT_DBG("");
	if (radio_has_disabled()) {
		isr_cb(isr_cb_param);
	}
}

void radio_isr_set(radio_isr_cb_t cb, void *param)
{
	BT_DBG("");
	irq_disable(LL_RADIO_IRQn);

	isr_cb_param = param;
	isr_cb = cb;

	/* Clear pending interrupts */
	ClearPendingIRQ(LL_RADIO_IRQn);

	irq_enable(LL_RADIO_IRQn);
}

/* Overrides for CMD_BLE5_RADIO_SETUP */
static u32_t pOverridesCommon[] = {
#if defined(CONFIG_BT_CTLR_DEBUG_PINS)
	/* See http://bit.ly/2vydFIa */
	(u32_t)0x008F88B3,
	HW_REG_OVERRIDE(
		0x1110,
		0
		| RFC_DBELL_SYSGPOCTL_GPOCTL0_RATGPO1 /* RX */
		| RFC_DBELL_SYSGPOCTL_GPOCTL1_CPEGPO1 /* PA (default setting) */
		| RFC_DBELL_SYSGPOCTL_GPOCTL2_RATGPO2 /* BLE TX Window */
		| RFC_DBELL_SYSGPOCTL_GPOCTL3_RATGPO0 /* TX */
	),
#endif /* defined(CONFIG_BT_CTLR_DEBUG_PINS) */
	(u32_t)0x00F388D3,
	/* Bluetooth 5: Set pilot tone length to 20 us Common */
	HW_REG_OVERRIDE(0x6024, 0x2E20),
	/* Bluetooth 5: Compensate for reduced pilot tone length */
	(uint32_t)0x01280263,
	/* Bluetooth 5: Default to no CTE. */
	HW_REG_OVERRIDE(0x5328, 0x0000), (u32_t)0xFFFFFFFF
};

/* Overrides for CMD_BLE5_RADIO_SETUP */
static u32_t pOverrides1Mbps[] = {
	/* Bluetooth 5: Set pilot tone length to 20 us */
	HW_REG_OVERRIDE(0x5320, 0x03C0),
	/* Bluetooth 5: Compensate syncTimeadjust */
	(u32_t)0x015302A3, (u32_t)0xFFFFFFFF
};

/* Overrides for CMD_BLE5_RADIO_SETUP */
static u32_t pOverrides2Mbps[] = {
	/* Bluetooth 5: Set pilot tone length to 20 us */
	HW_REG_OVERRIDE(0x5320, 0x03C0),
	/* Bluetooth 5: Compensate syncTimeAdjust */
	(u32_t)0x00F102A3, (u32_t)0xFFFFFFFF
};

/* Overrides for CMD_BLE5_RADIO_SETUP */
static u32_t pOverridesCoded[] = {
	/* Bluetooth 5: Set pilot tone length to 20 us */
	HW_REG_OVERRIDE(0x5320, 0x03C0),
	/* Bluetooth 5: Compensate syncTimeadjust */
	(u32_t)0x07A902A3,
	/* Rx: Set AGC reference level to 0x1B (default: 0x2E) */
	HW_REG_OVERRIDE(0x609C, 0x001B), (u32_t)0xFFFFFFFF
};

static struct ble_cc13xx_cc26xx_data ble_cc13xx_cc26xx_data = {

	.RF_modeBle = {
		.rfMode = RF_MODE_BLE,
		.cpePatchFxn = &rf_patch_cpe_bt5,
		.mcePatchFxn = 0,
		.rfePatchFxn = 0,
	},

	/* clang-format off */
	.cmd_set_frequency = {
		.commandNo = CMD_FS,
		.startTrigger = {
			.triggerType = TRIG_NOW,
		},
		.condition = {
			.rule = COND_NEVER,
		},
	},

	.cmd_ble_radio_setup = {
		.commandNo = CMD_BLE5_RADIO_SETUP,
		.startTrigger = {
			.triggerType = TRIG_NOW,
		},
		.condition = {
			.rule = COND_NEVER,
		},
		.config = {
			.biasMode = 0x1,
		},
		.txPower = 0x7217,
		.pRegOverrideCommon = pOverridesCommon,
		.pRegOverride1Mbps = pOverrides1Mbps,
		.pRegOverride2Mbps = pOverrides2Mbps,
		.pRegOverrideCoded = pOverridesCoded,
	},

	.cmd_ble_adv = {
		.pParams = (rfc_bleAdvPar_t *)&ble_cc13xx_cc26xx_data
				   .cmd_ble_adv_param,
		.pOutput = (rfc_bleAdvOutput_t *)&ble_cc13xx_cc26xx_data
				   .cmd_ble_adv_output,
		.condition = {
			.rule = TRIG_NEVER,
		},
	},

	.cmd_ble_adv_param = {
		.pRxQ = &ble_cc13xx_cc26xx_data.rx_queue,
		.rxConfig = {
			.bAutoFlushIgnored = true,
			.bAutoFlushCrcErr = true,
			/* SCAN_REQ will be discarded if true! */
			.bAutoFlushEmpty = false,
			.bIncludeLenByte = !!CC13XX_CC26XX_INCLUDE_LEN_BYTE,
			.bIncludeCrc = !!CC13XX_CC26XX_INCLUDE_CRC,
			.bAppendRssi = !!CC13XX_CC26XX_APPEND_RSSI,
			.bAppendTimestamp = !!CC13XX_CC26XX_APPEND_TIMESTAMP,
		},
		.advConfig = {
			/* Support Channel Selection Algorithm #2 */
			.chSel = IS_ENABLED(CONFIG_BT_CTLR_CHAN_SEL_2),
		},
		.endTrigger = {
			.triggerType = TRIG_NEVER,
		},
	},

	.cmd_ble_generic_rx = {
		.commandNo = CMD_BLE_GENERIC_RX,
		.condition = {
			.rule = COND_NEVER,
		},
		.pParams = (rfc_bleGenericRxPar_t *)&ble_cc13xx_cc26xx_data
			.cmd_ble_generic_rx_param,
		.pOutput = (rfc_bleGenericRxOutput_t *)&ble_cc13xx_cc26xx_data
			.cmd_ble_generic_rx_output,
	},

	.cmd_ble_generic_rx_param = {
		.pRxQ = &ble_cc13xx_cc26xx_data.rx_queue,
		.rxConfig = {
			.bAutoFlushIgnored = true,
			.bAutoFlushCrcErr = true,
			/* SCAN_REQ will be discarded if true! */
			.bAutoFlushEmpty = false,
			.bIncludeLenByte = !!CC13XX_CC26XX_INCLUDE_LEN_BYTE,
			.bIncludeCrc = !!CC13XX_CC26XX_INCLUDE_CRC,
			.bAppendRssi = !!CC13XX_CC26XX_APPEND_RSSI,
			.bAppendTimestamp = !!CC13XX_CC26XX_APPEND_TIMESTAMP,
		},
		.endTrigger = {
			.triggerType = TRIG_NEVER,
		},
	},

	.cmd_nop = {
		.commandNo = CMD_NOP,
		.condition = {
			.rule = COND_NEVER,
		},
	},

	.cmd_clear_rx = {
		.commandNo = CMD_CLEAR_RX,
		.pQueue = &ble_cc13xx_cc26xx_data.rx_queue,
	},
	.cmd_ble_adv_payload = {
		.commandNo = CMD_BLE_ADV_PAYLOAD,
		.pParams = &ble_cc13xx_cc26xx_data.cmd_ble_adv_param,
	},

	.cmd_ble_slave = {
		.commandNo = CMD_BLE_SLAVE,
		.pParams = &ble_cc13xx_cc26xx_data.cmd_ble_slave_param,
		.pOutput = &ble_cc13xx_cc26xx_data.cmd_ble_slave_output,
		.condition = {
			.rule = COND_NEVER,
		},
	},

	.cmd_ble_slave_param = {
		.pRxQ = &ble_cc13xx_cc26xx_data.rx_queue,
		.pTxQ = &ble_cc13xx_cc26xx_data.tx_queue,
		.rxConfig = {
			.bAutoFlushIgnored = true,
			.bAutoFlushCrcErr = true,
			.bAutoFlushEmpty = true,
			.bIncludeLenByte = !!CC13XX_CC26XX_INCLUDE_LEN_BYTE,
			.bIncludeCrc = !!CC13XX_CC26XX_INCLUDE_CRC,
			.bAppendRssi = !!CC13XX_CC26XX_APPEND_RSSI,
			.bAppendTimestamp = !!CC13XX_CC26XX_APPEND_TIMESTAMP,
		},
		.seqStat = {
			.bFirstPkt = true,
		},
		.timeoutTrigger = {
			.triggerType = TRIG_REL_START,
		},
		.endTrigger = {
			.triggerType = TRIG_NEVER,
		},
	},
	/* clang-format on */
};
static struct ble_cc13xx_cc26xx_data *drv_data = &ble_cc13xx_cc26xx_data;

/* specific for cc1352r1 */
static const struct FrequencyTableEntry config_frequencyTable_ble[] = {
	/* clang-format off */
	{ "2402  ", 0x0962, 0x0000, 0x65 },
	{ "2404  ", 0x0964, 0x0000, 0x40 },
	{ "2406  ", 0x0966, 0x0000, 0x41 },
	{ "2408  ", 0x0968, 0x0000, 0x42 },
	{ "2410  ", 0x096a, 0x0000, 0x43 },
	{ "2412  ", 0x096c, 0x0000, 0x44 },
	{ "2414  ", 0x096e, 0x0000, 0x45 },
	{ "2416  ", 0x0970, 0x0000, 0x46 },
	{ "2418  ", 0x0972, 0x0000, 0x47 },
	{ "2420  ", 0x0974, 0x0000, 0x48 },
	{ "2422  ", 0x0976, 0x0000, 0x49 },
	{ "2424  ", 0x0978, 0x0000, 0x4a },
	{ "2426  ", 0x097a, 0x0000, 0x66 },
	{ "2428  ", 0x097c, 0x0000, 0x4b },
	{ "2430  ", 0x097e, 0x0000, 0x4c },
	{ "2432  ", 0x0980, 0x0000, 0x4d },
	{ "2434  ", 0x0982, 0x0000, 0x4e },
	{ "2436  ", 0x0984, 0x0000, 0x4f },
	{ "2438  ", 0x0986, 0x0000, 0x50 },
	{ "2440  ", 0x0988, 0x0000, 0x51 },
	{ "2442  ", 0x098a, 0x0000, 0x52 },
	{ "2444  ", 0x098c, 0x0000, 0x53 },
	{ "2446  ", 0x098e, 0x0000, 0x54 },
	{ "2448  ", 0x0990, 0x0000, 0x55 },
	{ "2480  ", 0x0992, 0x0000, 0x56 },
	{ "2452  ", 0x0994, 0x0000, 0x57 },
	{ "2454  ", 0x0996, 0x0000, 0x58 },
	{ "2456  ", 0x0998, 0x0000, 0x59 },
	{ "2458  ", 0x099a, 0x0000, 0x5a },
	{ "2460  ", 0x099c, 0x0000, 0x5b },
	{ "2462  ", 0x099e, 0x0000, 0x5c },
	{ "2464  ", 0x09a0, 0x0000, 0x5d },
	{ "2466  ", 0x09a2, 0x0000, 0x5e },
	{ "2468  ", 0x09a4, 0x0000, 0x5f },
	{ "2470  ", 0x09a6, 0x0000, 0x60 },
	{ "2472  ", 0x09a8, 0x0000, 0x61 },
	{ "2474  ", 0x09aa, 0x0000, 0x62 },
	{ "2476  ", 0x09ac, 0x0000, 0x63 },
	{ "2478  ", 0x09ae, 0x0000, 0x64 },
	{ "2480  ", 0x09b0, 0x0000, 0x67 },
	/* clang-format on */
};

static const RF_TxPowerTable_Entry RF_BLE_txPowerTable[] = {
	{ -20, RF_TxPowerTable_DEFAULT_PA_ENTRY(6, 3, 0, 2) },
	{ -15, RF_TxPowerTable_DEFAULT_PA_ENTRY(10, 3, 0, 3) },
	{ -10, RF_TxPowerTable_DEFAULT_PA_ENTRY(15, 3, 0, 5) },
	{ -5, RF_TxPowerTable_DEFAULT_PA_ENTRY(22, 3, 0, 9) },
	{ 0, RF_TxPowerTable_DEFAULT_PA_ENTRY(19, 1, 0, 20) },
	{ 1, RF_TxPowerTable_DEFAULT_PA_ENTRY(22, 1, 0, 20) },
	{ 2, RF_TxPowerTable_DEFAULT_PA_ENTRY(25, 1, 0, 25) },
	{ 3, RF_TxPowerTable_DEFAULT_PA_ENTRY(29, 1, 0, 28) },
	{ 4, RF_TxPowerTable_DEFAULT_PA_ENTRY(35, 1, 0, 39) },
	{ 5, RF_TxPowerTable_DEFAULT_PA_ENTRY(23, 0, 0, 57) },
	RF_TxPowerTable_TERMINATION_ENTRY
};

static void rx_queue_reset(void) {
	BT_DBG("");
	/* Setup circular RX queue (TRM 25.3.2.7) */
	memset(&drv_data->rx_entry[0], 0, sizeof(drv_data->rx_entry[0]));
	memset(&drv_data->rx_entry[1], 0, sizeof(drv_data->rx_entry[1]));

	drv_data->rx_entry[0].pNextEntry = (u8_t *)&drv_data->rx_entry[1];
	drv_data->rx_entry[0].config.type = DATA_ENTRY_TYPE_PTR;
	drv_data->rx_entry[0].config.lenSz = 1;
	drv_data->rx_entry[0].status = DATA_ENTRY_PENDING;
	drv_data->rx_entry[0].length = sizeof(drv_data->rx_data[0]);
	drv_data->rx_entry[0].pData = drv_data->rx_data[0];

	drv_data->rx_entry[1].pNextEntry = (u8_t *)&drv_data->rx_entry[0];
	drv_data->rx_entry[1].config.type = DATA_ENTRY_TYPE_PTR;
	drv_data->rx_entry[1].config.lenSz = 1;
	drv_data->rx_entry[0].status = DATA_ENTRY_PENDING;
	drv_data->rx_entry[1].length = sizeof(drv_data->rx_data[1]);
	drv_data->rx_entry[1].pData = drv_data->rx_data[1];

	drv_data->rx_queue.pCurrEntry = (u8_t *)&drv_data->rx_entry[0];
	drv_data->rx_queue.pLastEntry = NULL;
}

static void tx_queue_reset(void) {
	BT_DBG("");
	/* Setup circular TX queue (TRM 25.3.2.7) */
	memset(&drv_data->tx_entry[0], 0, sizeof(drv_data->tx_entry[0]));
	memset(&drv_data->tx_entry[1], 0, sizeof(drv_data->tx_entry[1]));

	drv_data->tx_entry[0].pNextEntry = (u8_t *)&drv_data->tx_entry[1];
	drv_data->tx_entry[0].config.type = DATA_ENTRY_TYPE_PTR;
	drv_data->tx_entry[0].config.lenSz = 0;
	drv_data->tx_entry[0].config.irqIntv = 0;
	drv_data->tx_entry[0].status = DATA_ENTRY_FINISHED;
	drv_data->tx_entry[0].length = 0;
	drv_data->tx_entry[0].pData = drv_data->tx_data[0];

	drv_data->tx_entry[1].pNextEntry = (u8_t *)&drv_data->tx_entry[0];
	drv_data->tx_entry[1].config.type = DATA_ENTRY_TYPE_PTR;
	drv_data->tx_entry[1].config.lenSz = 1;
	drv_data->tx_entry[0].status = DATA_ENTRY_FINISHED;
	drv_data->tx_entry[1].length = sizeof(drv_data->tx_data[1]);
	drv_data->tx_entry[1].pData = drv_data->tx_data[1];

	drv_data->tx_queue.pCurrEntry = (u8_t *)&drv_data->tx_entry[0];
	drv_data->tx_queue.pLastEntry = NULL;
}

static void ble_cc13xx_cc26xx_data_init(void)
{
	BT_DBG("");
	RF_Params_init( & drv_data->rfParams );

	/* FIXME: this duplicates code in controller/hci/hci.c
	 * There must be a way to programmatically query the device address
	 */
	u8_t *mac;

	if (sys_read32(CCFG_BASE + CCFG_O_IEEE_BLE_0) != 0xFFFFFFFF &&
	    sys_read32(CCFG_BASE + CCFG_O_IEEE_BLE_1) != 0xFFFFFFFF) {
		mac = (u8_t *)(CCFG_BASE + CCFG_O_IEEE_BLE_0);
	} else {
		mac = (u8_t *)(FCFG1_BASE + FCFG1_O_MAC_BLE_0);
	}

	memcpy(drv_data->device_address, mac, sizeof(drv_data->device_address));
	mac = (u8_t *)drv_data->device_address;
	/* BT_ADDR_SET_STATIC(a) */
	mac[5] |= 0xc0;
	drv_data->cmd_ble_adv_param.pDeviceAddress = (u16_t *)mac;
	BT_DBG("device address: %02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1],
	       mac[2], mac[3], mac[4], mac[5]);
	/* Ensure that this address is marked as _random_ */
	drv_data->cmd_ble_adv_param.advConfig.deviceAddrType = 1;

	rx_queue_reset();
	tx_queue_reset();

	drv_data->active_command_handle = -1;

//	RF_RatConfigCompare_init(
//		(RF_RatConfigCompare *)&drv_data->rat_hcto_compare);
//	drv_data->rat_hcto_compare.callback = rat_deferred_hcto_callback;
}

static void radio_setup_completion(RF_Handle h, RF_CmdHandle ch, RF_EventMask e) {
	(void) h;
	(void) ch;
	(void) e;
	//BT_DBG("");
	isr_radio();
}

void radio_setup(void)
{
	BT_DBG("");
	ble_cc13xx_cc26xx_data_init();

	drv_data->rfHandle = RF_open( & drv_data->rfObject, & drv_data->RF_modeBle,
			   (RF_RadioSetup *)& drv_data->cmd_ble_radio_setup,
			   & drv_data->rfParams );

	LL_ASSERT( drv_data->rfHandle );

	/* For subsequent BLE commands switch to the correct channels */
	struct FrequencyTableEntry *fte =
		(struct FrequencyTableEntry *)&config_frequencyTable_ble[0];
	drv_data->cmd_set_frequency.frequency = fte->frequency;
	drv_data->cmd_set_frequency.fractFreq = fte->fractFreq;
	drv_data->whiten = fte->whitening;
	drv_data->chan = 37;

	RF_runCmd( drv_data->rfHandle, (RF_Op *)&drv_data->cmd_set_frequency,
		  RF_PriorityNormal, radio_setup_completion, RF_EventLastCmdDone );
}

void radio_reset(void)
{
	BT_DBG("");
	irq_disable(LL_RADIO_IRQn);
}

void radio_phy_set(u8_t phy, u8_t flags)
{
}

void radio_tx_power_set(u32_t power)
{
	BT_DBG("");
	RF_setTxPower( drv_data->rfHandle, RF_TxPowerTable_findValue( (RF_TxPowerTable_Entry *)RF_BLE_txPowerTable, power ) );
}

void radio_tx_power_max_set(void)
{
	BT_DBG("");
	RF_setTxPower( drv_data->rfHandle, RF_TxPowerTable_findValue( (RF_TxPowerTable_Entry *)RF_BLE_txPowerTable, RF_TxPowerTable_MAX_DBM) );
}

s8_t radio_tx_power_min_get(void)
{
	BT_DBG("");
	return RF_TxPowerTable_MIN_DBM;
}

s8_t radio_tx_power_max_get(void)
{
	BT_DBG("");
	return RF_TxPowerTable_MAX_DBM;
}

s8_t radio_tx_power_floor(s8_t power)
{
	BT_DBG("");
	return 0;
}

void radio_freq_chan_set(u32_t chan)
{
	BT_DBG("");
	/*
	 * The LLL expects the channel number to be computed as
	 * 2400 + chan [MHz]. Therefore a compensation of -2 MHz
	 * has been provided.
	 */
	LL_ASSERT(2 <= chan && chan <= 80);
	LL_ASSERT(!(chan & 0x1));

	unsigned int idx = chan - 2;
	struct FrequencyTableEntry *fte =
		(struct FrequencyTableEntry *)&config_frequencyTable_ble[idx];
	drv_data->whiten = fte->whitening;

	switch (chan) {
	case 2:
		drv_data->chan = 37;
		break;
	case 4 ... 24:
		drv_data->chan = (chan - 4) / 2;
		break;
	case 26:
		drv_data->chan = 38;
		break;
	case 28 ... 78:
		drv_data->chan = (chan - 6) / 2;
		break;
	case 80:
		drv_data->chan = 39;
		break;
	}
}

void radio_whiten_iv_set(u32_t iv)
{
	BT_DBG("iv: %u", iv);
}

void radio_aa_set(u8_t *aa)
{
	BT_DBG("");
	drv_data->access_address = *(u32_t *)aa;
}

void radio_pkt_configure(u8_t bits_len, u8_t max_len, u8_t flags)
{
	BT_DBG("");
	drv_data->max_len = max_len;
}

void radio_pkt_rx_set(void *rx_packet)
{
	BT_DBG("");
	drv_data->rx_entry[ 0 ].pData = rx_packet;
}

void radio_pkt_tx_set(void *tx_packet)
{
	BT_DBG("");
	drv_data->tx_entry[ 0 ].pData = tx_packet;
}

u32_t radio_tx_ready_delay_get(u8_t phy, u8_t flags)
{
	BT_DBG("");
	return 0;
	//return hal_radio_tx_ready_delay_us_get(phy, flags);
}

u32_t radio_tx_chain_delay_get(u8_t phy, u8_t flags)
{
	BT_DBG("");
	return 0;
	//return hal_radio_tx_chain_delay_us_get(phy, flags);
}

u32_t radio_rx_ready_delay_get(u8_t phy, u8_t flags)
{
	BT_DBG("");
	return 0;
	//return hal_radio_rx_ready_delay_us_get(phy, flags);
}

u32_t radio_rx_chain_delay_get(u8_t phy, u8_t flags)
{
	BT_DBG("");
	return 0;
	//return hal_radio_rx_chain_delay_us_get(phy, flags);
}

void radio_rx_enable(void)
{
	BT_DBG("");
	//nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_RXEN);
}

void radio_tx_enable(void)
{
	BT_DBG("");
	//nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_TXEN);
}

static void radio_disable_completion(RF_Handle h, RF_CmdHandle ch, RF_EventMask e) {
	(void) h;
	(void) ch;
	(void) e;
	//BT_DBG("");
	isr_radio();
}

void radio_disable(void)
{
	BT_DBG("");
	//BT_DBG("now: %u", cntr_cnt_get());
	/*
	 * 0b1011..Abort All - Cancels all pending events and abort any
	 * sequence-in-progress
	 */
	RFCDoorbellSendTo(CMDR_DIR_CMD(CMD_ABORT));

	/* Set all RX entries to empty */
	RFCDoorbellSendTo((u32_t)&drv_data->cmd_clear_rx);

	/* generate interrupt to get into isr_radio */
	RF_postCmd( drv_data->rfHandle, (RF_Op *)&drv_data->cmd_nop, RF_PriorityNormal,
		   radio_disable_completion, RF_EventLastCmdDone);

	//next_radio_cmd = NULL;
}

void radio_status_reset(void)
{
	BT_DBG("");
	// FIXME: determine the precise definition of "status"
	rx_queue_reset();
	tx_queue_reset();
	// NRF_RADIO->EVENTS_READY = 0;
	// NRF_RADIO->EVENTS_END = 0;
	// NRF_RADIO->EVENTS_DISABLED = 0;
}

u32_t radio_is_ready(void)
{
	BT_DBG("");
	// FIXME: determine the precise definition of "ready"
	return 0;
	// return (NRF_RADIO->EVENTS_READY != 0);
}

u32_t radio_is_done(void)
{
	BT_DBG("");
	return drv_data->rfStatus & RF_EventLastCmdDone;
}

u32_t radio_has_disabled(void)
{
	BT_DBG("");
	return 0;
	//return (NRF_RADIO->EVENTS_DISABLED != 0);
}

u32_t radio_is_idle(void)
{
	BT_DBG("");
	return 1;
	//return (NRF_RADIO->STATE == 0);
}

void radio_crc_configure(u32_t polynomial, u32_t iv)
{
	BT_DBG("");
	drv_data->polynomial = polynomial;
	drv_data->iv = iv;
}

u32_t radio_crc_is_valid(void)
{
	BT_DBG("");
	return !( drv_data->rfStatus & RF_EventRxNOk );
}

static u8_t MALIGN(4) _pkt_empty[PDU_EM_SIZE_MAX];
static u8_t MALIGN(4) _pkt_scratch[
			((RADIO_PDU_LEN_MAX + 3) > PDU_AC_SIZE_MAX) ?
			(RADIO_PDU_LEN_MAX + 3) : PDU_AC_SIZE_MAX];

void *radio_pkt_empty_get(void)
{
	BT_DBG("");
	return _pkt_empty;
}

void *radio_pkt_scratch_get(void)
{
	BT_DBG("");
	return _pkt_scratch;
}

void radio_switch_complete_and_rx(u8_t phy_rx)
{
	BT_DBG("");
}

void radio_switch_complete_and_tx(u8_t phy_rx, u8_t flags_rx, u8_t phy_tx,
				  u8_t flags_tx)
{
	BT_DBG("");
}

void radio_switch_complete_and_disable(void)
{
	BT_DBG("");
	while( ! radio_is_done() );
	radio_disable();
}

void radio_rssi_measure(void)
{
	BT_DBG("");
}

u32_t radio_rssi_get(void)
{
	BT_DBG("");
	return RF_getRssi( drv_data->rfHandle );
}

void radio_rssi_status_reset(void)
{
	BT_DBG("");
}

u32_t radio_rssi_is_ready(void)
{
	BT_DBG("");
	return RF_GET_RSSI_ERROR_VAL != RF_getRssi( drv_data->rfHandle );
}

void radio_filter_configure(u8_t bitmask_enable, u8_t bitmask_addr_type, u8_t *bdaddr)
{
	BT_DBG("");
}

void radio_filter_disable(void)
{
	BT_DBG("");
}

void radio_filter_status_reset(void)
{
	BT_DBG("");
}

u32_t radio_filter_has_match(void)
{
	BT_DBG("");
	return 0;
}

u32_t radio_filter_match_get(void)
{
	BT_DBG("");
	return 0;
}

void radio_bc_configure(u32_t n)
{
	BT_DBG("");
}

void radio_bc_status_reset(void)
{
	BT_DBG("");
}

u32_t radio_bc_has_match(void)
{
	BT_DBG("");
	return 0;
}

void radio_tmr_status_reset(void)
{
	BT_DBG("");
}

void radio_tmr_tifs_set(u32_t tifs)
{
	// should be possible according to the TRM, but it is not obvious how

	/*
	 * 25.8.4 Parameter Override
	 * Several parameters that are not configurable through the parameter structures can still be overridden by
	 * using the CMD_BLE5_RADIO_SETUP, CMD_RADIO_SETUP, CMD_UPDATE_RADIO_SETUP, or
	 * CMD_WRITE_FWPAR commands.
	 * The parameters that can be overridden in Bluetooth low energy mode are described in the Bluetooth
	 * Specification documents listed in Related Documentation. The parameters can, for example, be used to
	 * improve performance or to create noncompliant behavior that may be useful in some cases. Examples
	 * follow:
	 * - Modification of T_IFS and other timing
	 * ...
	 */
}

static void cmd_ble_adv_completion(RF_Handle h, RF_CmdHandle ch, RF_EventMask e) {
	//BT_DBG("");
	isr_radio();
}

static void pkt_tx( u8_t trx, u32_t ticks_start, u32_t remainder ) {
	(void) trx;

	BT_DBG("");

	// assume peripheral role
	if ( PDU_AC_ACCESS_ADDR == drv_data->access_address ) {

		drv_data->cmd_ble_adv.startTrigger.triggerType = TRIG_ABSTIME;
		drv_data->cmd_ble_adv.startTime = ticks_start;

		RF_postCmd( drv_data->rfHandle, (RF_Op *) & drv_data->cmd_ble_adv,
				   RF_PriorityNormal, cmd_ble_adv_completion, RF_EventLastCmdDone);

	} else {
	}
}

u32_t radio_tmr_start(u8_t trx, u32_t ticks_start, u32_t remainder)
{
	u32_t now = cntr_cnt_get();
	BT_DBG("now: %u trx: %u ticks_start: %u remainder: %u", now, trx, ticks_start, remainder);

	if ( trx ) {
		pkt_tx( trx, ticks_start, remainder );
	} else {
		//pkt_rx( ticks_start, remainder );
		LL_ASSERT(0 == 1);
	}

	return 0;
}

u32_t radio_tmr_start_tick(u8_t trx, u32_t tick)
{
	u32_t now = cntr_cnt_get();
	BT_DBG("now: %u trx: %u tick: %u", now, trx, tick);
	return 0;
}

void radio_tmr_start_us(u8_t trx, u32_t us)
{
	u32_t now = cntr_cnt_get();
	BT_DBG("now: %u trx: %u us: %u", now, trx, us);
}

u32_t radio_tmr_start_now(u8_t trx)
{
	u32_t now = cntr_cnt_get();
	BT_DBG("now: %u trx: %u", now, trx);
	return 0;
}

u32_t radio_tmr_start_get(void)
{
	BT_DBG("");
	return 0;
}

void radio_tmr_stop(void)
{
	BT_DBG("");
}

void radio_tmr_hcto_configure(u32_t hcto)
{
	BT_DBG("");
}

void radio_tmr_aa_capture(void)
{
	BT_DBG("");
}

u32_t radio_tmr_aa_get(void)
{
	BT_DBG("");
	return 0;
}

void radio_tmr_aa_save(u32_t aa)
{
	BT_DBG("");
}

u32_t radio_tmr_aa_restore(void)
{
	BT_DBG("");
	return 0;
}

u32_t radio_tmr_ready_get(void)
{
	BT_DBG("");
	return 0;
}

void radio_tmr_end_capture(void)
{
	BT_DBG("");
}

u32_t radio_tmr_end_get(void)
{
	BT_DBG("");
	return 0;
}

u32_t radio_tmr_tifs_base_get(void)
{
	BT_DBG("");
	return radio_tmr_end_get() + 0 /* meh */;
}

void radio_tmr_sample(void)
{
	BT_DBG("");
}

u32_t radio_tmr_sample_get(void)
{
	BT_DBG("");
	return 0;
}

void *radio_ccm_rx_pkt_set(struct ccm *ccm, u8_t phy, void *pkt)
{
	BT_DBG("");
	return NULL;
}

void *radio_ccm_tx_pkt_set(struct ccm *ccm, void *pkt)
{
	BT_DBG("");
	return NULL;
}

u32_t radio_ccm_is_done(void)
{
	BT_DBG("");
	return false;
}

u32_t radio_ccm_mic_is_valid(void)
{
	BT_DBG("");
	return false;
}

void radio_ar_configure(u32_t nirk, void *irk)
{
	BT_DBG("");
}

u32_t radio_ar_match_get(void)
{
	BT_DBG("");
	return 0;
}

void radio_ar_status_reset(void)
{
	BT_DBG("");
}

u32_t radio_ar_has_match(void)
{
	BT_DBG("");
	return false;
}

static void update_adv_data(u8_t *data, u8_t len, bool scan_rsp)
{
	BT_DBG("");
	drv_data->cmd_ble_adv_payload.payloadType = scan_rsp;

	if (NULL == data || 0 == len) {
		len = 0;
	}

	drv_data->cmd_ble_adv_payload.newLen =
		MIN(len, scan_rsp ? sizeof(drv_data->scan_rsp_data) :
				    sizeof(drv_data->adv_data));
	drv_data->cmd_ble_adv_payload.pNewData = data;

	RFCDoorbellSendTo((u32_t)&drv_data->cmd_ble_adv_payload);
}

void radio_set_scan_rsp_data(u8_t *data, u8_t len)
{
	BT_DBG("");
	update_adv_data(data, len, true);
}

void radio_slave_reset(void) {

	BT_DBG("");

	/*
	 *  BLE Core Spec 5.1: 4.5.5 Connection setup - Slave Role
	 *
	 *  The first packet received, regardless of a valid CRC match (i.e. only
	 *  the access code matches), in the Connection State by the slave
	 *  determines anchor point for the first connection event, and therefore
	 *  the timings of all future connection events in this connection.
	 */
	drv_data->cmd_ble_slave_param.rxConfig.bAutoFlushCrcErr = false;

	/* See TRM 25.8.5: Link Layer Connection
	 *
	 * Before the first operation on a connection, the bits in pParams->seqStat
	 * shall be set as follows by the system CPU:
	 */
	drv_data->cmd_ble_slave_param.seqStat.lastRxSn = 1;
	drv_data->cmd_ble_slave_param.seqStat.lastTxSn = 1;
	drv_data->cmd_ble_slave_param.seqStat.nextTxSn = 0;
	drv_data->cmd_ble_slave_param.seqStat.bFirstPkt = true;
	drv_data->cmd_ble_slave_param.seqStat.bAutoEmpty = true;
	drv_data->cmd_ble_slave_param.seqStat.bLlCtrlTx = false;
	drv_data->cmd_ble_slave_param.seqStat.bLlCtrlAckRx = false;
	drv_data->cmd_ble_slave_param.seqStat.bLlCtrlAckPending = false;
}

const PowerCC26X2_Config PowerCC26X2_config = {
	.policyInitFxn = NULL,
	.policyFxn = &PowerCC26XX_doWFI,
	.calibrateFxn = &PowerCC26XX_calibrate,
	.enablePolicy = false,
	.calibrateRCOSC_LF = false,
	.calibrateRCOSC_HF = false,
};

void radio_set_up_slave_cmd(void) {
	BT_DBG("");
}

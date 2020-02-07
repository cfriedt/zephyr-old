#define DeviceFamily_CC13X2
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <driverlib/ioc.h>

#include "smartrf_settings.h"

static RF_Object rfObject;
static RF_Handle rfHandle;
static RF_RatHandle ratHandle;
static RF_RatConfigCompare config;
static RF_RatConfigOutput output;

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

PIN_Config pinTable[] =
{
    IOID_24 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

#define D(fmt, args...) printk( fmt "\n", ##args )

static void describe_rf_event( const RF_EventMask e ) {

	if ( e & RF_EventCmdDone ) D( "A radio operation command in a chain finished.");
	if ( e & RF_EventLastCmdDone ) D( "The last radio operation command in a chain finished.");
	if ( e & RF_EventFGCmdDone ) D( "A IEEE-mode radio operation command in a chain finished.");
	if ( e & RF_EventLastFGCmdDone ) D( "A stand-alone IEEE-mode radio operation command or the last command in a chain finished.");
	if ( e & RF_EventTxDone ) D( "Packet transmitted");
	if ( e & RF_EventTXAck ) D( "ACK packet transmitted");
	if ( e & RF_EventTxCtrl ) D( "Control packet transmitted");
	if ( e & RF_EventTxCtrlAck ) D( "Acknowledgement received on a transmitted control packet");
	if ( e & RF_EventTxCtrlAckAck ) D( "Acknowledgement received on a transmitted control packet, and acknowledgement transmitted for that packet");
	if ( e & RF_EventTxRetrans ) D( "Packet retransmitted");
	if ( e & RF_EventTxEntryDone ) D( "Tx queue data entry state changed to Finished");
	if ( e & RF_EventTxBufferChange ) D( "A buffer change is complete");
	if ( e & RF_EventPaChanged ) D( "The PA was reconfigured on the fly.");
	if ( e & RF_EventRxOk ) D( "Packet received with CRC OK, payload, and not to be ignored");
	if ( e & RF_EventRxNOk ) D( "Packet received with CRC error");
	if ( e & RF_EventRxIgnored ) D( "Packet received with CRC OK, but to be ignored");
	if ( e & RF_EventRxEmpty ) D( "Packet received with CRC OK, not to be ignored, no payload");
	if ( e & RF_EventRxCtrl ) D( "Control packet received with CRC OK, not to be ignored");
	if ( e & RF_EventRxCtrlAck ) D( "Control packet received with CRC OK, not to be ignored, then ACK sent");
	if ( e & RF_EventRxBufFull ) D( "Packet received that did not fit in the Rx queue");
	if ( e & RF_EventRxEntryDone ) D( "Rx queue data entry changing state to Finished");
	if ( e & RF_EventDataWritten ) D( "Data written to partial read Rx buffer");
	if ( e & RF_EventNDataWritten ) D( "Specified number of bytes written to partial read Rx buffer");
	if ( e & RF_EventRxAborted ) D( "Packet reception stopped before packet was done");
	if ( e & RF_EventRxCollisionDetected ) D( "A collision was indicated during packet reception");
	if ( e & RF_EventModulesUnlocked ) D( "As part of the boot process, the CM0 has opened access to RF core modules and memories");
	if ( e & RF_EventInternalError ) D( "Internal error observed");
	if ( e & RF_EventMdmSoft ) D( "Synchronization word detected (MDMSOFT interrupt flag)");
	if ( e & RF_EventCmdCancelled ) D( "Command canceled before it was started.");
	if ( e & RF_EventCmdAborted ) D( "Abrupt command termination caused by RF_cancelCmd() or RF_flushCmd().");
	if ( e & RF_EventCmdStopped ) D( "Graceful command termination caused by RF_cancelCmd() or RF_flushCmd().");
	if ( e & RF_EventRatCh ) D( "A user-programmable RAT channel triggered an event.");
	//if ( e & RF_EventPowerUp ) D( "RF power up event. deprecated This event is deprecated. Use RF_ClientEventPowerUpFinished instead.");
	if ( e & RF_EventError ) D( "Event flag used for error callback functions to indicate an error. See RF_Params::pErrCb.");
	if ( e & RF_EventCmdPreempted ) D( "Command preempted by another command with higher priority. Applies only to multi-client applications.");

}

void onRatTriggered(RF_Handle h, RF_RatHandle rh, RF_EventMask e, uint32_t compareCaptureTime)
{
    describe_rf_event( e );
    if (e & RF_EventError)
    {
        while(1);
    }
    else
    {
        // Trigger precisely with the same period again
        config.timeout = compareCaptureTime + RF_convertMsToRatTicks(5);
        ratHandle = RF_ratCompare(rfHandle, &config, &output);
    }
}

const PINCC26XX_HWAttrs PINCC26XX_hwAttrs = {
    .intPriority = ~0,
    .swiPriority = 0
};

int main(int argc, char **argv)
{
	(void)argc;
	(void)argv;

	PIN_Status ps = PIN_init(pinTable);
	if (PIN_SUCCESS != ps) {
		while(1);
	}

    RF_Params rfParams;
    RF_Params_init(&rfParams);

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, pinTable);
    if (ledPinHandle == NULL)
    {
        while(1);
    }

    PINCC26XX_setMux(ledPinHandle, IOID_24, PINCC26XX_MUX_RFC_GPO2);
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);

    RF_RatConfigOutput_init(&output);
    output.mode = RF_RatOutputModeToggle;
    output.select = RF_RatOutputSelectRatGpo2;

    RF_RatConfigCompare_init(&config);
    config.callback = &onRatTriggered;
    config.channel = RF_RatChannel2;
    config.timeout = RF_getCurrentTime() + RF_convertMsToRatTicks(5);
    ratHandle = RF_ratCompare(rfHandle, &config, &output);

    while(1);
}

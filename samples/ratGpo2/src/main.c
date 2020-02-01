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
    IOID_7 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

void onRatTriggered(RF_Handle h, RF_RatHandle rh, RF_EventMask e, uint32_t compareCaptureTime)
{
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

    PINCC26XX_setMux(ledPinHandle, IOID_7, PINCC26XX_MUX_RFC_GPO);
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);

    RF_RatConfigOutput_init(&output);
    output.mode = RF_RatOutputModePulse;
    output.select = RF_RatOutputSelectRatGpo2;

    RF_RatConfigCompare_init(&config);
    config.callback = &onRatTriggered;
    config.channel = RF_RatChannel2;
    config.timeout = RF_getCurrentTime() + RF_convertMsToRatTicks(5);
    ratHandle = RF_ratCompare(rfHandle, &config, &output);

    while(1);
}

#include <stdbool.h>
#include <stddef.h>
#include <ti/drivers/Power.h>

#ifdef CONFIG_HAS_CC13X2_CC26X2_SDK
#include <ti/drivers/power/PowerCC26X2.h>

const PowerCC26X2_Config PowerCC26X2_config = {
    .policyInitFxn      = NULL,
    .policyFxn          = NULL,
    .calibrateFxn       = &PowerCC26XX_calibrate,
    .enablePolicy       = false,
    .calibrateRCOSC_LF  = true,
    .calibrateRCOSC_HF  = true,
};
#endif /* CONFIG_HAS_CC13X2_CC26X2_SDK */

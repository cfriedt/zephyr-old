#ifndef SMARTRF_SETTINGS_H_
#define SMARTRF_SETTINGS_H_

#define DeviceFamily_CC13X2
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/rf_mailbox.h)
#include DeviceFamily_constructPath(driverlib/rf_common_cmd.h)
#include DeviceFamily_constructPath(driverlib/rf_prop_cmd.h)
#include <ti/drivers/rf/RF.h>
#include DeviceFamily_constructPath(rf_patches/rf_patch_cpe_prop.h)

#include <inc/hw_rfc_dbell.h>

extern RF_Mode RF_prop;
extern rfc_CMD_PROP_RADIO_DIV_SETUP_t RF_cmdPropRadioDivSetup;

#endif /* SMARTRF_SETTINGS_H_ */

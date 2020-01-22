/*
 * Copyright (c) 2016 Nordic Semiconductor ASA
 * Copyright (c) 2016 Vinayak Kariappa Chettimada
 * Copyright 2019 NXP
 * Copyright (c) 2013-2019, Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sys/dlist.h>
#include <sys/mempool_base.h>

#include "hal/cntr.h"
#include "swi.h"

#define LOG_MODULE_NAME bt_ctlr_ti_cntr
#include "common/log.h"
#include "hal/debug.h"

#define DeviceFamily_CC13X2
#include <ti/drivers/rf/RF.h>
#include <driverlib/aon_rtc.h>
#include <driverlib/aon_event.h>
#include <driverlib/interrupt.h>

void cntr_init(void)
{
	irq_disable(LL_RTC0_IRQn);

	AONEventMcuWakeUpSet(AON_EVENT_MCU_WU0, AON_EVENT_RTC0);
	AONRTCCombinedEventConfig(AON_RTC_CH0 | AON_RTC_CH1);

	irq_enable(LL_RTC0_IRQn);
}

static u8_t refcount;

u32_t cntr_start(void)
{
	if (refcount++) {
		return 1;
	}

	AONRTCChannelEnable(AON_RTC_CH1);

	return 0;
}

u32_t cntr_stop(void)
{
	LL_ASSERT(refcount);

	if (--refcount) {
		return 1;
	}

	AONRTCChannelDisable(AON_RTC_CH1);

	return 0;
}

u32_t cntr_cnt_get(void)
{
	return AONRTCCurrentCompareValueGet();
}

void cntr_cmp_set(u8_t cmp, u32_t value)
{
	ARG_UNUSED(cmp);

	AONRTCChannelDisable(AON_RTC_CH1);
	AONRTCEventClear(AON_RTC_CH1);
	AONRTCModeCh1Set(AON_RTC_MODE_CH1_COMPARE);
	AONRTCCompareValueSet(AON_RTC_CH1, value);
	AONRTCChannelEnable(AON_RTC_CH1);
}

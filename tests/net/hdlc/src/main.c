/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <net/hdlc.h>

#include "hdlc_test.h"

#include <ztest.h>

/* context tests */
extern void test_init_16(void);
extern void test_init_32(void);

/* rx state tests */
extern void test_state_start_16(void);
extern void test_state_start_32(void);
extern void test_state_data_16(void);
extern void test_state_data_32(void);
extern void test_state_esc_16(void);
extern void test_state_esc_32(void);

/* rx tests */
extern void test_rx_forty_two_16(void);
extern void test_rx_forty_two_32(void);
extern void test_rx_hello_world_16(void);
extern void test_rx_hello_world_32(void);

/* tx tests */
extern void test_tx_forty_two_16(void);
extern void test_tx_forty_two_32(void);
extern void test_tx_hello_world_16(void);
extern void test_tx_hello_world_32(void);

void test_main(void)
{
	ztest_test_suite(test_hdlc,
		ztest_unit_test(test_init_16),
		ztest_unit_test(test_init_32),
		ztest_unit_test(test_state_start_16),
		ztest_unit_test(test_state_start_32),
		ztest_unit_test(test_state_data_16),
		ztest_unit_test(test_state_data_32),
		ztest_unit_test(test_state_esc_16),
		ztest_unit_test(test_state_esc_32),
		ztest_unit_test(test_rx_forty_two_16),
		ztest_unit_test(test_rx_forty_two_32),
		ztest_unit_test(test_rx_hello_world_16),
		ztest_unit_test(test_rx_hello_world_32),
		ztest_unit_test(test_tx_forty_two_16),
		ztest_unit_test(test_tx_forty_two_32),
		ztest_unit_test(test_tx_hello_world_16),
		ztest_unit_test(test_tx_hello_world_32));
	ztest_run_test_suite(test_hdlc);
}

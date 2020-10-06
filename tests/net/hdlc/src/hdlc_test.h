/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef HDLC_TEST_H_
#define HDLC_TEST_H_

#include <stdint.h>

#include <net/hdlc.h>

#define HELLO_WORLD_MSG "Hello, HDLC world!"

struct buf {
    uint8_t offs;
    uint8_t data[32];
};

int rx(void *arg, uint8_t byte);
int tx(void *arg, uint8_t byte);
int erase(void *arg, uint16_t n);

extern struct buf rx_buf;
extern struct buf tx_buf;

extern const uint8_t forty_two_frame_16[7];
extern const uint8_t forty_two_frame_32[9];

extern const uint8_t hello_world_frame_16[24];
extern const uint8_t hello_world_frame_32[26];

void get_ctx16(struct hdlc_context *ctx);
void get_ctx32(struct hdlc_context *ctx);

#endif /* HDLC_TEST_H_ */

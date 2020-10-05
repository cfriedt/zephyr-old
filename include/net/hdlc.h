/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief High-Level Data Link Control
 *
 * Support for encoding and decoding HDLC frames.
 * 
 * @see <a href="https://en.wikipedia.org/wiki/High-Level_Data_Link_Control">High-Level Data Link Control</a>
 * @see <a href="https://tools.ietf.org/html/rfc1662">RFC 1662</a>
 */

#ifndef ZEPHYR_INCLUDE_NET_HDLC_H_
#define ZEPHYR_INCLUDE_NET_HDLC_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief High-Level Data Link Control (HDLC) library
 * @defgroup net_hdlc Network Core Library
 * @ingroup networking
 * @{
 */

/** HDLC L2 driver API. Used by PPP */

/** HDLC Receive states */
enum hdlc_state {
	/** Awaiting @ref HDLC_FLAG */
	HDLC_STATE_START,
	/** Receiving data */
	HDLC_STATE_DATA,
	/** Receiving data that has been escaped with @ref HDLC_ESC */
	HDLC_STATE_ESC,
};

/** HDLC special values */
enum {
	/** Start- and End-of-Frame indicator */
	HDLC_FLAG = 0x7e,
	/** Escape indicator */
	HDLC_ESC = 0x7d,
	/** Value XOR'ed with bytes that must be escaped */
	HDLC_XOR = 0x20,
};

/** HDLC Frame Check Sequence values */
enum {
	/** Initializer for 16-bit FCS */
	HDLC_FCS16_INIT = 0xffff,
	/** Check value for 16-bit FCS */
	HDLC_FCS16_GOOD = 0xf0b8,
	/** Initializer for 32-bit FCS */
	HDLC_FCS32_INIT = 0xffffffff,
	/** Check value for 32-bit FCS */
	HDLC_FCS32_GOOD = 0xdebb20e3,
};

/**
 * HDLC receive handler
 *
 * This function type is used to receive a byte into an application-specific
 * buffer.
 * 
 * @param arg application-specific argument (e.g. the buffer)
 * @param byte the byte to be received
 * @return 0 on success
 * @return otherwise, a negative errno value
 */
typedef int (*hdlc_rx)(void *arg, uint8_t byte);

/**
 * HDLC transmit handler
 * 
 * This function type is used to transmit a byte via an application-specific
 * channel.
 *
 * @param arg application-specific argument (e.g. the buffer)
 * @param byte the byte to send
 * @return 0 on success
 * @return otherwise, a negative errno value
 */
typedef int (*hdlc_tx)(void *arg, uint8_t byte);

/**
 * HDLC erase handler
 *
 * This function type is used to erase data from the application-specific
 * buffer.
 * 
 * It is normally called when a corrupt or invalid data is received and
 * either @ref HDLC_FCS16_GOOD or @ref HDLC_FCS32_GOOD is not obtained
 * upon frame completion. 
 *
 * @param arg application-specific argument (e.g. the buffer)
 * @param n the number of bytes to erase
 * @return 0 on success
 * @return otherwise, a negative errno value
 */
typedef int (*hdlc_erase)(void *arg, uint16_t n);

/**
 * HDLC context
 *
 * @a fcs32 indicates if the context should be using a 32-bit or 16-bit Frame Check Sequence
 * @a fcs is the current value of the frame check sequence
 * @a state contains the current HDLC receive state
 * @a frame_len is the current length of the HDLC frame
 * @a rx the receive handler
 * @a rx_arg argument to the receive handler
 * @a tx the transmit handler
 * @a tx_arg argument to the transmit handler
 * @a erase the erase handler
 * @a erase_arg argument to the erase handler
 */
struct hdlc_context {
	bool fcs32;
	uint32_t fcs;
	enum hdlc_state state;
	uint16_t frame_len;
	hdlc_rx rx;
	void *rx_arg;
	hdlc_tx tx;
	void *tx_arg;
	hdlc_erase erase;
	void *erase_arg;
};

/**
 * Declare and initialize an HDLC context at compile-time
 *
 * Note: due to a quirk in @ref crc32_ieee_update, the
 * 32-bit FCS must be inverted here. 
 * 
 * @code{.c}
 * #include <net/hdlc.h>
 *
 * struct my_struct {
 *     uint8_t buffer[42];
 * };
 *
 * static struct my_struct my_input;
 * static struct my_struct my_output;
 *
 * static int my_rx_handler(void *arg, uint8_t byte);
 * static int my_tx_handler(void *arg, uint8_t byte);
 * static int my_erase_handler(void *arg, uint16_t n);
 *
 * static DECLARE_HDLC_CONTEXT( \
 *     my_hdlc, \
 *     false, \
 *     my_rx_handler, \
 *     &my_input, \
 *     my_tx_handler, \
 *     &my_output, \
 *     my_erase_handler, \
 *     &my_input);
 * @endcode
 */
#define DECLARE_HDLC_CONTEXT(name, fcs32, rx, rx_arg, tx, tx_arg, erase, erase_arg) \
	struct hdlc_context name = { \
		fcs32, fcs32 ? ~HDLC_FCS32_INIT : HDLC_FCS16_INIT, HDLC_STATE_START, 0, rx, rx_arg, tx, tx_arg, erase, erase_arg}

/**
 * Check if a byte is reserved for HDLC usage
 *
 * @return true if @p byte is reserved
 * @return false, otherwise
 */
static inline bool hdlc_is_reserved(uint8_t byte)
{
	return (byte == HDLC_FLAG) || (byte == HDLC_ESC);
}

/**
 * Check if an HDLC context is valid
 *
 * @param hdlc_context the HDLC context to check
 * @return true if the HDLC context is valid
 * @return false if the HDLC context is invalid
 */
bool hdlc_context_is_valid(struct hdlc_context *hdlc_context);

/**
 * Initialize an HDLC context
 *
 * @param hdlc_context the HDLC context to initialize
 * @param fcs32 use a 32-bit CRC instead of a 16-bit CRC
 * @param rx application-specific receive handler
 * @param rx_arg argument for application-specific receive handler (e.g. buffer)
 * @param erase application-specific erase handler
 * @param erase_arg argument for application-specific erase handler (e.g. buffer)
 * @param tx application-specific send handler
 * @param tx_arg argument for application-specific send handler (e.g. uart)
 * @return 0 on success
 * @return a negative errno value on failure
 */
int hdlc_context_init(struct hdlc_context *hdlc_context, bool fcs32, hdlc_rx rx,
		      void *rx_arg, hdlc_tx tx, void *tx_arg, hdlc_erase erase, void *erase_arg);

/**
 * Reset an HDLC context back to its initial state
 * 
 * If @p hdlc_context has an @ref hdlc_context.erase handler, then
 * the handler is invoked prior to resetting internal fields in
 * order to discard unused data in the receive buffer.
 * 
 * Otherwise, this function is equivalent to re-initializing
 * @p hdlc_context using @ref hdlc_context_init with the same
 * initial arguments.
 *
 * @param hdlc_context the HDLC context to reset
 */
void hdlc_context_reset(struct hdlc_context *hdlc_context);

/**
 * Process a byte using HDLC framing rules
 *
 * If an error occurs (e.g. unable to receive a byte to the application-specific buffer), then the receive buffer is eraseed and bytes are discarded until the next @ref HDLC_FLAG is received.
 *
 * @param hdlc_context the HDLC context to use
 * @param byte the byte to process
 * @return 0 when a byte is processed normally
 * @return > 0 when an HDLC frame is available
 * @return a negative errno value on error
 */
int hdlc_process_byte(struct hdlc_context *hdlc_context, uint8_t byte);

/**
 * Write a buffer for the specific HDLC address
 *
 * @param hdlc_context the HDLC context to use
 * @param addr the address to use
 * @param addr_size the size (in bytes) of @p addr
 * @param ctrl the control bytes to use
 * @param ctrl_size the size (in bytes) of @p ctrl
 * @param data the data to ransit
 * @param data_size the size (in bytes) of @p data
 * @return 0 on success
 * @return a negative errno value on error
 */
int hdlc_write(struct hdlc_context *hdlc_context, const uint8_t *addr,
	       uint8_t addr_size, const uint8_t *ctrl, uint8_t ctrl_size,
	       const uint8_t *data, uint16_t data_len);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_NET_HDLC_H_ */

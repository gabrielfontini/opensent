/*
 * OpenSENT hardware abstraction. Implemented by firmware/h750 (real board)
 * and firmware/sim (host simulator with a virtual TX->RX loopback).
 * SPDX-License-Identifier: MIT
 */
#ifndef OPENSENT_HW_H
#define OPENSENT_HW_H

#include <stdbool.h>
#include <stdint.h>

#include "sent.h"

#define HW_RX_CHANNELS 3u /* 0 = RX0 pin, 1 = RX1 pin, 2 = monitor of the TX pin */
#define HW_DAC_CHANNELS 2u

void        hw_init(void);
uint32_t    hw_millis(void);
void        hw_led(bool on);
const char *hw_board(void);
const char *hw_uid(void);
const char *hw_pins(void);  /* human readable pin map */

/* console */
void hw_write(const char *s, unsigned n);

/* SENT transmitter: the hardware calls sent_tx_next(tx) whenever it needs
 * the next pulse. hw_tx_lock/unlock bracket changes to tx->cfg. */
uint32_t hw_tx_clk(void);
void     hw_tx_start(sent_tx_t *tx);
void     hw_tx_stop(void);
void     hw_tx_lock(void);
void     hw_tx_unlock(void);
void     hw_tx_drive(bool open_drain);

/* SENT receivers: free running capture timestamps of the active edge */
uint32_t hw_rx_clk(void);
void     hw_rx_enable(unsigned ch, bool en, bool invert);
unsigned hw_rx_read(unsigned ch, uint32_t *ts, unsigned max, uint32_t *overruns);
/* raw capture: timestamp both edges (they alternate). Arming flushes the
 * channel and returns the line level just before the first captured edge. */
bool     hw_rx_both_edges(unsigned ch, bool both, bool invert);
bool     hw_rx_pin(unsigned ch);
/* drive the (unconnected) RX pin with its internal pulls: wire-free test */
bool     hw_rx_pulltest(unsigned ch, sent_tx_t *gen, unsigned frames);

/* analog outputs (bridge) */
float hw_dac_max(void);
void  hw_dac(unsigned ch, float volts);
float hw_dac_meas(unsigned ch); /* measured output (V), < 0 if unsupported */

/* CAN (classic, 11-bit IDs). loopback = internal, no transceiver needed */
bool hw_can_start(uint32_t bitrate, bool loopback);
void hw_can_stop(void);
bool hw_can_send(uint32_t id, const uint8_t *d, uint8_t len);
bool hw_can_recv(uint32_t *id, uint8_t *d, uint8_t *len);
void hw_can_status(uint32_t *tec, uint32_t *rec, uint32_t *psr);

/* non-volatile startup script (QSPI flash on the H750, a file in the sim) */
const char *hw_cfg_info(void);
int         hw_cfg_load(char *buf, unsigned max); /* bytes, or -1 if nothing valid stored */
bool        hw_cfg_store(const char *buf, unsigned len);
bool        hw_boot_skip(void); /* user asked to skip the startup script (button held) */

/* logic analyzer (second USB port, SUMP protocol): sampled port, status, test signal */
bool        hw_la_port(char name);
const char *hw_la_info(void);
bool        hw_la_testsig(uint32_t hz);

void hw_bootloader(void);
/* hang on purpose to prove the watchdog resets the board; false if unsupported */
bool hw_watchdog_test(void);

#endif

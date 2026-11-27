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
/* raw capture: timestamp both edges (they alternate), read the pin level */
void     hw_rx_both_edges(unsigned ch, bool both, bool invert);
bool     hw_rx_pin(unsigned ch);

/* analog outputs (bridge) */
float hw_dac_max(void);
void  hw_dac(unsigned ch, float volts);
float hw_dac_meas(unsigned ch); /* measured output (V), < 0 if unsupported */

void hw_bootloader(void);

#endif

/*
 * Robustness fuzzing of the SENT decoder: random and adversarial pulse
 * streams must never crash, hang or read out of bounds (run under ASan +
 * UBSan), and the decoder must re-lock on a clean stream afterwards.
 * SPDX-License-Identifier: MIT
 */
#include "sent.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint64_t s = 88172645463325252ull;
static uint32_t rnd(void)
{
    s ^= s << 13;
    s ^= s >> 7;
    s ^= s << 17;
    return (uint32_t)s;
}

int main(int argc, char **argv)
{
    long iters = argc > 1 ? atol(argv[1]) : 2000000;
    const uint32_t clk = 200000000u;
    unsigned fails = 0;
    for (int cfgi = 0; cfgi < 24; cfgi++) {
        sent_rx_cfg_t c;
        sent_rx_cfg_default(&c, clk);
        c.nibbles  = (uint8_t)(cfgi % 4 == 0 ? 0 : 1 + cfgi % 8);
        c.tick_ns  = cfgi % 3 == 0 ? 0.0f : 3000.0f;
        c.crc_mode = (uint8_t)(cfgi % 4);
        c.no_pause = cfgi % 5 == 0;
        c.fmt      = (uint8_t)(cfgi % SENT_FMT_COUNT);
        static sent_rx_t rx;
        sent_rx_init(&rx, &c);
        sent_frame_t f;
        for (long i = 0; i < iters / 24; i++) {
            uint32_t r = rnd(), v;
            switch (r & 7) {
            case 0: v = rnd(); break;                                  /* anything, incl. 0 and huge */
            case 1: v = rnd() % 64; break;                             /* glitches */
            case 2: v = 600u * (56u + (rnd() % 3) - 1u); break;        /* sync-ish */
            case 3: v = 600u * (12u + rnd() % 16u); break;             /* nibble */
            case 4: v = 600u * (12u + rnd() % 900u); break;            /* pause-ish */
            case 5: v = (rnd() % 2) ? 0u : 0xFFFFFFFFu; break;         /* extremes */
            default: v = 600u * (10u + rnd() % 60u) + rnd() % 600u; break;
            }
            if (sent_rx_feed(&rx, v, &f)) {
                if (f.n > SENT_MAX_DATA || f.err >= SENT_E_COUNT) {
                    printf("bad frame fields n=%u err=%u\n", f.n, f.err);
                    fails++;
                }
            }
        }
        /* recovery: a clean stream must decode again */
        sent_tx_cfg_t tc;
        sent_tx_cfg_default(&tc);
        if (c.nibbles) {
            tc.nibbles = c.nibbles;
        }
        if (c.crc_mode != SENT_CRC_AUTO) {
            tc.crc_mode = c.crc_mode;
        }
        tc.fc[0].kind = SENT_WAVE_COUNT_UP;
        tc.fc[0].max = 4095;
        sent_tx_t tx;
        sent_tx_init(&tx, clk, &tc);
        uint32_t ok0 = rx.st.ok;
        for (int i = 0; i < 400; i++) {
            sent_rx_feed(&rx, sent_tx_next(&tx, NULL), &f);
        }
        if (rx.st.ok - ok0 < 30) {
            printf("cfg %d: no recovery (ok %u)\n", cfgi, rx.st.ok - ok0);
            fails++;
        }
    }
    printf("fuzz: %ld pulses, %u failures\n", iters, fails);
    return fails ? 1 : 0;
}

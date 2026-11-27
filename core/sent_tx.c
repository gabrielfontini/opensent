/*
 * OpenSENT - SENT frame generator (sensor emulation).
 *
 * Produces a continuous stream of pulse lengths in timer clocks. Frame
 * content is built when the previous frame has been fully streamed, so
 * configuration changes apply at frame boundaries. Rounding to timer
 * clocks uses error diffusion: the average tick is exact even when the
 * tick is not an integer number of clocks.
 *
 * SPDX-License-Identifier: MIT
 */
#include "sent.h"

#include <math.h>
#include <string.h>

void sent_tx_cfg_default(sent_tx_cfg_t *c)
{
    memset(c, 0, sizeof *c);
    c->tick_ns     = 3000.0f;
    c->low_ticks   = 5;
    c->nibbles     = 6;
    c->crc_mode    = SENT_CRC_2010;
    c->fmt         = SENT_FMT_12_12;
    c->pause_mode  = SENT_PAUSE_OFF;
    c->pause_ticks = 0;
    c->fc[0].kind  = SENT_WAVE_CONST;
    c->fc[0].value = 0x800;
    c->fc[1].kind  = SENT_WAVE_CONST;
    c->fc[1].value = 0x7FF;
    c->slow_fmt    = SENT_SLOW_NONE;
}

void sent_tx_init(sent_tx_t *tx, uint32_t clk_hz, const sent_tx_cfg_t *cfg)
{
    memset(tx, 0, sizeof *tx);
    tx->clk_hz      = clk_hz;
    tx->cfg         = *cfg;
    tx->scale       = 1.0f;
    tx->fault_every = 1;
}

static uint32_t wave_value(const sent_wave_t *w, double t, uint32_t frame, uint32_t maxval)
{
    double lo = w->min, hi = w->max, v;
    double p = 0.0;
    if (w->kind != SENT_WAVE_CONST && w->kind != SENT_WAVE_COUNT_UP && w->period_s > 0.0f) {
        p = fmod(t / (double)w->period_s, 1.0);
    }
    switch (w->kind) {
    case SENT_WAVE_RAMP:   v = lo + (hi - lo) * p; break;
    case SENT_WAVE_TRI:    v = lo + (hi - lo) * (p < 0.5 ? 2.0 * p : 2.0 - 2.0 * p); break;
    case SENT_WAVE_SINE:   v = lo + (hi - lo) * (0.5 - 0.5 * cos(6.283185307179586 * p)); break;
    case SENT_WAVE_SQUARE: v = p < 0.5 ? lo : hi; break;
    case SENT_WAVE_COUNT_UP: {
        uint32_t span = w->max >= w->min ? w->max - w->min + 1u : 1u;
        v = (double)(w->min + frame % span);
        break;
    }
    case SENT_WAVE_CONST:
    default:
        v = (double)w->value;
        break;
    }
    if (v < 0.0) {
        v = 0.0;
    }
    uint32_t u = (uint32_t)(v + 0.5);
    return u > maxval ? maxval : u;
}

static uint32_t mask_bits(unsigned nibbles)
{
    return nibbles >= 8 ? 0xFFFFFFFFu : ((1u << (4 * nibbles)) - 1u);
}

unsigned sent_tx_build_frame(sent_tx_t *tx, uint16_t *ticks)
{
    const sent_tx_cfg_t *c = &tx->cfg;
    sent_format_t fmt = sent_format((sent_format_id_t)c->fmt);
    unsigned n = c->nibbles;
    if (n < 1) {
        n = 1;
    }
    if (n > SENT_MAX_DATA) {
        n = SENT_MAX_DATA;
    }

    /* fault decision */
    sent_fault_t fault = SENT_FAULT_NONE;
    if (tx->fault != SENT_FAULT_NONE && tx->fault_count) {
        uint32_t every = tx->fault_every ? tx->fault_every : 1u;
        if (tx->fault_phase % every == 0) {
            fault = (sent_fault_t)tx->fault;
            if (tx->fault_count != 0xFFFFFFFFu) {
                tx->fault_count--;
            }
            tx->faults_injected++;
        }
        tx->fault_phase++;
    }
    if (fault == SENT_FAULT_SCRC) {
        tx->slow_bad = true; /* applies to the next message that starts */
        fault = SENT_FAULT_NONE;
    }

    /* fast channel content */
    uint8_t d[SENT_MAX_DATA] = {0};
    if (c->use_raw) {
        memcpy(d, c->raw, n);
    } else {
        tx->last_fc1 = wave_value(&c->fc[0], tx->t_s, tx->frames, mask_bits(fmt.fc1_nibbles));
        tx->last_fc2 = wave_value(&c->fc[1], tx->t_s, tx->frames, mask_bits(fmt.fc2_nibbles));
        uint8_t tmp[SENT_MAX_DATA + 8];
        unsigned k = sent_pack(&fmt, tx->last_fc1, tx->last_fc2, tmp);
        for (unsigned i = 0; i < n && i < k; i++) {
            d[i] = tmp[i];
        }
    }

    /* status nibble: bits 0..1 application, 2..3 serial channel */
    uint8_t status = c->status_bits & 0x3u;
    if (c->slow_fmt != SENT_SLOW_NONE && c->slow_n > 0) {
        if (tx->slow_msg >= c->slow_n) {
            tx->slow_msg = 0;
        }
        if (tx->slow_idx == 0) { /* corruption is latched per message */
            tx->slow_cur_bad = tx->slow_bad;
            tx->slow_bad     = false;
        }
        uint8_t b3, b2;
        unsigned len = sent_slow_bits((sent_slow_fmt_t)c->slow_fmt, c->slow_id[tx->slow_msg],
                                      c->slow_val[tx->slow_msg], tx->slow_idx, tx->slow_cur_bad, &b3, &b2);
        status |= (uint8_t)((b3 << 3) | (b2 << 2));
        if (++tx->slow_idx >= len) {
            tx->slow_idx = 0;
            tx->slow_msg = (uint8_t)((tx->slow_msg + 1u) % c->slow_n);
        }
    }

    uint8_t crc = sent_crc4((sent_crc_mode_t)c->crc_mode, status, d, n);

    tx->last_status = status;
    tx->last_n      = (uint8_t)n;
    memcpy(tx->last_d, d, n);
    tx->last_d[n] = crc;

    if (fault == SENT_FAULT_CRC) {
        crc = (uint8_t)((crc + 1u) & 0xFu);
    }

    unsigned k = 0;
    ticks[k++] = SENT_SYNC_TICKS;
    ticks[k++] = (uint16_t)(SENT_NIBBLE_OFFSET + status);
    unsigned nd = n;
    if (fault == SENT_FAULT_SHORT && nd > 1) {
        nd--;
    }
    for (unsigned i = 0; i < nd; i++) {
        ticks[k++] = (uint16_t)(SENT_NIBBLE_OFFSET + d[i]);
    }
    if (fault == SENT_FAULT_LONG) {
        ticks[k++] = SENT_NIBBLE_OFFSET + 5u;
    }
    ticks[k++] = (uint16_t)(SENT_NIBBLE_OFFSET + crc);

    if (fault == SENT_FAULT_SYNC) {
        ticks[0] = 40;
    }
    if (fault == SENT_FAULT_NIBBLE) {
        ticks[2] = 30;
    }
    tx->scale = fault == SENT_FAULT_CAL ? 1.03f : 1.0f;

    if (c->pause_mode == SENT_PAUSE_FIXED && c->pause_ticks >= SENT_PAUSE_MIN) {
        ticks[k++] = c->pause_ticks > SENT_PAUSE_MAX ? SENT_PAUSE_MAX : c->pause_ticks;
    } else if (c->pause_mode == SENT_PAUSE_CONST) {
        unsigned total = 0;
        for (unsigned i = 0; i < k; i++) {
            total += ticks[i];
        }
        unsigned p = c->pause_ticks > total + SENT_PAUSE_MIN ? c->pause_ticks - total : SENT_PAUSE_MIN;
        ticks[k++] = (uint16_t)(p > SENT_PAUSE_MAX ? SENT_PAUSE_MAX : p);
    }
    tx->frames++;
    return k;
}

uint32_t sent_tx_next(sent_tx_t *tx, uint32_t *low)
{
    if (tx->pos >= tx->nticks) {
        tx->nticks = (uint8_t)sent_tx_build_frame(tx, tx->ticks);
        tx->pos    = 0;
    }
    const double clk_per_tick = (double)tx->cfg.tick_ns * 1e-9 * (double)tx->clk_hz * (double)tx->scale;
    double exact = (double)tx->ticks[tx->pos++] * clk_per_tick + tx->frac;
    uint32_t out = (uint32_t)(exact + 0.5);
    tx->frac = exact - (double)out;
    tx->t_s += (double)out / (double)tx->clk_hz;
    if (low) {
        unsigned lt = tx->cfg.low_ticks < 4 ? 4u : tx->cfg.low_ticks;
        *low = (uint32_t)((double)lt * clk_per_tick + 0.5);
    }
    return out;
}

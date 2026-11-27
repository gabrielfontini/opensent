/*
 * OpenSENT - streaming SENT decoder.
 *
 * Works on falling-edge intervals. A "segment" is the list of pulses
 * between two sync pulses; it is decoded when the next sync arrives, so
 * pause pulses (fixed, constant-frame-length, shorter or longer than a
 * sync, even ones that look like a sync) are recognised by position
 * instead of by guessing.
 *
 * Each nibble is measured against the sync pulse of its own frame
 * (ratio * 56), which is what J2716 asks for and makes the decoder immune
 * to slow clock drift of the sensor.
 *
 * SPDX-License-Identifier: MIT
 */
#include "sent.h"

#include <math.h>
#include <string.h>

#define SYNC_TOL        0.20f   /* sync-to-sync tolerance */
#define NIBBLE_MAX_TICK 27.5f
#define NIBBLE_MIN_TICK 11.5f
#define GLITCH_TICKS    2u      /* pulses shorter than this are merged into the next */
#define GLITCH_LOCKED   11u     /* same, once locked */
#define MARGIN_TICKS    0.35f   /* |deviation| above this sets SENT_F_MARGIN */
#define LEARN_VOTES     3u
#define CRC_LOCK_VOTES  4u
#define UNLEARN_ERRORS  8u      /* consecutive errors that discard a learned length / CRC */
#define SEG_MAX         (sizeof(((sent_rx_t *)0)->seg) / sizeof(uint32_t))

void sent_rx_cfg_default(sent_rx_cfg_t *c, uint32_t clk_hz)
{
    memset(c, 0, sizeof *c);
    c->clk_hz   = clk_hz;
    c->tick_ns  = 0.0f;
    c->tol_pct  = 25;
    c->nibbles  = 0;
    c->crc_mode = SENT_CRC_AUTO;
    c->fmt      = SENT_FMT_12_12;
    c->no_pause = false;
}

void sent_rx_reset(sent_rx_t *rx)
{
    rx->have_sync   = false;
    rx->locked      = false;
    rx->sync        = 0;
    rx->prev_sync   = 0;
    rx->nseg        = 0;
    rx->learned_n   = 0;
    rx->learn_votes = 0;
    rx->crc_lock    = 0xFF;
    rx->crc_votes   = 0;
    rx->carry       = 0;
    rx->bad_len     = 0;
    rx->bad_crc     = 0;
    memset(&rx->st, 0, sizeof rx->st);
    sent_slow_rx_reset(&rx->slow, true);
}

void sent_rx_init(sent_rx_t *rx, const sent_rx_cfg_t *cfg)
{
    memset(rx, 0, sizeof *rx);
    rx->cfg = *cfg;
    rx->fmt = sent_format((sent_format_id_t)cfg->fmt);
    sent_rx_reset(rx);
}

/* absolute plausibility of a sync pulse, used when nothing is known */
static bool sync_in_range(const sent_rx_t *rx, uint32_t clocks)
{
    float tick_ns = (float)clocks * (1e9f / (float)rx->cfg.clk_hz) / (float)SENT_SYNC_TICKS;
    if (rx->cfg.tick_ns > 0.0f) {
        float tol = (float)rx->cfg.tol_pct / 100.0f;
        return tick_ns >= rx->cfg.tick_ns * (1.0f - tol) && tick_ns <= rx->cfg.tick_ns * (1.0f + tol);
    }
    return tick_ns >= 1500.0f && tick_ns <= 100000.0f;
}

static unsigned known_nibbles(const sent_rx_t *rx)
{
    if (rx->cfg.nibbles) {
        return rx->cfg.nibbles;
    }
    return rx->learn_votes >= LEARN_VOTES ? rx->learned_n : 0u;
}

static void open_segment(sent_rx_t *rx, uint32_t clocks)
{
    rx->have_sync = true;
    rx->sync      = clocks;
    rx->sync_ts   = rx->ts - clocks;
    rx->nseg      = 0;
}

static void lose_lock(sent_rx_t *rx)
{
    if (rx->locked) {
        rx->st.resyncs++;
    }
    rx->locked = false;
    sent_slow_rx_reset(&rx->slow, false);
}

static bool crc_matches(const sent_rx_t *rx, uint8_t status, const uint8_t *d, unsigned n,
                        uint8_t crc, uint8_t *mode_out)
{
    if (rx->cfg.crc_mode != SENT_CRC_AUTO) {
        *mode_out = rx->cfg.crc_mode;
        return sent_crc4((sent_crc_mode_t)rx->cfg.crc_mode, status, d, n) == crc;
    }
    if (rx->crc_lock != 0xFF && rx->crc_votes >= CRC_LOCK_VOTES) {
        *mode_out = rx->crc_lock;
        return sent_crc4((sent_crc_mode_t)rx->crc_lock, status, d, n) == crc;
    }
    for (uint8_t m = SENT_CRC_2010; m <= SENT_CRC_STATUS; m++) {
        if (sent_crc4((sent_crc_mode_t)m, status, d, n) == crc) {
            *mode_out = m;
            return true;
        }
    }
    *mode_out = 0xFF;
    return false;
}

/* Interpret p[0..cnt-1] as status, data..., crc. */
static sent_err_t decode_nibbles(const sent_rx_t *rx, const uint32_t *p, unsigned cnt, sent_frame_t *f)
{
    const float scale = (float)SENT_SYNC_TICKS / (float)rx->sync;
    float worst = 0.0f;
    uint8_t v[SENT_MAX_DATA + 2];
    sent_err_t err = SENT_OK;

    for (unsigned i = 0; i < cnt; i++) {
        float r = (float)p[i] * scale;
        if (r < NIBBLE_MIN_TICK) {
            return SENT_E_SHORT;
        }
        if (r > NIBBLE_MAX_TICK) {
            err  = SENT_E_NIBBLE;
            v[i] = 0;
            continue;
        }
        float q   = floorf(r + 0.5f);
        float dev = fabsf(r - q);
        if (dev > worst) {
            worst = dev;
        }
        v[i] = (uint8_t)((int)q - (int)SENT_NIBBLE_OFFSET);
    }
    f->status = v[0];
    f->n      = (uint8_t)(cnt - 2);
    memcpy(f->d, &v[1], f->n);
    f->crc        = v[cnt - 1];
    f->max_dev_mt = (uint16_t)(worst * 1000.0f + 0.5f);
    f->flags      = worst > MARGIN_TICKS ? SENT_F_MARGIN : 0u;
    if (err != SENT_OK) {
        return err;
    }
    if (!crc_matches(rx, f->status, f->d, f->n, f->crc, &f->crc_mode)) {
        return SENT_E_CRC;
    }
    return SENT_OK;
}

/* Decode p[0..cnt-1] (status..crc, optionally followed by a pause) into f.
 * No side effects on the decoder state. */
static sent_err_t decode_raw(const sent_rx_t *rx, const uint32_t *p, unsigned cnt, sent_frame_t *f)
{
    const float scale = (float)SENT_SYNC_TICKS / (float)rx->sync;
    const unsigned n_cfg = known_nibbles(rx);
    bool pause = false;

    memset(f, 0, sizeof *f);
    f->ts      = rx->sync_ts;
    f->tick_ns = (float)rx->sync * (1e9f / (float)rx->cfg.clk_hz) / (float)SENT_SYNC_TICKS;

    sent_err_t err = SENT_OK;
    float last = cnt ? (float)p[cnt - 1] * scale : 0.0f;

    if (n_cfg) {
        if (cnt == n_cfg + 3u && !rx->cfg.no_pause) {
            pause = true;
        } else if (cnt != n_cfg + 2u) {
            err = SENT_E_LENGTH;
        }
    } else if (last > NIBBLE_MAX_TICK && !rx->cfg.no_pause) {
        pause = true;
    }
    unsigned body = cnt - (pause ? 1u : 0u);
    if (err == SENT_OK && (body < 3u || body > SENT_MAX_DATA + 2u)) {
        err = SENT_E_LENGTH;
    }

    if (err == SENT_OK) {
        err = decode_nibbles(rx, p, body, f);
        /* learning with a nibble-sized last pulse: it is either the CRC or a
         * short pause. Try both readings; when both pass the CRC (1 in 16,
         * and it sticks with a constant signal) prefer the J2716 typical
         * 6 data nibbles, then the reading with the pause. */
        if (!n_cfg && !pause && !rx->cfg.no_pause && cnt >= 4u) {
            sent_frame_t alt = *f;
            if (decode_nibbles(rx, p, cnt - 1u, &alt) == SENT_OK) {
                int da = (int)f->n - 6, db = (int)alt.n - 6;
                if (err != SENT_OK || (db < 0 ? -db : db) <= (da < 0 ? -da : da)) {
                    *f    = alt;
                    pause = true;
                    err   = SENT_OK;
                }
            }
        }
    }

    if (pause) {
        f->flags |= SENT_F_PAUSE;
        float q = floorf(last + 0.5f);
        f->pause_ticks = q > 65535.0f ? 65535u : (uint16_t)q;
        if (last < (float)SENT_PAUSE_MIN - 0.5f && err == SENT_OK) {
            err = SENT_E_SHORT;
        }
    }

    if (rx->prev_sync) {
        uint32_t a = rx->sync, b = rx->prev_sync;
        uint32_t d = a > b ? a - b : b - a;
        if ((uint64_t)d * 64u > b) {
            f->flags |= SENT_F_CAL;
        }
    }
    if (rx->cfg.tick_ns > 0.0f && fabsf(f->tick_ns - rx->cfg.tick_ns) > 0.20f * rx->cfg.tick_ns) {
        f->flags |= SENT_F_TICK;
    }
    f->err = (uint8_t)err;
    return err;
}

/* account for a decoded frame */
static void commit(sent_rx_t *rx, sent_frame_t *f)
{
    rx->st.frames++;
    rx->st.err[f->err]++;
    if (f->flags & SENT_F_PAUSE) {
        rx->st.pauses++;
    }
    if (f->flags & SENT_F_CAL) {
        rx->st.cal_flags++;
    }
    if (f->flags & SENT_F_MARGIN) {
        rx->st.margin_flags++;
    }
    if (f->err != SENT_OK) {
        /* garbage (noise at power-up...) may have taught us a wrong frame
         * length or CRC variant: forget it after repeated errors */
        if (f->err == SENT_E_LENGTH && !rx->cfg.nibbles && ++rx->bad_len >= UNLEARN_ERRORS) {
            rx->learned_n = 0;
            rx->learn_votes = 0;
            rx->bad_len = 0;
        }
        if (f->err == SENT_E_CRC && rx->cfg.crc_mode == SENT_CRC_AUTO && ++rx->bad_crc >= UNLEARN_ERRORS) {
            rx->crc_lock = 0xFF;
            rx->crc_votes = 0;
            rx->bad_crc = 0;
        }
        lose_lock(rx);
        return;
    }
    rx->bad_len = rx->bad_crc = 0;
    rx->st.ok++;
    rx->locked    = true;
    rx->prev_sync = rx->sync;
    sent_unpack(&rx->fmt, f->d, f->n, &f->fc1, &f->fc2);

    if (!rx->cfg.nibbles) {
        if (f->n == rx->learned_n) {
            if (rx->learn_votes < 255) {
                rx->learn_votes++;
            }
        } else if (rx->learn_votes <= 1) {
            rx->learned_n   = f->n;
            rx->learn_votes = 1;
        } else {
            rx->learn_votes--;
        }
    }
    if (rx->cfg.crc_mode == SENT_CRC_AUTO && rx->crc_votes < CRC_LOCK_VOTES) {
        if (f->crc_mode == rx->crc_lock) {
            rx->crc_votes++;
        } else {
            rx->crc_lock  = f->crc_mode;
            rx->crc_votes = 1;
        }
    }
    sent_slow_rx_feed(&rx->slow, f->status, f->ts);
}

static bool feed_one(sent_rx_t *rx, uint32_t clocks, sent_frame_t *out);

/* hunting and the candidate sync was wrong: replay the pulses after it */
static bool replay(sent_rx_t *rx, uint32_t clocks, sent_frame_t *out)
{
    uint32_t tmp[SEG_MAX + 1];
    unsigned n = rx->nseg;
    memcpy(tmp, rx->seg, n * sizeof tmp[0]);
    tmp[n++] = clocks;
    for (unsigned i = 0; i < n; i++) {
        rx->ts -= tmp[i];
    }
    rx->have_sync = false;
    bool got = false;
    for (unsigned i = 0; i < n; i++) {
        rx->ts += tmp[i];
        got |= feed_one(rx, tmp[i], out);
    }
    return got;
}

static bool feed_one(sent_rx_t *rx, uint32_t clocks, sent_frame_t *out)
{
    if (!rx->have_sync) {
        if (sync_in_range(rx, clocks)) {
            open_segment(rx, clocks);
        }
        return false;
    }

    const float r = (float)clocks * (float)SENT_SYNC_TICKS / (float)rx->sync;
    const bool sync_like = fabsf(r - (float)SENT_SYNC_TICKS) <= SYNC_TOL * (float)SENT_SYNC_TICKS;

    if (rx->nseg == 0 && r > NIBBLE_MAX_TICK) {
        /* a status nibble is never this long: the pulse that opened the
         * segment was a pause (e.g. constant frame length), this is the sync */
        if (rx->locked) {
            rx->st.pauses++;
        }
        if (sync_in_range(rx, clocks) || sync_like) {
            open_segment(rx, clocks);
        } else {
            rx->have_sync = false;
        }
        return false;
    }

    if (sync_like) {
        decode_raw(rx, rx->seg, rx->nseg, out);
        commit(rx, out);
        open_segment(rx, clocks);
        return true;
    }

    if (r < NIBBLE_MIN_TICK && !rx->locked) {
        return replay(rx, clocks, out);
    }

    if (r > NIBBLE_MAX_TICK && !rx->locked) {
        /* hunting: either a pause closing a frame, or a better sync candidate */
        if (rx->nseg >= 3u && rx->nseg < SEG_MAX) {
            rx->seg[rx->nseg] = clocks;
            if (decode_raw(rx, rx->seg, rx->nseg + 1u, out) == SENT_OK) {
                commit(rx, out);
                rx->have_sync = false; /* the next pulse must be a sync */
                return true;
            }
        }
        if (sync_in_range(rx, clocks)) {
            open_segment(rx, clocks);
        } else {
            rx->have_sync = false;
        }
        return false;
    }

    /* locked with a known length and the slot after the CRC (pause) is
     * already taken: this pulse should have been a sync */
    const unsigned n_known = known_nibbles(rx);
    if (rx->locked && n_known && rx->nseg >= n_known + (rx->cfg.no_pause ? 2u : 3u)) {
        if (decode_raw(rx, rx->seg, rx->nseg, out) == SENT_OK) {
            commit(rx, out);           /* that frame was fine ... */
            rx->st.err[SENT_E_SYNC]++; /* ... but the next one is lost */
            lose_lock(rx);
        } else {
            out->err = SENT_E_LENGTH; /* one frame with too many pulses */
            commit(rx, out);
        }
        rx->have_sync = false;
        return true;
    }

    if (rx->nseg >= SEG_MAX) {
        memset(out, 0, sizeof *out);
        out->ts  = rx->sync_ts;
        out->err = SENT_E_LENGTH;
        commit(rx, out);
        rx->have_sync = false;
        return true;
    }
    rx->seg[rx->nseg++] = clocks;
    return false;
}

bool sent_rx_feed(sent_rx_t *rx, uint32_t clocks, sent_frame_t *out)
{
    rx->st.pulses++;
    rx->ts += clocks;

    /* glitch filter: a spurious edge pair (noise, ringing on a slow rising
     * edge) splits one pulse in two and the first part is shorter than any
     * legal pulse: merge it into the next one. While locked anything below
     * 11 ticks is illegal; while hunting only tiny spikes are merged so a
     * wrong sync candidate can still be detected. */
    uint32_t glitch = rx->locked      ? rx->sync * GLITCH_LOCKED / SENT_SYNC_TICKS
                    : rx->have_sync   ? rx->sync * GLITCH_TICKS / SENT_SYNC_TICKS
                                      : rx->cfg.clk_hz / 1000000u;
    if (clocks < glitch) {
        rx->carry += clocks;
        rx->st.glitches++;
        return false;
    }
    clocks += rx->carry;
    rx->carry = 0;
    return feed_one(rx, clocks, out);
}

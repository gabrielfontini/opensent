/*
 * OpenSENT instrument application: command interpreter, receive channels,
 * frame streaming and the SENT -> analog bridge. Hardware independent.
 *
 * Protocol (see docs/protocol.md): one ASCII line in, exactly one line out
 * (OK / value / ERR <code>,"msg"); several commands separated by ';'.
 * Asynchronous frame records start with '#', so a host can tell them apart.
 *
 * SPDX-License-Identifier: MIT
 */
#include "app.h"

#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hw.h"

#define FW_VERSION "0.1.0"
#define RAW_KEEP   64u
#define LINE_MAX   200u
#define OUT_MAX    1024u

typedef struct {
    bool         en, inv;
    sent_rx_cfg_t cfg;
    sent_rx_t    rx;
    sent_frame_t last;      /* last frame (any result) */
    sent_frame_t last_ok;   /* last good frame */
    bool         have_last, have_ok, have_prev;
    uint32_t     last_ok_ms;
    uint32_t     prev_ts;
    uint32_t     raw[RAW_KEEP];
    uint8_t      raw_pos;
    uint32_t     overruns;
} chan_t;

typedef struct {
    bool     en;
    uint8_t  ch;        /* rx channel */
    uint8_t  sig;       /* 1 = FC1, 2 = FC2 */
    float    in_min, in_max, v_min, v_max;
    float    fail_v;
    uint32_t timeout_ms;
    float    out;
    bool     failed;
} bridge_t;

#define CAP_MAX 8192u

static struct {
    bool     active, done;
    uint8_t  ch;
    bool     level0;    /* line level before the first captured edge */
    uint32_t n, target;
    uint32_t first, last;
    uint32_t ovr;       /* edges lost while capturing (capture unusable) */
    uint32_t d[CAP_MAX]; /* intervals between consecutive edges (clocks) */
} CAP;

/* SENT -> CAN gateway: one mapping per receive channel */
static struct {
    bool     on, loopback;
    uint32_t bitrate;
    uint32_t id[HW_RX_CHANNELS], period_ms[HW_RX_CHANNELS], last_ms[HW_RX_CHANNELS];
    uint32_t err_mark[HW_RX_CHANNELS];
    uint8_t  counter[HW_RX_CHANNELS];
    uint32_t tx, tx_drop, rx;
    uint32_t rx_id;
    uint8_t  rx_d[8], rx_len;
} CAN;

static struct {
    sent_tx_t     tx;
    sent_tx_cfg_t txc;  /* staged config; copied into tx under lock */
    bool          tx_on, tx_od;
    chan_t        ch[HW_RX_CHANNELS];
    bridge_t      br[HW_DAC_CHANNELS];
    uint8_t       stream_mask;
    uint32_t      stream_every, stream_n[HW_RX_CHANNELS];
    uint32_t      stream_dropped;
    int           last_err;
    char          last_err_msg[48];
    uint32_t      led_ms;
} S;

/* ------------------------------------------------------------ output */

static char   out[OUT_MAX];
static size_t out_len;

static void outf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(out + out_len, sizeof out - out_len, fmt, ap);
    va_end(ap);
    if (n > 0) {
        out_len += (size_t)n;
        if (out_len >= sizeof out) {
            out_len = sizeof out - 1;
        }
    }
}

/* ------------------------------------------------------------ errors */

typedef enum {
    E_OK = 0,
    E_CMD = -100,
    E_PARAM_NOT_ALLOWED = -108,
    E_MISSING = -109,
    E_UNDEF = -113,
    E_DATA = -104,
    E_RANGE = -222,
    E_ILLEGAL = -224,
    E_STATE = -221,
    E_OVERRUN = -363,
} err_t;

static const char *err_text(err_t e)
{
    switch (e) {
    case E_CMD: return "Command error";
    case E_PARAM_NOT_ALLOWED: return "Parameter not allowed";
    case E_MISSING: return "Missing parameter";
    case E_UNDEF: return "Undefined header";
    case E_DATA: return "Data type error";
    case E_RANGE: return "Data out of range";
    case E_ILLEGAL: return "Illegal parameter value";
    case E_STATE: return "Settings conflict";
    case E_OVERRUN: return "Input buffer overrun";
    default: return "No error";
    }
}

/* ------------------------------------------------------------ helpers */

static bool ieq(const char *a, const char *b)
{
    while (*a && *b) {
        if (toupper((unsigned char)*a++) != toupper((unsigned char)*b++)) {
            return false;
        }
    }
    return *a == *b;
}

static bool parse_bool(const char *s, bool *v)
{
    if (ieq(s, "ON") || ieq(s, "1")) {
        *v = true;
        return true;
    }
    if (ieq(s, "OFF") || ieq(s, "0")) {
        *v = false;
        return true;
    }
    return false;
}

static bool parse_num(const char *s, double *v)
{
    char *end;
    if (!s || !*s) {
        return false;
    }
    *v = strtod(s, &end);
    while (*end == ' ') {
        end++;
    }
    return *end == '\0' && isfinite(*v);
}

static bool parse_uint(const char *s, uint32_t *v)
{
    char *end;
    if (!s || !*s) {
        return false;
    }
    unsigned long x = strtoul(s, &end, 0);
    while (*end == ' ') {
        end++;
    }
    if (*end != '\0') {
        return false;
    }
    *v = (uint32_t)x;
    return true;
}

/* split comma separated args in place */
static unsigned split(char *s, char **argv, unsigned max)
{
    unsigned n = 0;
    if (!s) {
        return 0;
    }
    while (*s && n < max) {
        while (*s == ' ') {
            s++;
        }
        argv[n++] = s;
        char *c = strchr(s, ',');
        char *e = c ? c : s + strlen(s);
        char *t = e;
        while (t > s && t[-1] == ' ') {
            *--t = '\0';
        }
        if (!c) {
            break;
        }
        *c = '\0';
        s  = c + 1;
    }
    return n;
}

static void tx_apply(void)
{
    hw_tx_lock();
    S.tx.cfg = S.txc;
    hw_tx_unlock();
}

static void rx_apply(unsigned ch)
{
    chan_t *c = &S.ch[ch];
    c->cfg.clk_hz = hw_rx_clk();
    sent_rx_init(&c->rx, &c->cfg);
    c->have_last = c->have_ok = c->have_prev = false;
    c->overruns = 0;
}

static void defaults(void)
{
    hw_tx_stop();
    S.tx_on = false;
    sent_tx_cfg_default(&S.txc);
    sent_tx_init(&S.tx, hw_tx_clk(), &S.txc);
    S.tx_od = false;
    hw_tx_drive(false);
    for (unsigned i = 0; i < HW_RX_CHANNELS; i++) {
        chan_t *c = &S.ch[i];
        memset(c, 0, sizeof *c);
        sent_rx_cfg_default(&c->cfg, hw_rx_clk());
        rx_apply(i);
        c->en = false;
        hw_rx_enable(i, false, false);
    }
    for (unsigned i = 0; i < HW_DAC_CHANNELS; i++) {
        bridge_t *b = &S.br[i];
        memset(b, 0, sizeof *b);
        b->ch = 0;
        b->sig = 1;
        b->in_min = 0;
        b->in_max = 4095;
        b->v_min = 0;
        b->v_max = hw_dac_max();
        b->fail_v = 0;
        b->timeout_ms = 100;
        hw_dac(i, 0.0f);
    }
    hw_can_stop();
    memset(&CAN, 0, sizeof CAN);
    CAN.bitrate = 500000;
    S.stream_mask = 0;
    S.stream_every = 1;
    S.last_err = 0;
}

void app_init(void)
{
    memset(&S, 0, sizeof S);
    defaults();
}

/* ------------------------------------------------------- formatting */

static void fmt_frame(unsigned ch, const sent_frame_t *f)
{
    outf("ch=%u ts_us=%.1f err=%s status=%X data=", ch,
         (double)f->ts * 1e6 / (double)hw_rx_clk(), sent_err_name((sent_err_t)f->err), f->status);
    for (unsigned i = 0; i < f->n; i++) {
        outf("%X", f->d[i]);
    }
    outf(" crc=%X crc_mode=%s fc1=%lu fc2=%lu tick_ns=%.1f pause=%u flags=%u dev_mt=%u", f->crc,
         f->crc_mode < 3 ? sent_crc_name((sent_crc_mode_t)f->crc_mode) : "-", (unsigned long)f->fc1,
         (unsigned long)f->fc2, (double)f->tick_ns, f->pause_ticks, f->flags, f->max_dev_mt);
}

/* --------------------------------------------------------- commands */

static void process_channel(unsigned ch);

typedef struct {
    char *hdr;      /* upper-cased header without '?' */
    bool  query;
    char *arg;      /* raw argument string or NULL */
    int   idx;      /* numeric suffix of the first node, -1 if none */
} cmd_t;

static err_t need_arg(const cmd_t *c)
{
    return (c->arg && *c->arg) ? E_OK : E_MISSING;
}

/* "RX1:LAST" -> node "RX", idx 1, rest "LAST" */
static const char *node(cmd_t *c, const char *name)
{
    size_t n = strlen(name);
    if (strncmp(c->hdr, name, n) != 0) {
        return NULL;
    }
    const char *p = c->hdr + n;
    c->idx = -1;
    if (isdigit((unsigned char)*p)) {
        c->idx = *p - '0';
        p++;
    }
    if (*p == ':') {
        return p + 1;
    }
    if (*p == '\0') {
        return p;
    }
    return NULL;
}

static err_t cmd_tx(cmd_t *c, const char *sub)
{
    sent_tx_cfg_t *t = &S.txc;
    char *av[6];
    double v;
    uint32_t u;

    if (ieq(sub, "STATE")) {
        if (c->query) {
            outf("%d", S.tx_on);
            return E_OK;
        }
        bool on;
        if (need_arg(c) || !parse_bool(c->arg, &on)) {
            return E_ILLEGAL;
        }
        if (on && !S.tx_on) {
            sent_tx_init(&S.tx, hw_tx_clk(), t);
            hw_tx_start(&S.tx);
        } else if (!on && S.tx_on) {
            hw_tx_stop();
        }
        S.tx_on = on;
    } else if (ieq(sub, "TICK")) {
        if (c->query) {
            outf("%.4f", (double)t->tick_ns / 1000.0);
            return E_OK;
        }
        if (!parse_num(c->arg, &v)) {
            return E_DATA;
        }
        if (v < 0.5 || v > 100.0) {
            return E_RANGE;
        }
        t->tick_ns = (float)(v * 1000.0);
    } else if (ieq(sub, "LOW")) {
        if (c->query) {
            outf("%u", t->low_ticks);
            return E_OK;
        }
        if (!parse_uint(c->arg, &u)) {
            return E_DATA;
        }
        if (u < 4 || u > 11) {
            return E_RANGE;
        }
        t->low_ticks = (uint8_t)u;
    } else if (ieq(sub, "NIB")) {
        if (c->query) {
            outf("%u", t->nibbles);
            return E_OK;
        }
        if (!parse_uint(c->arg, &u)) {
            return E_DATA;
        }
        if (u < 1 || u > SENT_MAX_DATA) {
            return E_RANGE;
        }
        t->nibbles = (uint8_t)u;
    } else if (ieq(sub, "CRC")) {
        if (c->query) {
            outf("%s", sent_crc_name((sent_crc_mode_t)t->crc_mode));
            return E_OK;
        }
        if (need_arg(c)) {
            return E_MISSING;
        }
        if (ieq(c->arg, "2010")) t->crc_mode = SENT_CRC_2010;
        else if (ieq(c->arg, "LEGACY")) t->crc_mode = SENT_CRC_LEGACY;
        else if (ieq(c->arg, "STATUS")) t->crc_mode = SENT_CRC_STATUS;
        else return E_ILLEGAL;
    } else if (ieq(sub, "FMT")) {
        if (c->query) {
            outf("%s", sent_format_name((sent_format_id_t)t->fmt));
            return E_OK;
        }
        sent_format_id_t f;
        if (need_arg(c) || !sent_format_parse(c->arg, &f)) {
            return E_ILLEGAL;
        }
        t->fmt = (uint8_t)f;
    } else if (ieq(sub, "STATUS")) {
        if (c->query) {
            outf("%u", t->status_bits);
            return E_OK;
        }
        if (!parse_uint(c->arg, &u)) {
            return E_DATA;
        }
        if (u > 3) {
            return E_RANGE;
        }
        t->status_bits = (uint8_t)u;
    } else if (ieq(sub, "FC1") || ieq(sub, "FC2")) {
        sent_wave_t *w = &t->fc[sub[2] - '1'];
        if (c->query) {
            outf("%lu", (unsigned long)(sub[2] == '1' ? S.tx.last_fc1 : S.tx.last_fc2));
            return E_OK;
        }
        if (!parse_uint(c->arg, &u)) {
            return E_DATA;
        }
        if (u > 0xFFFF) {
            return E_RANGE;
        }
        w->kind  = SENT_WAVE_CONST;
        w->value = u;
        t->use_raw = false;
    } else if (ieq(sub, "WAVE")) {
        if (c->query) {
            for (int i = 0; i < 2; i++) {
                const sent_wave_t *w = &t->fc[i];
                outf("%s%s,%lu,%lu,%lu,%g", i ? ";" : "", sent_wave_name((sent_wave_kind_t)w->kind),
                     (unsigned long)w->value, (unsigned long)w->min, (unsigned long)w->max,
                     (double)w->period_s);
            }
            return E_OK;
        }
        unsigned n = split(c->arg, av, 6);
        sent_wave_kind_t k;
        uint32_t ch, lo = 0, hi = 0;
        double per = 1.0;
        if (n < 2 || !parse_uint(av[0], &ch) || !sent_wave_parse(av[1], &k)) {
            return E_ILLEGAL;
        }
        if (ch < 1 || ch > 2) {
            return E_RANGE;
        }
        if (n >= 4 && (!parse_uint(av[2], &lo) || !parse_uint(av[3], &hi))) {
            return E_DATA;
        }
        if (n >= 5 && (!parse_num(av[4], &per) || per <= 0)) {
            return E_RANGE;
        }
        if (k != SENT_WAVE_CONST && n < 4) {
            return E_MISSING;
        }
        sent_wave_t *w = &t->fc[ch - 1];
        w->kind = (uint8_t)k;
        if (k == SENT_WAVE_CONST) {
            w->value = lo;
        }
        w->min = lo;
        w->max = hi;
        w->period_s = (float)per;
        t->use_raw = false;
    } else if (ieq(sub, "RAW")) {
        if (c->query) {
            if (!t->use_raw) {
                outf("OFF");
            }
            for (unsigned i = 0; t->use_raw && i < t->nibbles; i++) {
                outf("%X", t->raw[i]);
            }
            return E_OK;
        }
        if (need_arg(c)) {
            return E_MISSING;
        }
        if (ieq(c->arg, "OFF")) {
            t->use_raw = false;
        } else {
            size_t n = strlen(c->arg);
            if (n < 1 || n > SENT_MAX_DATA) {
                return E_RANGE;
            }
            for (size_t i = 0; i < n; i++) {
                if (!isxdigit((unsigned char)c->arg[i])) {
                    return E_DATA;
                }
                char h[2] = {c->arg[i], 0};
                t->raw[i] = (uint8_t)strtoul(h, NULL, 16);
            }
            t->nibbles = (uint8_t)n;
            t->use_raw = true;
        }
    } else if (ieq(sub, "PAUSE")) {
        if (c->query) {
            if (t->pause_mode == SENT_PAUSE_OFF) outf("OFF");
            else if (t->pause_mode == SENT_PAUSE_FIXED) outf("%u", t->pause_ticks);
            else outf("CONST,%u", t->pause_ticks);
            return E_OK;
        }
        unsigned n = split(c->arg, av, 3);
        if (n == 0) {
            return E_MISSING;
        }
        if (ieq(av[0], "OFF")) {
            t->pause_mode = SENT_PAUSE_OFF;
        } else if (ieq(av[0], "CONST")) {
            if (n < 2 || !parse_uint(av[1], &u)) {
                return E_MISSING;
            }
            if (u < 100 || u > 1000) {
                return E_RANGE;
            }
            t->pause_mode = SENT_PAUSE_CONST;
            t->pause_ticks = (uint16_t)u;
        } else {
            if (!parse_uint(av[0], &u)) {
                return E_DATA;
            }
            if (u < SENT_PAUSE_MIN || u > SENT_PAUSE_MAX) {
                return E_RANGE;
            }
            t->pause_mode = SENT_PAUSE_FIXED;
            t->pause_ticks = (uint16_t)u;
        }
    } else if (ieq(sub, "SLOW")) {
        if (c->query) {
            outf("%s", sent_slow_name((sent_slow_fmt_t)t->slow_fmt));
            for (unsigned i = 0; i < t->slow_n; i++) {
                outf(",%u=%u", t->slow_id[i], t->slow_val[i]);
            }
            return E_OK;
        }
        if (need_arg(c)) {
            return E_MISSING;
        }
        if (ieq(c->arg, "OFF")) t->slow_fmt = SENT_SLOW_NONE;
        else if (ieq(c->arg, "SHORT")) t->slow_fmt = SENT_SLOW_SHORT;
        else if (ieq(c->arg, "ENH12")) t->slow_fmt = SENT_SLOW_ENH12;
        else if (ieq(c->arg, "ENH16")) t->slow_fmt = SENT_SLOW_ENH16;
        else return E_ILLEGAL;
    } else if (ieq(sub, "SLOW:ADD")) {
        uint32_t id, val;
        if (split(c->arg, av, 3) != 2 || !parse_uint(av[0], &id) || !parse_uint(av[1], &val)) {
            return E_MISSING;
        }
        if (id > 255 || val > 0xFFFF) {
            return E_RANGE;
        }
        for (unsigned i = 0; i < t->slow_n; i++) {
            if (t->slow_id[i] == id) {
                t->slow_val[i] = (uint16_t)val;
                tx_apply();
                return E_OK;
            }
        }
        if (t->slow_n >= SENT_TX_SLOW_MAX) {
            return E_RANGE;
        }
        t->slow_id[t->slow_n]  = (uint8_t)id;
        t->slow_val[t->slow_n] = (uint16_t)val;
        t->slow_n++;
    } else if (ieq(sub, "SLOW:CLR")) {
        t->slow_n = 0;
    } else if (ieq(sub, "FAULT")) {
        if (c->query) {
            outf("%s,%lu,%lu,injected=%lu", sent_fault_name((sent_fault_t)S.tx.fault),
                 (unsigned long)S.tx.fault_count, (unsigned long)S.tx.fault_every,
                 (unsigned long)S.tx.faults_injected);
            return E_OK;
        }
        unsigned n = split(c->arg, av, 4);
        sent_fault_t f;
        uint32_t count = 1, every = 1;
        if (n < 1 || !sent_fault_parse(av[0], &f)) {
            return E_ILLEGAL;
        }
        if (n >= 2 && !parse_uint(av[1], &count)) {
            return E_DATA;
        }
        if (n >= 3 && (!parse_uint(av[2], &every) || every == 0)) {
            return E_RANGE;
        }
        hw_tx_lock();
        S.tx.fault_count = count == 0 ? 0xFFFFFFFFu : count;
        S.tx.fault_every = every;
        S.tx.fault_phase = 0;
        S.tx.faults_injected = 0;
        S.tx.fault = (uint8_t)f;
        hw_tx_unlock();
        return E_OK;
    } else if (ieq(sub, "DRIVE")) {
        if (c->query) {
            outf("%s", S.tx_od ? "OD" : "PP");
            return E_OK;
        }
        if (need_arg(c)) {
            return E_MISSING;
        }
        if (ieq(c->arg, "OD")) S.tx_od = true;
        else if (ieq(c->arg, "PP")) S.tx_od = false;
        else return E_ILLEGAL;
        hw_tx_drive(S.tx_od);
        return E_OK;
    } else if (ieq(sub, "INFO") && c->query) {
        hw_tx_lock();
        uint32_t frames = S.tx.frames, fi = S.tx.faults_injected;
        uint8_t st = S.tx.last_status, n = S.tx.last_n, d[SENT_MAX_DATA + 1];
        memcpy(d, S.tx.last_d, sizeof d);
        uint32_t f1 = S.tx.last_fc1, f2 = S.tx.last_fc2;
        double ts = S.tx.t_s;
        hw_tx_unlock();
        outf("state=%d frames=%lu faults=%lu t_s=%.3f status=%X data=", S.tx_on, (unsigned long)frames,
             (unsigned long)fi, ts, st);
        for (unsigned i = 0; i < n; i++) {
            outf("%X", d[i]);
        }
        outf(" crc=%X fc1=%lu fc2=%lu tick_ns=%.1f", d[n], (unsigned long)f1, (unsigned long)f2,
             (double)S.txc.tick_ns);
        return E_OK;
    } else {
        return E_UNDEF;
    }
    if (c->query) {
        return E_UNDEF;
    }
    tx_apply();
    return E_OK;
}

static err_t cmd_rx(cmd_t *c, const char *sub)
{
    unsigned ch = c->idx < 0 ? 0u : (unsigned)c->idx;
    if (ch >= HW_RX_CHANNELS) {
        return E_RANGE;
    }
    chan_t *x = &S.ch[ch];
    sent_rx_cfg_t *k = &x->cfg;
    double v;
    uint32_t u;

    if (ieq(sub, "STATE")) {
        if (c->query) {
            outf("%d", x->en);
            return E_OK;
        }
        bool on;
        if (need_arg(c) || !parse_bool(c->arg, &on)) {
            return E_ILLEGAL;
        }
        x->en = on;
        x->have_prev = false;
        hw_rx_enable(ch, on, x->inv);
        return E_OK;
    } else if (ieq(sub, "POL")) {
        if (c->query) {
            outf("%s", x->inv ? "INV" : "NORM");
            return E_OK;
        }
        if (need_arg(c)) return E_MISSING;
        if (ieq(c->arg, "INV")) x->inv = true;
        else if (ieq(c->arg, "NORM")) x->inv = false;
        else return E_ILLEGAL;
        hw_rx_enable(ch, x->en, x->inv);
        x->have_prev = false;
        return E_OK;
    } else if (ieq(sub, "TICK")) {
        if (c->query) {
            if (k->tick_ns > 0) outf("%.4f", (double)k->tick_ns / 1000.0);
            else outf("AUTO");
            return E_OK;
        }
        if (need_arg(c)) return E_MISSING;
        if (ieq(c->arg, "AUTO")) {
            k->tick_ns = 0;
        } else {
            if (!parse_num(c->arg, &v)) return E_DATA;
            if (v < 0.5 || v > 100.0) return E_RANGE;
            k->tick_ns = (float)(v * 1000.0);
        }
    } else if (ieq(sub, "NIB")) {
        if (c->query) {
            if (k->nibbles) outf("%u", k->nibbles);
            else outf("AUTO");
            return E_OK;
        }
        if (need_arg(c)) return E_MISSING;
        if (ieq(c->arg, "AUTO")) {
            k->nibbles = 0;
        } else {
            if (!parse_uint(c->arg, &u)) return E_DATA;
            if (u < 1 || u > SENT_MAX_DATA) return E_RANGE;
            k->nibbles = (uint8_t)u;
        }
    } else if (ieq(sub, "CRC")) {
        if (c->query) {
            outf("%s", sent_crc_name((sent_crc_mode_t)k->crc_mode));
            return E_OK;
        }
        if (need_arg(c)) return E_MISSING;
        if (ieq(c->arg, "AUTO")) k->crc_mode = SENT_CRC_AUTO;
        else if (ieq(c->arg, "2010")) k->crc_mode = SENT_CRC_2010;
        else if (ieq(c->arg, "LEGACY")) k->crc_mode = SENT_CRC_LEGACY;
        else if (ieq(c->arg, "STATUS")) k->crc_mode = SENT_CRC_STATUS;
        else return E_ILLEGAL;
    } else if (ieq(sub, "FMT")) {
        if (c->query) {
            outf("%s", sent_format_name((sent_format_id_t)k->fmt));
            return E_OK;
        }
        sent_format_id_t f;
        if (need_arg(c) || !sent_format_parse(c->arg, &f)) return E_ILLEGAL;
        k->fmt = (uint8_t)f;
    } else if (ieq(sub, "PAUSE")) {
        if (c->query) {
            outf("%s", k->no_pause ? "NONE" : "AUTO");
            return E_OK;
        }
        if (need_arg(c)) return E_MISSING;
        if (ieq(c->arg, "NONE")) k->no_pause = true;
        else if (ieq(c->arg, "AUTO")) k->no_pause = false;
        else return E_ILLEGAL;
    } else if (ieq(sub, "CAP")) {
        if (c->query) {
            outf("state=%s ch=%u n=%lu target=%lu level0=%d clk=%lu ovr=%lu",
                 CAP.active ? "RUN" : (CAP.done ? "DONE" : "IDLE"), CAP.ch, (unsigned long)CAP.n,
                 (unsigned long)CAP.target, CAP.level0, (unsigned long)hw_rx_clk(), (unsigned long)CAP.ovr);
            return E_OK;
        }
        if (!parse_uint(c->arg, &u)) return E_DATA;
        if (u < 2 || u > CAP_MAX) return E_RANGE;
        if (!x->en) return E_STATE;
        if (CAP.active) {
            hw_rx_both_edges(CAP.ch, false, S.ch[CAP.ch].inv);
        }
        memset(&CAP, 0, sizeof CAP - sizeof CAP.d);
        CAP.ch = (uint8_t)ch;
        CAP.target = u;
        CAP.level0 = hw_rx_both_edges(ch, true, x->inv);
        CAP.active = true;
        return E_OK;
    } else if (ieq(sub, "PULLTEST")) {
        /* wire-free receiver test: frames with FC1 counting from 0, FC2 = 0x5A5 */
        if (!parse_uint(c->arg, &u)) return E_DATA;
        if (u < 1 || u > 2000) return E_RANGE;
        if (!x->en || ch > 1) return E_STATE;
        static sent_tx_t gen;
        sent_tx_cfg_t gc;
        sent_tx_cfg_default(&gc);
        gc.tick_ns = 5000.0f;
        gc.fc[0].kind = SENT_WAVE_COUNT_UP;
        gc.fc[0].min = 0;
        gc.fc[0].max = 4095;
        gc.fc[1].value = 0x5A5;
        sent_tx_init(&gen, hw_tx_clk(), &gc);
        for (uint32_t done = 0; done < u;) {
            uint32_t n = u - done > 20u ? 20u : u - done;
            if (!hw_rx_pulltest(ch, &gen, n)) return E_STATE;
            done += n;
            process_channel(ch); /* keep the capture ring drained */
        }
        return E_OK;
    } else if (ieq(sub, "CAP:DATA") && c->query) {
        char *av[3];
        uint32_t off, cnt;
        if (split(c->arg, av, 3) != 2 || !parse_uint(av[0], &off) || !parse_uint(av[1], &cnt)) return E_MISSING;
        if (cnt > 100 || off > CAP.n) return E_RANGE;
        if (CAP.ch != ch) return E_STATE;
        uint32_t end = off + cnt > CAP.n ? CAP.n : off + cnt;
        if (end == off) {
            outf("END");
        }
        for (uint32_t i = off; i < end; i++) {
            outf("%s%lu", i > off ? "," : "", (unsigned long)CAP.d[i]);
        }
        return E_OK;
    } else if (ieq(sub, "CLR")) {
        /* reset statistics and learned state, keep configuration */
    } else if (ieq(sub, "LAST") && c->query) {
        if (!x->have_last) {
            outf("NONE");
            return E_OK;
        }
        fmt_frame(ch, &x->last);
        return E_OK;
    } else if (ieq(sub, "VAL") && c->query) {
        /* last good values, compact: fc1,fc2,status,age_ms */
        if (!x->have_ok) {
            outf("NONE");
            return E_OK;
        }
        outf("%lu,%lu,%X,%lu", (unsigned long)x->last_ok.fc1, (unsigned long)x->last_ok.fc2, x->last_ok.status,
             (unsigned long)(hw_millis() - x->last_ok_ms));
        return E_OK;
    } else if (ieq(sub, "STAT") && c->query) {
        const sent_rx_stats_t *s = &x->rx.st;
        outf("en=%d lock=%d frames=%lu ok=%lu crc=%lu nibble=%lu length=%lu short=%lu sync=%lu pauses=%lu "
             "cal=%lu margin=%lu glitches=%lu resyncs=%lu pulses=%lu overruns=%lu n=%u crc_mode=%s "
             "tick_ns=%.1f slow_ok=%lu slow_err=%lu",
             x->en, x->rx.locked, (unsigned long)s->frames, (unsigned long)s->ok,
             (unsigned long)s->err[SENT_E_CRC], (unsigned long)s->err[SENT_E_NIBBLE],
             (unsigned long)s->err[SENT_E_LENGTH], (unsigned long)s->err[SENT_E_SHORT],
             (unsigned long)s->err[SENT_E_SYNC], (unsigned long)s->pauses, (unsigned long)s->cal_flags,
             (unsigned long)s->margin_flags, (unsigned long)s->glitches, (unsigned long)s->resyncs,
             (unsigned long)s->pulses, (unsigned long)x->overruns,
             k->nibbles ? k->nibbles : x->rx.learned_n,
             k->crc_mode != SENT_CRC_AUTO ? sent_crc_name((sent_crc_mode_t)k->crc_mode)
             : (x->rx.crc_lock < 3 ? sent_crc_name((sent_crc_mode_t)x->rx.crc_lock) : "-"),
             x->have_ok ? (double)x->last_ok.tick_ns : 0.0, (unsigned long)x->rx.slow.ok,
             (unsigned long)x->rx.slow.crc_err);
        return E_OK;
    } else if (ieq(sub, "SLOW") && c->query) {
        if (x->rx.slow.nmsg == 0) {
            outf("NONE");
        }
        for (unsigned i = 0; i < x->rx.slow.nmsg; i++) {
            const sent_slow_msg_t *m = &x->rx.slow.msg[i];
            outf("%s%s:%u=%u", i ? "," : "", sent_slow_name((sent_slow_fmt_t)m->fmt), m->id, m->value);
        }
        return E_OK;
    } else if (ieq(sub, "RAW") && c->query) {
        /* last captured intervals in ns, oldest first */
        uint32_t n = RAW_KEEP;
        if (c->arg && *c->arg && (!parse_uint(c->arg, &n) || n == 0 || n > RAW_KEEP)) {
            return E_RANGE;
        }
        uint64_t f = hw_rx_clk();
        for (uint32_t i = 0; i < n; i++) {
            uint32_t r = x->raw[(x->raw_pos + RAW_KEEP - n + i) % RAW_KEEP];
            outf("%s%lu", i ? "," : "", (unsigned long)((uint64_t)r * 1000000000ull / f));
        }
        return E_OK;
    } else {
        return E_UNDEF;
    }
    if (c->query) {
        return E_UNDEF;
    }
    rx_apply(ch);
    return E_OK;
}

static err_t cmd_bridge(cmd_t *c, const char *sub)
{
    unsigned n = c->idx < 1 ? 1u : (unsigned)c->idx;
    if (n > HW_DAC_CHANNELS) {
        return E_RANGE;
    }
    bridge_t *b = &S.br[n - 1];
    char *av[8];
    double v[6];
    if (ieq(sub, "STATE")) {
        if (c->query) {
            outf("%d", b->en);
            return E_OK;
        }
        bool on;
        if (need_arg(c) || !parse_bool(c->arg, &on)) return E_ILLEGAL;
        b->en = on;
        if (!on) hw_dac(n - 1, 0.0f);
        return E_OK;
    }
    if (ieq(sub, "MAP")) {
        if (c->query) {
            outf("RX%u,FC%u,%g,%g,%g,%g", b->ch, b->sig, (double)b->in_min, (double)b->in_max,
                 (double)b->v_min, (double)b->v_max);
            return E_OK;
        }
        unsigned k = split(c->arg, av, 8);
        if (k != 6) return E_MISSING;
        const char *ch = av[0], *sig = av[1];
        if (toupper((unsigned char)ch[0]) == 'R') ch += 2; /* "RX0" */
        uint32_t chn;
        if (!parse_uint(ch, &chn) || chn >= HW_RX_CHANNELS) return E_RANGE;
        unsigned s;
        if (ieq(sig, "FC1") || ieq(sig, "1")) s = 1;
        else if (ieq(sig, "FC2") || ieq(sig, "2")) s = 2;
        else return E_ILLEGAL;
        for (int i = 0; i < 4; i++) {
            if (!parse_num(av[2 + i], &v[i])) return E_DATA;
        }
        if (v[0] == v[1]) return E_ILLEGAL;
        if (v[2] < 0 || v[3] < 0 || v[2] > hw_dac_max() || v[3] > hw_dac_max()) return E_RANGE;
        b->ch = (uint8_t)chn;
        b->sig = (uint8_t)s;
        b->in_min = (float)v[0];
        b->in_max = (float)v[1];
        b->v_min = (float)v[2];
        b->v_max = (float)v[3];
        return E_OK;
    }
    if (ieq(sub, "FAIL")) {
        if (c->query) {
            outf("%g,%lu", (double)b->fail_v, (unsigned long)b->timeout_ms);
            return E_OK;
        }
        unsigned k = split(c->arg, av, 3);
        if (k != 2 || !parse_num(av[0], &v[0]) || !parse_num(av[1], &v[1])) return E_MISSING;
        if (v[0] < 0 || v[0] > hw_dac_max() || v[1] < 1 || v[1] > 60000) return E_RANGE;
        b->fail_v = (float)v[0];
        b->timeout_ms = (uint32_t)v[1];
        return E_OK;
    }
    if (ieq(sub, "OUT") && c->query) {
        /* commanded volts, state, measured volts (on-chip read-back) */
        outf("%.4f,%s,%.4f", (double)b->out, !b->en ? "OFF" : (b->failed ? "FAIL" : "OK"),
             (double)hw_dac_meas(n - 1));
        return E_OK;
    }
    return E_UNDEF;
}

static err_t cmd_can(cmd_t *c, const char *sub)
{
    char *av[4];
    uint32_t u, ch, id, per;
    if (ieq(sub, "STATE")) {
        if (c->query) {
            outf("%d", CAN.on);
            return E_OK;
        }
        bool on;
        if (need_arg(c) || !parse_bool(c->arg, &on)) return E_ILLEGAL;
        if (on) {
            if (!hw_can_start(CAN.bitrate, CAN.loopback)) return E_STATE;
        } else {
            hw_can_stop();
        }
        CAN.on = on;
        return E_OK;
    }
    if (ieq(sub, "BITRATE")) {
        if (c->query) {
            outf("%lu", (unsigned long)CAN.bitrate);
            return E_OK;
        }
        if (!parse_uint(c->arg, &u)) return E_DATA;
        if (u < 10000 || u > 1000000 || 25000000u % (25u * u)) return E_RANGE;
        CAN.bitrate = u;
        return CAN.on ? (hw_can_start(u, CAN.loopback) ? E_OK : E_STATE) : E_OK;
    }
    if (ieq(sub, "MODE")) {
        if (c->query) {
            outf("%s", CAN.loopback ? "LOOPBACK" : "NORMAL");
            return E_OK;
        }
        if (need_arg(c)) return E_MISSING;
        if (ieq(c->arg, "LOOPBACK")) CAN.loopback = true;
        else if (ieq(c->arg, "NORMAL")) CAN.loopback = false;
        else return E_ILLEGAL;
        return CAN.on ? (hw_can_start(CAN.bitrate, CAN.loopback) ? E_OK : E_STATE) : E_OK;
    }
    if (ieq(sub, "MAP")) {
        if (c->query) {
            for (unsigned i = 0; i < HW_RX_CHANNELS; i++) {
                outf("%sRX%u=0x%03lX/%lums", i ? "," : "", i, (unsigned long)CAN.id[i],
                     (unsigned long)CAN.period_ms[i]);
            }
            return E_OK;
        }
        unsigned n = split(c->arg, av, 4);
        if (n < 2) return E_MISSING;
        const char *a0 = av[0];
        if (toupper((unsigned char)a0[0]) == 'R') a0 += 2;
        per = 10;
        if (!parse_uint(a0, &ch) || !parse_uint(av[1], &id) || (n >= 3 && !parse_uint(av[2], &per))) return E_DATA;
        if (ch >= HW_RX_CHANNELS || id > 0x7FF || per < 1 || per > 10000) return E_RANGE;
        CAN.id[ch] = id; /* 0 disables */
        CAN.period_ms[ch] = per;
        CAN.err_mark[ch] = 0;
        return E_OK;
    }
    if (ieq(sub, "SEND")) {
        uint8_t d[8];
        if (split(c->arg, av, 3) != 2 || !parse_uint(av[0], &id)) return E_MISSING;
        size_t len = strlen(av[1]);
        if (id > 0x7FF || len % 2 || len > 16) return E_RANGE;
        for (size_t i = 0; i < len / 2; i++) {
            char h[3] = {av[1][2 * i], av[1][2 * i + 1], 0};
            if (!isxdigit((unsigned char)h[0]) || !isxdigit((unsigned char)h[1])) return E_DATA;
            d[i] = (uint8_t)strtoul(h, NULL, 16);
        }
        if (!CAN.on) return E_STATE;
        if (!hw_can_send(id, d, (uint8_t)(len / 2))) {
            CAN.tx_drop++;
            return E_STATE;
        }
        CAN.tx++;
        return E_OK;
    }
    if (ieq(sub, "STAT") && c->query) {
        uint32_t tec, rec, psr;
        hw_can_status(&tec, &rec, &psr);
        outf("on=%d mode=%s bitrate=%lu tx=%lu tx_drop=%lu rx=%lu tec=%lu rec=%lu lec=%lu bus_off=%lu", CAN.on,
             CAN.loopback ? "LOOPBACK" : "NORMAL", (unsigned long)CAN.bitrate, (unsigned long)CAN.tx,
             (unsigned long)CAN.tx_drop, (unsigned long)CAN.rx, (unsigned long)tec, (unsigned long)rec,
             (unsigned long)(psr & 7u), (unsigned long)((psr >> 7) & 1u));
        return E_OK;
    }
    if (ieq(sub, "RX") && c->query) {
        if (!CAN.rx) {
            outf("NONE");
            return E_OK;
        }
        outf("id=0x%03lX len=%u data=", (unsigned long)CAN.rx_id, CAN.rx_len);
        for (unsigned i = 0; i < CAN.rx_len; i++) {
            outf("%02X", CAN.rx_d[i]);
        }
        return E_OK;
    }
    return E_UNDEF;
}

static err_t execute(char *s)
{
    while (*s == ' ' || *s == ':') {
        s++;
    }
    if (!*s) {
        return E_OK;
    }
    cmd_t c = {0};
    c.hdr = s;
    char *sp = strchr(s, ' ');
    if (sp) {
        *sp = '\0';
        c.arg = sp + 1;
        while (*c.arg == ' ') c.arg++;
        char *e = c.arg + strlen(c.arg);
        while (e > c.arg && e[-1] == ' ') *--e = '\0';
    }
    size_t hl = strlen(c.hdr);
    if (hl && c.hdr[hl - 1] == '?') {
        c.query = true;
        c.hdr[hl - 1] = '\0';
    }
    for (char *p = c.hdr; *p; p++) {
        *p = (char)toupper((unsigned char)*p);
    }
    const char *sub;

    if (ieq(c.hdr, "*IDN") && c.query) {
        outf("OpenSENT,%s,%s,%s", hw_board(), hw_uid(), FW_VERSION);
        return E_OK;
    }
    if (ieq(c.hdr, "*RST")) {
        defaults();
        outf("OK");
        return E_OK;
    }
    if (ieq(c.hdr, "*OPC") && c.query) {
        outf("1");
        return E_OK;
    }
    if (ieq(c.hdr, "HELP") && c.query) {
        outf("*IDN?,*RST,*OPC?,HELP?,PINS?,SYST:ERR?,SYST:BOOT,STREAM[?],"
             "TX:STATE[?],TX:TICK[?],TX:LOW[?],TX:NIB[?],TX:CRC[?],TX:FMT[?],TX:STATUS[?],TX:FC1[?],TX:FC2[?],"
             "TX:WAVE[?],TX:RAW[?],TX:PAUSE[?],TX:SLOW[?],TX:SLOW:ADD,TX:SLOW:CLR,TX:FAULT[?],TX:DRIVE[?],TX:INFO?,"
             "RXn:STATE[?],RXn:POL[?],RXn:TICK[?],RXn:NIB[?],RXn:CRC[?],RXn:FMT[?],RXn:PAUSE[?],RXn:CLR,"
             "RXn:LAST?,RXn:VAL?,RXn:STAT?,RXn:SLOW?,RXn:RAW?,RXn:CAP[?],RXn:CAP:DATA?,RXn:PULLTEST,"
             "BRIDGEn:STATE[?],BRIDGEn:MAP[?],BRIDGEn:FAIL[?],BRIDGEn:OUT?,DACn[?],DACn:MEAS?,"
             "CAN:STATE[?],CAN:BITRATE[?],CAN:MODE[?],CAN:MAP[?],CAN:SEND,CAN:STAT?,CAN:RX?");
        return E_OK;
    }
    if (ieq(c.hdr, "PINS") && c.query) {
        outf("%s", hw_pins());
        return E_OK;
    }
    if (ieq(c.hdr, "SYST:ERR") && c.query) {
        outf("%d,\"%s\"", S.last_err, err_text((err_t)S.last_err));
        S.last_err = 0;
        return E_OK;
    }
    if (ieq(c.hdr, "SYST:BOOT")) {
        hw_bootloader();
        outf("OK");
        return E_OK;
    }
    if (ieq(c.hdr, "STREAM")) {
        if (c.query) {
            outf("%u,%lu,dropped=%lu", S.stream_mask, (unsigned long)S.stream_every,
                 (unsigned long)S.stream_dropped);
            return E_OK;
        }
        char *av[3];
        unsigned n = split(c.arg, av, 3);
        uint32_t m, every = 1;
        bool b;
        if (n < 1) return E_MISSING;
        if (parse_bool(av[0], &b)) m = b ? 1u : 0u;
        else if (!parse_uint(av[0], &m) || m > 7) return E_RANGE;
        if (n >= 2 && (!parse_uint(av[1], &every) || every == 0)) return E_RANGE;
        S.stream_mask = (uint8_t)m;
        S.stream_every = every;
        S.stream_dropped = 0;
        outf("OK");
        return E_OK;
    }
    if ((sub = node(&c, "TX")) != NULL && c.idx < 0) {
        err_t e = cmd_tx(&c, sub);
        if (e == E_OK && !c.query) {
            outf("OK");
        }
        return e;
    }
    if ((sub = node(&c, "RX")) != NULL) {
        err_t e = cmd_rx(&c, sub);
        if (e == E_OK && !c.query) {
            outf("OK");
        }
        return e;
    }
    if ((sub = node(&c, "CAN")) != NULL && c.idx < 0) {
        err_t e = cmd_can(&c, sub);
        if (e == E_OK && !c.query) {
            outf("OK");
        }
        return e;
    }
    if ((sub = node(&c, "BRIDGE")) != NULL) {
        err_t e = cmd_bridge(&c, sub);
        if (e == E_OK && !c.query) {
            outf("OK");
        }
        return e;
    }
    if ((sub = node(&c, "DAC")) != NULL && ieq(sub, "MEAS") && c.query && c.idx >= 1 &&
        c.idx <= (int)HW_DAC_CHANNELS) {
        outf("%.4f", (double)hw_dac_meas((unsigned)c.idx - 1));
        return E_OK;
    }
    if ((sub = node(&c, "DAC")) != NULL && *sub == '\0' && c.idx >= 1 && c.idx <= (int)HW_DAC_CHANNELS) {
        double v;
        if (c.query) {
            outf("%.4f", (double)S.br[c.idx - 1].out);
            return E_OK;
        }
        if (!parse_num(c.arg, &v)) return E_DATA;
        if (v < 0 || v > hw_dac_max()) return E_RANGE;
        if (S.br[c.idx - 1].en) return E_STATE;
        S.br[c.idx - 1].out = (float)v;
        hw_dac((unsigned)c.idx - 1, (float)v);
        outf("OK");
        return E_OK;
    }
    return E_UNDEF;
}

void app_line(char *line)
{
    out_len = 0;
    out[0] = '\0';
    size_t n = strlen(line);
    while (n && (line[n - 1] == '\r' || line[n - 1] == '\n' || line[n - 1] == ' ')) {
        line[--n] = '\0';
    }
    if (n == 0) {
        return;
    }
    if (n > LINE_MAX) {
        outf("ERR %d,\"%s\"\n", E_OVERRUN, err_text(E_OVERRUN));
        hw_write(out, (unsigned)out_len);
        return;
    }
    char *p = line;
    bool first = true;
    while (p) {
        char *semi = strchr(p, ';');
        if (semi) {
            *semi = '\0';
        }
        if (!first) {
            outf(";");
        }
        first = false;
        err_t e = execute(p);
        if (e != E_OK) {
            S.last_err = e;
            outf("ERR %d,\"%s\"", e, err_text(e));
            break;
        }
        p = semi ? semi + 1 : NULL;
    }
    outf("\n");
    hw_write(out, (unsigned)out_len);
}

/* ---------------------------------------------------------- runtime */

static void stream_frame(unsigned ch, const sent_frame_t *f)
{
    char b[160];
    int n = snprintf(b, sizeof b, "#F %u,%.1f,%s,%X,", ch, (double)f->ts * 1e6 / (double)hw_rx_clk(),
                     sent_err_name((sent_err_t)f->err), f->status);
    for (unsigned i = 0; i < f->n && n < (int)sizeof b - 4; i++) {
        b[n++] = "0123456789ABCDEF"[f->d[i] & 0xF];
    }
    n += snprintf(b + n, sizeof b - (size_t)n, ",%X,%lu,%lu,%.1f,%u,%u\n", f->crc, (unsigned long)f->fc1,
                  (unsigned long)f->fc2, (double)f->tick_ns, f->flags, f->pause_ticks);
    if (n > 0) {
        hw_write(b, (unsigned)n);
    }
}

static void process_channel(unsigned ch)
{
    chan_t *x = &S.ch[ch];
    uint32_t ts[64];
    for (int round = 0; round < 32; round++) {
        uint32_t ovr = 0;
        unsigned n = hw_rx_read(ch, ts, 64, &ovr);
        if (ovr) {
            x->overruns += ovr;
            x->have_prev = false; /* lost edges: restart interval chain */
            if (CAP.active && CAP.ch == ch) {
                CAP.ovr += ovr;
            }
        }
        if (!n) {
            break;
        }
        if (!x->en) {
            continue;
        }
        if (CAP.active && CAP.ch == ch) {
            for (unsigned i = 0; i < n && CAP.active; i++) {
                if (CAP.n == 0 && !CAP.first) {
                    CAP.first = ts[i] | 1u; /* nonzero marker */
                    CAP.last  = ts[i];
                    continue;
                }
                CAP.d[CAP.n++] = ts[i] - CAP.last;
                CAP.last = ts[i];
                if (CAP.n + 1u >= CAP.target || CAP.n >= CAP_MAX) {
                    CAP.active = false;
                    CAP.done = true;
                    hw_rx_both_edges(ch, false, x->inv);
                    x->have_prev = false;
                }
            }
            continue;
        }
        for (unsigned i = 0; i < n; i++) {
            if (!x->have_prev) {
                x->prev_ts = ts[i];
                x->have_prev = true;
                continue;
            }
            uint32_t d = ts[i] - x->prev_ts;
            x->prev_ts = ts[i];
            x->raw[x->raw_pos] = d;
            x->raw_pos = (uint8_t)((x->raw_pos + 1u) % RAW_KEEP);
            sent_frame_t f;
            if (sent_rx_feed(&x->rx, d, &f)) {
                x->last = f;
                x->have_last = true;
                if (f.err == SENT_OK) {
                    x->last_ok = f;
                    x->have_ok = true;
                    x->last_ok_ms = hw_millis();
                    S.led_ms = hw_millis();
                }
                if (S.stream_mask & (1u << ch)) {
                    if (f.err != SENT_OK || ++S.stream_n[ch] >= S.stream_every) {
                        S.stream_n[ch] = 0;
                        stream_frame(ch, &f);
                    }
                }
            }
        }
    }
}

static void bridges(void)
{
    uint32_t now = hw_millis();
    for (unsigned i = 0; i < HW_DAC_CHANNELS; i++) {
        bridge_t *b = &S.br[i];
        if (!b->en) {
            continue;
        }
        const chan_t *x = &S.ch[b->ch];
        float v;
        if (x->en && x->have_ok && now - x->last_ok_ms <= b->timeout_ms) {
            float in = (float)(b->sig == 1 ? x->last_ok.fc1 : x->last_ok.fc2);
            float t = (in - b->in_min) / (b->in_max - b->in_min);
            if (t < 0) t = 0;
            if (t > 1) t = 1;
            v = b->v_min + t * (b->v_max - b->v_min);
            b->failed = false;
        } else {
            v = b->fail_v;
            b->failed = true;
        }
        if (v != b->out) {
            b->out = v;
            hw_dac(i, v);
        }
    }
}

/* SENT -> CAN: 8 bytes, big endian
 *  0-1 FC1, 2-3 FC2, 4 status nibble | 0x80 valid, 5 errors since the
 *  previous message (saturated), 6 rolling counter, 7 data age in ms */
static void can_gateway(void)
{
    if (!CAN.on) {
        return;
    }
    uint32_t id, now = hw_millis();
    uint8_t d[8], len;
    while (hw_can_recv(&id, d, &len)) {
        CAN.rx++;
        CAN.rx_id = id;
        CAN.rx_len = len;
        memcpy(CAN.rx_d, d, len);
    }
    for (unsigned ch = 0; ch < HW_RX_CHANNELS; ch++) {
        if (!CAN.id[ch] || now - CAN.last_ms[ch] < CAN.period_ms[ch]) {
            continue;
        }
        CAN.last_ms[ch] = now;
        const chan_t *x = &S.ch[ch];
        uint32_t age = x->have_ok ? now - x->last_ok_ms : 0xFFFFFFFFu;
        bool valid = x->en && x->have_ok && age <= 3u * CAN.period_ms[ch] + 10u;
        uint32_t errs = x->rx.st.frames - x->rx.st.ok;
        uint32_t de = errs - CAN.err_mark[ch];
        CAN.err_mark[ch] = errs;
        uint32_t f1 = valid ? x->last_ok.fc1 : 0, f2 = valid ? x->last_ok.fc2 : 0;
        uint8_t m[8] = {(uint8_t)(f1 >> 8), (uint8_t)f1, (uint8_t)(f2 >> 8), (uint8_t)f2,
                        (uint8_t)((valid ? 0x80u : 0u) | (x->last_ok.status & 0xFu)), (uint8_t)(de > 255 ? 255 : de),
                        CAN.counter[ch]++, (uint8_t)(age > 255 ? 255 : age)};
        if (hw_can_send(CAN.id[ch], m, 8)) {
            CAN.tx++;
        } else {
            CAN.tx_drop++;
        }
    }
}

void app_task(void)
{
    for (unsigned ch = 0; ch < HW_RX_CHANNELS; ch++) {
        process_channel(ch);
    }
    bridges();
    can_gateway();
    uint32_t now = hw_millis();
    /* LED: on while good frames arrive, slow blink when idle, fast if TX only */
    bool led = (now - S.led_ms < 50u) ? ((now / 25u) & 1u) : (S.tx_on ? ((now / 100u) & 1u) : ((now / 500u) & 1u));
    hw_led(led);
}

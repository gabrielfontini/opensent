/*
 * OpenSENT native unit tests (host build, no hardware).
 * SPDX-License-Identifier: MIT
 */
#include "sent.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_fail, g_checks;

#define CHECK(cond, ...)                                                    \
    do {                                                                    \
        g_checks++;                                                         \
        if (!(cond)) {                                                      \
            g_fail++;                                                       \
            printf("  FAIL %s:%d: %s -- ", __FILE__, __LINE__, #cond);     \
            printf(__VA_ARGS__);                                            \
            printf("\n");                                                   \
        }                                                                   \
    } while (0)

/* deterministic PRNG */
static uint32_t rng_state = 12345;
static uint32_t rnd(void)
{
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}
static double rnd_uniform(void) { return (double)rnd() / 4294967296.0; }

/* -------------------------------------------------------------- CRC */

static void test_crc(void)
{
    printf("crc\n");
    /* the 2010 CRC table used throughout the industry: crc of a single
     * nibble n starting at seed s=0 is table[n] with augmentation */
    static const uint8_t table[16] = {0, 13, 7, 10, 14, 3, 9, 4, 1, 12, 6, 11, 15, 2, 8, 5};
    /* table[i] == i * x^4 mod P; our legacy CRC with 1 nibble = table[5] ^ n */
    for (uint8_t n = 0; n < 16; n++) {
        CHECK(sent_crc4(SENT_CRC_LEGACY, 0, &n, 1) == (table[5] ^ n), "legacy n=%u", n);
        CHECK(sent_crc4(SENT_CRC_2010, 0, &n, 1) == table[table[5] ^ n], "2010 n=%u", n);
    }
    /* reference: table-driven loop as written in the J2716 appendix */
    for (int trial = 0; trial < 1000; trial++) {
        uint8_t d[6];
        for (int i = 0; i < 6; i++) {
            d[i] = rnd() & 0xF;
        }
        uint8_t c = 5;
        for (int i = 0; i < 6; i++) {
            c = d[i] ^ table[c];
        }
        uint8_t legacy = c;
        c = table[c];
        CHECK(sent_crc4(SENT_CRC_2010, 0, d, 6) == c, "2010 trial %d", trial);
        CHECK(sent_crc4(SENT_CRC_LEGACY, 0, d, 6) == legacy, "legacy trial %d", trial);
        uint8_t s = rnd() & 0xF;
        c = 5;
        c = table[c ^ s];
        for (int i = 0; i < 6; i++) {
            c = table[c ^ d[i]];
        }
        CHECK(sent_crc4(SENT_CRC_STATUS, s, d, 6) == c, "status trial %d", trial);
    }
    /* crc6 is affine: crc(x) = crc(0) ^ L(x); L(1) = x^6 mod (x^6+x^4+x^3+1) = 25 */
    uint8_t a = sent_crc6(0x000000), b = sent_crc6(0x000001), c2 = sent_crc6(0x000003);
    CHECK((a ^ b ^ sent_crc6(0x000002) ^ c2) == 0, "crc6 linear");
    CHECK((a ^ b) == 25, "crc6 x^6 term: got %u", (unsigned)(a ^ b));
}

/* --------------------------------------------------------- formats */

static void test_formats(void)
{
    printf("formats\n");
    for (unsigned id = 0; id < SENT_FMT_COUNT; id++) {
        sent_format_t f = sent_format((sent_format_id_t)id);
        for (int t = 0; t < 200; t++) {
            uint32_t m1 = f.fc1_nibbles ? (1u << (4 * f.fc1_nibbles)) - 1 : 0;
            uint32_t m2 = f.fc2_nibbles ? (1u << (4 * f.fc2_nibbles)) - 1 : 0;
            uint32_t a = rnd() & m1, b = rnd() & m2, a2, b2;
            uint8_t d[8];
            unsigned k = sent_pack(&f, a, b, d);
            CHECK(k == (unsigned)f.fc1_nibbles + f.fc2_nibbles, "len");
            CHECK(sent_unpack(&f, d, k, &a2, &b2), "unpack");
            CHECK(a == a2 && b == b2, "fmt %s %x/%x -> %x/%x", sent_format_name(id), a, b, a2, b2);
        }
    }
    /* standard 12+12: FC2 = 0x123 goes on the wire as 3,2,1 */
    sent_format_t f = sent_format(SENT_FMT_12_12);
    uint8_t d[6];
    sent_pack(&f, 0xABC, 0x123, d);
    CHECK(d[0] == 0xA && d[1] == 0xB && d[2] == 0xC && d[3] == 3 && d[4] == 2 && d[5] == 1, "12_12 order");
}

/* ------------------------------------------------- frame simulation */

/* Turn a tick list into clocks, with tick error (ppm-like factor) and
 * per-edge timing jitter (in ticks, uniform +/-). */
typedef struct {
    double clk_hz, tick_ns, factor, jitter_ticks;
    double edge;      /* time of last falling edge (clocks, ideal) */
    double last_meas; /* measured time of last falling edge */
} sim_t;

static uint32_t sim_pulse(sim_t *s, unsigned ticks)
{
    double len = ticks * s->tick_ns * s->factor * 1e-9 * s->clk_hz;
    s->edge += len;
    double j = (rnd_uniform() * 2.0 - 1.0) * s->jitter_ticks * s->tick_ns * 1e-9 * s->clk_hz;
    double meas = floor(s->edge + j);
    uint32_t out = (uint32_t)(meas - s->last_meas);
    s->last_meas = meas;
    return out;
}

typedef struct {
    const char *name;
    float tick_us;
    double factor;
    double jitter;
    uint8_t nibbles;
    uint8_t crc;
    uint8_t pause_mode;
    uint16_t pause_ticks;
    uint8_t rx_nibbles;   /* 0 = learn */
    float rx_tick_ns;     /* 0 = auto */
} rt_case_t;

static void roundtrip(const rt_case_t *tc)
{
    const uint32_t clk = 200000000u;
    sent_tx_cfg_t tcfg;
    sent_tx_cfg_default(&tcfg);
    tcfg.tick_ns     = tc->tick_us * 1000.0f;
    tcfg.nibbles     = tc->nibbles;
    tcfg.crc_mode    = tc->crc;
    tcfg.pause_mode  = tc->pause_mode;
    tcfg.pause_ticks = tc->pause_ticks;
    tcfg.fmt         = tc->nibbles >= 6 ? SENT_FMT_12_12 : (tc->nibbles >= 3 ? SENT_FMT_12 : SENT_FMT_RAW);
    tcfg.use_raw     = true;
    sent_tx_t tx;
    sent_tx_init(&tx, clk, &tcfg);

    sent_rx_cfg_t rcfg;
    sent_rx_cfg_default(&rcfg, clk);
    rcfg.nibbles = tc->rx_nibbles;
    rcfg.tick_ns = tc->rx_tick_ns;
    rcfg.fmt     = SENT_FMT_RAW;
    sent_rx_t rx;
    sent_rx_init(&rx, &rcfg);

    sim_t sim = {clk, tcfg.tick_ns, tc->factor, tc->jitter, 0, 0};
    /* start mid-frame: a few random nibble-like pulses first */
    sent_frame_t fr;
    for (int i = 0; i < 5; i++) {
        sent_rx_feed(&rx, sim_pulse(&sim, 12 + (rnd() % 16)), &fr);
    }

    enum { N = 400 };
    static uint8_t sent_d[N][SENT_MAX_DATA + 1];
    static uint8_t sent_s[N];
    int got = 0, ok = 0, mism = 0, first_ok = -1;
    for (int f = 0; f < N; f++) {
        for (unsigned i = 0; i < tcfg.nibbles; i++) {
            tx.cfg.raw[i] = rnd() & 0xF;
        }
        tx.cfg.status_bits = rnd() & 3;
        uint16_t ticks[SENT_MAX_DATA + 5];
        unsigned k = sent_tx_build_frame(&tx, ticks);
        memcpy(sent_d[f], tx.last_d, tx.last_n);
        sent_s[f] = tx.last_status;
        for (unsigned i = 0; i < k; i++) {
            if (sent_rx_feed(&rx, sim_pulse(&sim, ticks[i]), &fr)) {
                got++;
                /* the frame decoded at this sync is frame f-1 */
                int idx = (i == 0) ? f - 1 : f;
                if (fr.err == SENT_OK) {
                    ok++;
                    if (first_ok < 0) {
                        first_ok = idx;
                    }
                    if (idx < 0 || fr.status != sent_s[idx] || memcmp(fr.d, sent_d[idx], fr.n) != 0 ||
                        fr.n != tcfg.nibbles) {
                        mism++;
                    }
                }
            }
        }
    }
    int expected = N - 1 - (first_ok < 0 ? N : first_ok);
    CHECK(first_ok >= 0 && first_ok <= 4, "[%s] lock after %d frames", tc->name, first_ok);
    CHECK(mism == 0, "[%s] %d mismatching frames", tc->name, mism);
    CHECK(ok >= expected - 1, "[%s] ok %d expected %d (got %d, crc %u len %u nib %u sync %u)", tc->name, ok,
          expected, got, rx.st.err[SENT_E_CRC], rx.st.err[SENT_E_LENGTH], rx.st.err[SENT_E_NIBBLE],
          rx.st.err[SENT_E_SYNC]);
    if (tc->crc != SENT_CRC_STATUS || 1) {
        CHECK(rx.crc_lock == tc->crc || rx.crc_votes < 4, "[%s] crc lock %u", tc->name, rx.crc_lock);
    }
    printf("  %-34s ok %3d/%3d  tick %.3f us  dev %u mt  pauses %u\n", tc->name, ok, expected,
           fr.tick_ns / 1000.0f, fr.max_dev_mt, rx.st.pauses);
}

static void test_roundtrip(void)
{
    printf("roundtrip\n");
    static const rt_case_t cases[] = {
        {"3us 6nib 2010", 3.0f, 1.0, 0.05, 6, SENT_CRC_2010, SENT_PAUSE_OFF, 0, 6, 3000},
        {"3us 6nib learn auto-tick", 3.0f, 1.0, 0.05, 6, SENT_CRC_2010, SENT_PAUSE_OFF, 0, 0, 0},
        {"3us +19% clock", 3.0f, 1.19, 0.05, 6, SENT_CRC_2010, SENT_PAUSE_OFF, 0, 6, 3000},
        {"3us -19% clock", 3.0f, 0.81, 0.05, 6, SENT_CRC_2010, SENT_PAUSE_OFF, 0, 6, 3000},
        {"3us jitter 0.2 tick", 3.0f, 1.0, 0.20, 6, SENT_CRC_2010, SENT_PAUSE_OFF, 0, 6, 3000},
        {"legacy crc", 3.0f, 1.0, 0.05, 6, SENT_CRC_LEGACY, SENT_PAUSE_OFF, 0, 6, 0},
        {"status crc", 3.0f, 1.0, 0.05, 6, SENT_CRC_STATUS, SENT_PAUSE_OFF, 0, 6, 0},
        {"pause fixed 12 (nibble-like)", 3.0f, 1.0, 0.05, 6, SENT_CRC_2010, SENT_PAUSE_FIXED, 12, 6, 0},
        {"pause fixed 56 (sync-like)", 3.0f, 1.0, 0.05, 6, SENT_CRC_2010, SENT_PAUSE_FIXED, 56, 6, 0},
        {"pause fixed 50", 3.0f, 1.0, 0.05, 6, SENT_CRC_2010, SENT_PAUSE_FIXED, 50, 6, 0},
        {"pause fixed 300", 3.0f, 1.0, 0.05, 6, SENT_CRC_2010, SENT_PAUSE_FIXED, 300, 6, 0},
        {"pause const 282 learn", 3.0f, 1.0, 0.05, 6, SENT_CRC_2010, SENT_PAUSE_CONST, 282, 0, 0},
        {"pause const 300 auto-tick", 3.0f, 1.0, 0.05, 6, SENT_CRC_2010, SENT_PAUSE_CONST, 300, 0, 0},
        {"pause 768 auto-tick", 10.0f, 1.0, 0.05, 6, SENT_CRC_2010, SENT_PAUSE_FIXED, 768, 0, 0},
        {"pause fixed 20 learn", 3.0f, 1.0, 0.05, 6, SENT_CRC_2010, SENT_PAUSE_FIXED, 20, 0, 0},
        {"90us tick", 90.0f, 1.0, 0.05, 6, SENT_CRC_2010, SENT_PAUSE_OFF, 0, 6, 90000},
        {"10us auto", 10.0f, 1.0, 0.05, 6, SENT_CRC_2010, SENT_PAUSE_OFF, 0, 0, 0},
        {"4 nibbles", 3.0f, 1.0, 0.05, 4, SENT_CRC_2010, SENT_PAUSE_OFF, 0, 4, 0},
        {"3 nibbles learn", 3.0f, 1.0, 0.05, 3, SENT_CRC_2010, SENT_PAUSE_OFF, 0, 0, 0},
        {"8 nibbles", 3.0f, 1.0, 0.05, 8, SENT_CRC_2010, SENT_PAUSE_OFF, 0, 8, 0},
    };
    for (unsigned i = 0; i < sizeof cases / sizeof cases[0]; i++) {
        rng_state = 1000 + i;
        roundtrip(&cases[i]);
    }
}

/* --------------------------------------------- timing of sent_tx_next */

static void test_tx_timing(void)
{
    printf("tx timing\n");
    sent_tx_cfg_t c;
    sent_tx_cfg_default(&c);
    c.tick_ns = 3333.3f; /* not an integer number of 200 MHz clocks */
    sent_tx_t tx;
    sent_tx_init(&tx, 200000000u, &c);
    double total_clk = 0;
    uint64_t total_ticks = 0;
    uint32_t low = 0;
    for (int i = 0; i < 9000; i++) {
        unsigned pos = tx.pos >= tx.nticks ? 0 : tx.pos;
        uint32_t p = sent_tx_next(&tx, &low);
        if (pos == 0) {
            pos = 0;
        }
        total_clk += p;
        total_ticks += tx.ticks[tx.pos - 1];
        CHECK(low < p, "low < period");
    }
    double avg_tick_ns = total_clk / (double)total_ticks / 200e6 * 1e9;
    CHECK(fabs(avg_tick_ns - 3333.3) < 0.01, "average tick %.4f ns", avg_tick_ns);
    CHECK(low == (uint32_t)(5 * 3333.3 * 0.2 + 0.5), "low %u", low);
    printf("  average tick %.4f ns over %llu ticks\n", avg_tick_ns, (unsigned long long)total_ticks);
}

/* ------------------------------------------------------ slow channel */

static void run_slow(sent_slow_fmt_t fmt, const uint8_t *ids, const uint16_t *vals, unsigned n, bool corrupt)
{
    const uint32_t clk = 200000000u;
    sent_tx_cfg_t c;
    sent_tx_cfg_default(&c);
    c.slow_fmt = fmt;
    c.slow_n   = (uint8_t)n;
    for (unsigned i = 0; i < n; i++) {
        c.slow_id[i]  = ids[i];
        c.slow_val[i] = vals[i];
    }
    sent_tx_t tx;
    sent_tx_init(&tx, clk, &c);
    if (corrupt) {
        tx.fault       = SENT_FAULT_SCRC;
        tx.fault_count = 1;
    }
    sent_rx_cfg_t rc;
    sent_rx_cfg_default(&rc, clk);
    sent_rx_t rx;
    sent_rx_init(&rx, &rc);
    sent_frame_t fr;
    unsigned frames = (fmt == SENT_SLOW_SHORT ? 16 : 18) * n * 3 + 40;
    for (unsigned i = 0; i < frames * 9; i++) {
        sent_rx_feed(&rx, sent_tx_next(&tx, NULL), &fr);
    }
    unsigned found = 0;
    for (unsigned i = 0; i < n; i++) {
        for (unsigned j = 0; j < rx.slow.nmsg; j++) {
            const sent_slow_msg_t *m = &rx.slow.msg[j];
            if (m->id == ids[i] && m->fmt == fmt) {
                CHECK(m->value == vals[i], "%s id %u value %u != %u", sent_slow_name(fmt), ids[i], m->value,
                      vals[i]);
                found++;
            }
        }
    }
    CHECK(found == n, "%s: found %u of %u messages", sent_slow_name(fmt), found, n);
    CHECK(rx.slow.nmsg == n, "%s: table has %u entries", sent_slow_name(fmt), rx.slow.nmsg);
    if (corrupt) {
        CHECK(rx.slow.crc_err == 1, "%s: slow crc errors %u", sent_slow_name(fmt), rx.slow.crc_err);
    } else {
        CHECK(rx.slow.crc_err == 0, "%s: slow crc errors %u", sent_slow_name(fmt), rx.slow.crc_err);
    }
    printf("  %-6s %u msgs ok %u crc_err %u\n", sent_slow_name(fmt), n, rx.slow.ok, rx.slow.crc_err);
}

static void test_slow(void)
{
    printf("slow channel\n");
    const uint8_t  ids_s[] = {0x1, 0x3, 0x7, 0xF, 0x0};
    const uint16_t val_s[] = {0x00, 0xFF, 0x5A, 0x81, 0x33};
    run_slow(SENT_SLOW_SHORT, ids_s, val_s, 5, false);
    run_slow(SENT_SLOW_SHORT, ids_s, val_s, 5, true);
    const uint8_t  ids_e[] = {0x01, 0x23, 0x90, 0xFF, 0x00};
    const uint16_t val_e[] = {0x000, 0xFFF, 0xA5A, 0x123, 0x800};
    run_slow(SENT_SLOW_ENH12, ids_e, val_e, 5, false);
    run_slow(SENT_SLOW_ENH12, ids_e, val_e, 5, true);
    const uint8_t  ids_16[] = {0x1, 0x2, 0xF, 0x0};
    const uint16_t val_16[] = {0xFFFF, 0x1234, 0x8001, 0x0000};
    run_slow(SENT_SLOW_ENH16, ids_16, val_16, 4, false);
}

/* ------------------------------------------------------- faults */

static void fault_case(sent_fault_t fault, sent_err_t expect, bool no_pause)
{
    const uint32_t clk = 200000000u;
    sent_tx_cfg_t c;
    sent_tx_cfg_default(&c);
    sent_tx_t tx;
    sent_tx_init(&tx, clk, &c);
    sent_rx_cfg_t rc;
    sent_rx_cfg_default(&rc, clk);
    rc.nibbles  = 6;
    rc.tick_ns  = 3000;
    rc.no_pause = no_pause;
    sent_rx_t rx;
    sent_rx_init(&rx, &rc);
    sent_frame_t fr;
    unsigned pulses = 0;
    /* 20 clean frames, then 5 faults every 10 frames, then 20 clean */
    for (int i = 0; i < 20 * 9; i++, pulses++) {
        sent_rx_feed(&rx, sent_tx_next(&tx, NULL), &fr);
    }
    uint32_t ok_before = rx.st.ok;
    tx.fault_every = 10;
    tx.fault_count = 5;
    tx.fault       = (uint8_t)fault;
    for (int i = 0; i < 70 * 9; i++) {
        sent_rx_feed(&rx, sent_tx_next(&tx, NULL), &fr);
    }
    (void)ok_before;
    CHECK(tx.faults_injected == 5, "%s injected %u", sent_fault_name(fault), tx.faults_injected);
    if (fault == SENT_FAULT_CAL) {
        CHECK(rx.st.cal_flags >= 5 && rx.st.cal_flags <= 10, "CAL flags %u", rx.st.cal_flags);
        CHECK(rx.st.ok >= rx.st.frames - 1, "CAL frames still decode: ok %u frames %u", rx.st.ok, rx.st.frames);
    } else {
        CHECK(rx.st.err[expect] == 5, "%s -> %s count %u (crc %u nib %u len %u short %u sync %u)",
              sent_fault_name(fault), sent_err_name(expect), rx.st.err[expect], rx.st.err[SENT_E_CRC],
              rx.st.err[SENT_E_NIBBLE], rx.st.err[SENT_E_LENGTH], rx.st.err[SENT_E_SHORT],
              rx.st.err[SENT_E_SYNC]);
    }
    /* decoder must recover: the last 10 frames are clean */
    uint32_t ok_mid = rx.st.ok;
    for (int i = 0; i < 10 * 9; i++) {
        sent_rx_feed(&rx, sent_tx_next(&tx, NULL), &fr);
    }
    CHECK(rx.st.ok - ok_mid >= 9, "%s: recovered %u", sent_fault_name(fault), rx.st.ok - ok_mid);
    printf("  %-7s -> %-6s frames %u ok %u errs crc %u nib %u len %u sync %u cal %u\n", sent_fault_name(fault),
           sent_err_name(expect), rx.st.frames, rx.st.ok, rx.st.err[SENT_E_CRC], rx.st.err[SENT_E_NIBBLE],
           rx.st.err[SENT_E_LENGTH], rx.st.err[SENT_E_SYNC], rx.st.cal_flags);
}

static void test_faults(void)
{
    printf("fault injection\n");
    fault_case(SENT_FAULT_CRC, SENT_E_CRC, false);
    fault_case(SENT_FAULT_NIBBLE, SENT_E_NIBBLE, false);
    fault_case(SENT_FAULT_SHORT, SENT_E_LENGTH, false);
    fault_case(SENT_FAULT_LONG, SENT_E_LENGTH, true);
    fault_case(SENT_FAULT_SYNC, SENT_E_SYNC, false);
    fault_case(SENT_FAULT_CAL, SENT_OK, false);
}

/* --------------------------------------------- waveforms / tx misc */

static void test_waves(void)
{
    printf("waveforms\n");
    sent_tx_cfg_t c;
    sent_tx_cfg_default(&c);
    c.fc[0].kind = SENT_WAVE_TRI;
    c.fc[0].min = 100;
    c.fc[0].max = 4000;
    c.fc[0].period_s = 0.1f;
    c.fc[1].kind = SENT_WAVE_COUNT_UP;
    c.fc[1].min = 0;
    c.fc[1].max = 4095;
    sent_tx_t tx;
    sent_tx_init(&tx, 200000000u, &c);
    uint32_t lo = 0xFFFFFFFF, hi = 0, prev2 = 0;
    bool counting = true;
    uint16_t ticks[16];
    for (int i = 0; i < 2000; i++) {
        unsigned k = sent_tx_build_frame(&tx, ticks);
        for (unsigned j = 0; j < k; j++) {
            tx.t_s += ticks[j] * 3e-6;
        }
        if (tx.last_fc1 < lo) lo = tx.last_fc1;
        if (tx.last_fc1 > hi) hi = tx.last_fc1;
        if (i > 0 && tx.last_fc2 != ((prev2 + 1) & 0xFFF)) counting = false;
        prev2 = tx.last_fc2;
    }
    CHECK(lo <= 110 && hi >= 3990 && hi <= 4000 && lo >= 100, "tri range %u..%u", lo, hi);
    CHECK(counting, "counter increments");
}

/* ------------------------------------------- real sensor captures */

typedef struct {
    const char *file;
    const char *what;
} capture_t;

static int replay_csv(const char *path, sent_rx_t *rx, bool quiet)
{
    FILE *fp = fopen(path, "r");
    if (!fp) {
        return -1;
    }
    char line[256];
    double prev_t = -1, t;
    int v, prev_v = -1;
    sent_frame_t fr;
    int frames = 0;
    if (!fgets(line, sizeof line, fp)) { /* header */
        fclose(fp);
        return -1;
    }
    while (fgets(line, sizeof line, fp)) {
        if (sscanf(line, "%lf , %d", &t, &v) != 2 && sscanf(line, "%lf,%d", &t, &v) != 2) {
            continue;
        }
        if (v == 0 && prev_v == 1) { /* falling edge */
            if (prev_t >= 0) {
                uint32_t clk = (uint32_t)llround((t - prev_t) * (double)rx->cfg.clk_hz);
                if (sent_rx_feed(rx, clk, &fr)) {
                    frames++;
                    if (!quiet && frames <= 3 && fr.err == SENT_OK) {
                        printf("      S=%X D=", fr.status);
                        for (unsigned i = 0; i < fr.n; i++) printf("%X", fr.d[i]);
                        printf(" C=%X fc1=%u fc2=%u crc=%s tick=%.3fus\n", fr.crc, fr.fc1, fr.fc2,
                               sent_crc_name(fr.crc_mode), fr.tick_ns / 1000.0f);
                    }
                }
            }
            prev_t = t;
        }
        prev_v = v;
    }
    fclose(fp);
    return frames;
}

static void test_captures(const char *dir)
{
    printf("real sensor captures (%s)\n", dir);
    static const capture_t caps[] = {
        {"SENT-fuel-pressure.csv", "GM GDI fuel pressure"},
        {"ford-sent-idle.csv", "Ford throttle idle"},
        {"ford-sent-closed.csv", "Ford throttle closed"},
        {"opel-throttle-idle.csv", "Opel throttle idle"},
        {"opel-throttle-move.csv", "Opel throttle moving"},
        {"vag_04e.906.051.csv", "VAG 04E906051 MAP"},
        {"mb_A_000_905_31_01-ambient.csv", "Mercedes EGT"},
    };
    int seen = 0;
    for (unsigned i = 0; i < sizeof caps / sizeof caps[0]; i++) {
        char path[512];
        snprintf(path, sizeof path, "%s/%s", dir, caps[i].file);
        sent_rx_cfg_t rc;
        sent_rx_cfg_default(&rc, 1000000000u); /* timestamps in ns */
        static sent_rx_t rx;
        sent_rx_init(&rx, &rc);
        int frames = replay_csv(path, &rx, false);
        if (frames < 0) {
            continue;
        }
        seen++;
        double okp = rx.st.frames ? 100.0 * rx.st.ok / rx.st.frames : 0;
        printf("  %-22s frames %4u ok %5.1f%% n=%u crc=%s pauses %u cal %u margin %u slow ok %u err %u ids %u\n",
               caps[i].what, rx.st.frames, okp, rx.learned_n, sent_crc_name(rx.crc_lock), rx.st.pauses,
               rx.st.cal_flags, rx.st.margin_flags, rx.slow.ok, rx.slow.crc_err, rx.slow.nmsg);
        for (unsigned j = 0; j < rx.slow.nmsg && j < 6; j++) {
            printf("      slow %-5s id 0x%02X = 0x%04X (%u)\n", sent_slow_name(rx.slow.msg[j].fmt),
                   rx.slow.msg[j].id, rx.slow.msg[j].value, rx.slow.msg[j].count);
        }
        CHECK(rx.st.frames > 50, "%s: frames %u", caps[i].file, rx.st.frames);
        CHECK(okp > 95.0, "%s: ok %.1f%%", caps[i].file, okp);
    }
    if (!seen) {
        printf("  (skipped: run tools/fetch_captures.sh to download them)\n");
    }
}

int main(int argc, char **argv)
{
    test_crc();
    test_formats();
    test_tx_timing();
    test_roundtrip();
    test_slow();
    test_faults();
    test_waves();
    test_captures(argc > 1 ? argv[1] : "tests/data");
    printf("\n%d checks, %d failures\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}

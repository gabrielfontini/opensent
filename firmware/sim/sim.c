/*
 * OpenSENT host simulator: runs the real instrument application on Linux
 * behind a pseudo terminal, so the PC tools can be developed and tested
 * without hardware. The SENT transmitter runs in virtual time and its
 * edges are delivered to the TX monitor channel (RX2) and, through a
 * virtual jumper wire, to RX0.
 *
 *   opensent-sim [--link /tmp/opensent]   prints the pty path
 *
 * SPDX-License-Identifier: MIT
 */
#define _XOPEN_SOURCE 600
#define _DEFAULT_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "app.h"
#include "hw.h"

#define CLK 200000000u
#define RING 8192u

static int      master = -1;
static uint64_t t0_ns;
static sent_tx_t *tx;
static uint64_t tx_time; /* virtual clocks */
static bool     rx_en[HW_RX_CHANNELS];
static uint32_t ring[HW_RX_CHANNELS][RING];
static unsigned head[HW_RX_CHANNELS], tail[HW_RX_CHANNELS];
static uint32_t ovr[HW_RX_CHANNELS];
static int      jumper = 1;
static bool     both[HW_RX_CHANNELS];
static bool     line_level = true;
static float    dac[HW_DAC_CHANNELS];

static uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static uint64_t virt_clk(void) { return (now_ns() - t0_ns) * (CLK / 1000000u) / 1000u; }

void        hw_init(void) { t0_ns = now_ns(); }
uint32_t    hw_millis(void) { return (uint32_t)((now_ns() - t0_ns) / 1000000u); }
void        hw_led(bool on) { (void)on; }
const char *hw_board(void) { return "SIM"; }
const char *hw_uid(void) { return "000000000000"; }
const char *hw_pins(void) { return "TX=virtual RX0=virtual-jumper RX1=none RX2=TX-monitor DAC1=virtual DAC2=virtual"; }
uint32_t    hw_tx_clk(void) { return CLK; }
uint32_t    hw_rx_clk(void) { return CLK; }
void        hw_tx_lock(void) {}
void        hw_tx_unlock(void) {}
void        hw_tx_drive(bool od) { (void)od; }
float       hw_dac_max(void) { return 3.3f; }
void        hw_dac(unsigned ch, float v) { if (ch < HW_DAC_CHANNELS) dac[ch] = v; }
void        hw_bootloader(void) {}
float       hw_dac_meas(unsigned ch) { return ch < HW_DAC_CHANNELS ? dac[ch] : -1.0f; }

void hw_write(const char *s, unsigned n)
{
    while (n) {
        ssize_t w = write(master, s, n);
        if (w < 0) {
            if (errno == EAGAIN) {
                struct pollfd p = {master, POLLOUT, 0};
                poll(&p, 1, 10);
                continue;
            }
            return;
        }
        s += w;
        n -= (unsigned)w;
    }
}

void hw_tx_start(sent_tx_t *t)
{
    tx = t;
    tx_time = virt_clk();
    /* the first falling edge starts the first sync */
    for (unsigned ch = 0; ch < HW_RX_CHANNELS; ch++) {
        (void)ch;
    }
}

void hw_tx_stop(void) { tx = NULL; line_level = true; }
void hw_rx_both_edges(unsigned ch, bool b, bool inv) { (void)inv; if (ch < HW_RX_CHANNELS) both[ch] = b; }
bool hw_rx_pin(unsigned ch) { (void)ch; return line_level; }

void hw_rx_enable(unsigned ch, bool en, bool invert)
{
    (void)invert;
    if (ch < HW_RX_CHANNELS) {
        rx_en[ch] = en;
        head[ch] = tail[ch] = 0;
    }
}

static void push(unsigned ch, uint32_t ts)
{
    if (!rx_en[ch]) {
        return;
    }
    unsigned nh = (head[ch] + 1) % RING;
    if (nh == tail[ch]) {
        ovr[ch]++;
        return;
    }
    ring[ch][head[ch]] = ts;
    head[ch] = nh;
}

static void sim_advance(void)
{
    if (!tx) {
        return;
    }
    uint64_t target = virt_clk();
    if (tx_time < target && target - tx_time > CLK / 20) { /* never catch up more than 50 ms */
        tx_time = target - CLK / 20;
    }
    while (tx_time < target) {
        /* falling edge at tx_time, then the pulse lasts p clocks */
        uint32_t ts = (uint32_t)tx_time, low = 0;
        push(2, ts);
        if (jumper) {
            push(0, ts + 3); /* a few ns of cable/filter delay */
        }
        uint32_t p = sent_tx_next(tx, &low);
        if (both[2]) {
            push(2, ts + low);
        }
        if (jumper && both[0]) {
            push(0, ts + low + 3);
        }
        line_level = true;
        tx_time += p;
    }
}

bool hw_rx_pulltest(unsigned ch, sent_tx_t *gen, unsigned frames)
{
    uint16_t ticks[SENT_MAX_DATA + 5];
    uint64_t t = virt_clk();
    double clk_per_tick = gen->cfg.tick_ns * 1e-9 * CLK;
    for (unsigned f = 0; f < frames; f++) {
        unsigned k = sent_tx_build_frame(gen, ticks);
        for (unsigned i = 0; i < k; i++) {
            push(ch, (uint32_t)t);
            t += (uint64_t)(ticks[i] * clk_per_tick);
        }
        push(ch, (uint32_t)t);
        t += CLK / 10000; /* 100 us gap */
    }
    return true;
}

unsigned hw_rx_read(unsigned ch, uint32_t *ts, unsigned max, uint32_t *overruns)
{
    if (ch == 0) {
        sim_advance();
    }
    unsigned n = 0;
    while (n < max && tail[ch] != head[ch]) {
        ts[n++] = ring[ch][tail[ch]];
        tail[ch] = (tail[ch] + 1) % RING;
    }
    *overruns = ovr[ch];
    ovr[ch] = 0;
    return n;
}

int main(int argc, char **argv)
{
    const char *link = NULL;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--link") && i + 1 < argc) {
            link = argv[++i];
        } else if (!strcmp(argv[i], "--no-jumper")) {
            jumper = 0;
        }
    }
    signal(SIGPIPE, SIG_IGN);
    master = posix_openpt(O_RDWR | O_NOCTTY);
    if (master < 0 || grantpt(master) || unlockpt(master)) {
        perror("pty");
        return 1;
    }
    const char *slave = ptsname(master);
    int sfd = open(slave, O_RDWR | O_NOCTTY); /* keep open: no HUP when clients reconnect */
    struct termios tio;
    tcgetattr(sfd, &tio);
    cfmakeraw(&tio);
    tcsetattr(sfd, TCSANOW, &tio);
    fcntl(master, F_SETFL, O_NONBLOCK);
    if (link) {
        unlink(link);
        if (symlink(slave, link)) {
            perror("symlink");
        }
    }
    printf("%s\n", slave);
    fflush(stdout);

    hw_init();
    app_init();
    char line[512];
    size_t len = 0;
    for (;;) {
        struct pollfd p = {master, POLLIN, 0};
        poll(&p, 1, 1);
        char buf[256];
        ssize_t r = read(master, buf, sizeof buf);
        for (ssize_t i = 0; i < r; i++) {
            char c = buf[i];
            if (c == '\n' || c == '\r') {
                line[len] = '\0';
                if (len) {
                    app_line(line);
                }
                len = 0;
            } else if (len < sizeof line - 1) {
                line[len++] = c;
            }
        }
        app_task();
    }
}

/*
 * OpenSENT logic analyzer: SUMP / Openbench Logic Sniffer protocol on the
 * second USB CDC port, so sigrok / PulseView drive it with their stock
 * "ols" driver.
 *
 * 16 channels = one whole GPIO port (default GPIOB, so channel 10 is the
 * SENT TX pin). TIM8 update events trigger DMA2 Stream0, which copies
 * GPIOx->IDR into a 128 K-sample ring in AXI SRAM (DMA double-buffer
 * mode, because one stream counts at most 65535 items). The CPU
 * watches the incoming samples for the parallel trigger (stage 0
 * mask/value), lets the post-trigger part complete, stops, and streams the
 * buffer to the host newest sample first, as the protocol requires.
 *
 * SPDX-License-Identifier: MIT
 */
#include "la.h"

#include <string.h>

#include "stm32h7xx.h"
#include "tusb.h"

#include "clock.h"

#define LA_HALF       65532u          /* NDTR is 16 bit: double-buffer mode, two halves */
#define LA_N          (2u * LA_HALF)  /* ring of 131064 samples (16 bit) */
#define LA_ITF        1u
#define DMAMUX_TIM8_UP 51u
#define BASE_CLOCK    100000000u /* SUMP reference clock: rate = 100 MHz / (divider + 1) */

/* in D2 SRAM1/2, the DMA2 domain: no contention with the CPU's AXI traffic */
__attribute__((section(".sram12"))) static uint16_t buf[LA_N];

typedef enum { ST_IDLE, ST_ARMED, ST_POST, ST_SEND } state_t;

static struct {
    state_t  st;
    GPIO_TypeDef *port;
    char     port_name;
    uint32_t divider;
    uint32_t readcount, delaycount; /* in samples */
    uint32_t mask, value;
    uint32_t flags;
    volatile uint32_t laps;       /* completed halves */
    uint32_t scanned;     /* absolute index of the next sample to check for the trigger */
    uint32_t trig;        /* absolute index of the trigger sample */
    uint32_t stop_at;     /* absolute index after the last sample to keep */
    uint32_t send_left;   /* samples still to send */
    uint32_t send_idx;    /* absolute index of the next sample to send (counts down) */
    uint8_t  groups;      /* enabled channel groups bit mask (bit0 = ch 0-7, bit1 = ch 8-15) */
    uint8_t  cmd[5];
    uint8_t  cmd_len;
    uint32_t captures;
    uint32_t rate;
} L;

void DMA2_Stream0_IRQHandler(void)
{
    if (DMA2->LISR & DMA_LISR_TCIF0) {
        DMA2->LIFCR = DMA_LIFCR_CTCIF0;
        L.laps++;
    }
}

static uint32_t written(void)
{
    uint32_t laps, ndtr, f1, f2;
    __disable_irq();
    do {
        f1 = DMA2->LISR & DMA_LISR_TCIF0;
        ndtr = DMA2_Stream0->NDTR;
        f2 = DMA2->LISR & DMA_LISR_TCIF0;
    } while (f1 != f2);
    laps = L.laps + (f1 ? 1u : 0u);
    __enable_irq();
    return laps * LA_HALF + (LA_HALF - ndtr);
}

static uint32_t s_last_isr, s_last_written;

static void stop_sampling(void)
{
    s_last_isr = DMA2->LISR & 0x3Du;
    s_last_written = L.st != ST_IDLE ? written() : s_last_written;
    TIM8->CR1 &= ~TIM_CR1_CEN;
    TIM8->DIER = 0;
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream0->CR & DMA_SxCR_EN) {
    }
    DMA2->LIFCR = 0x3Du;
}

static void start_sampling(void)
{
    stop_sampling();
    uint32_t arr = 2u * (L.divider + 1u) - 1u; /* TIM8 runs at 200 MHz */
    L.rate = BASE_CLOCK / (L.divider + 1u);
    TIM8->CR1 = 0;
    TIM8->PSC = 0;
    TIM8->ARR = arr;
    TIM8->CNT = 0;
    TIM8->EGR = TIM_EGR_UG;
    TIM8->SR = 0;
    L.laps = 0;
    DMA2_Stream0->PAR  = (uint32_t)&L.port->IDR;
    DMA2_Stream0->M0AR = (uint32_t)buf;
    DMA2_Stream0->M1AR = (uint32_t)(buf + LA_HALF);
    DMA2_Stream0->NDTR = LA_HALF;
    DMA2_Stream0->FCR  = 0;
    DMA2_Stream0->CR   = DMA_SxCR_DBM | DMA_SxCR_CIRC | DMA_SxCR_MINC | DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_TCIE |
                         DMA_SxCR_PL_1 | DMA_SxCR_PL_0;
    DMAMUX1_Channel8->CCR = DMAMUX_TIM8_UP;
    DMA2_Stream0->CR |= DMA_SxCR_EN;
    TIM8->DIER = TIM_DIER_UDE;
    TIM8->CR1 = TIM_CR1_CEN;
}

static void send_bytes(const void *p, uint32_t n)
{
    const uint8_t *b = p;
    uint32_t t0 = 0;
    while (n && tud_cdc_n_connected(LA_ITF)) {
        uint32_t w = tud_cdc_n_write(LA_ITF, b, n);
        b += w;
        n -= w;
        if (n) {
            tud_cdc_n_write_flush(LA_ITF);
            tud_task();
            if (++t0 > 200000u) {
                return;
            }
        }
    }
    tud_cdc_n_write_flush(LA_ITF);
}

static void put_u32be(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

static void metadata(void)
{
    uint8_t m[96];
    unsigned k = 0;
    const char *name = "OpenSENT-LA";
    const char *ver = "0.1.0";
    m[k++] = 0x01;
    memcpy(&m[k], name, strlen(name) + 1);
    k += (unsigned)strlen(name) + 1;
    m[k++] = 0x02;
    memcpy(&m[k], ver, strlen(ver) + 1);
    k += (unsigned)strlen(ver) + 1;
    m[k++] = 0x20; put_u32be(&m[k], 16); k += 4;              /* probes */
    m[k++] = 0x21; put_u32be(&m[k], LA_N); k += 4;            /* sample memory (samples) */
    m[k++] = 0x23; put_u32be(&m[k], LA_MAX_RATE); k += 4;     /* max sample rate */
    m[k++] = 0x24; put_u32be(&m[k], 2); k += 4;               /* protocol version */
    m[k++] = 0x00;
    send_bytes(m, k);
}

static void reset_state(void)
{
    stop_sampling();
    L.st = ST_IDLE;
}

static void arm(void)
{
    if (L.readcount < 4 || L.readcount > LA_N) {
        L.readcount = L.readcount < 4 ? 4 : LA_N;
    }
    if (L.delaycount > L.readcount) {
        L.delaycount = L.readcount;
    }
    L.groups = (uint8_t)(~(L.flags >> 2) & 0x3u);
    if (!L.groups) {
        L.groups = 3;
    }
    L.scanned = L.readcount - L.delaycount; /* need this many pre-trigger samples first */
    start_sampling();
    L.st = ST_ARMED;
    L.captures++;
}

static void long_cmd(const uint8_t *c)
{
    uint32_t a = (uint32_t)c[1] | ((uint32_t)c[2] << 8) | ((uint32_t)c[3] << 16) | ((uint32_t)c[4] << 24);
    switch (c[0]) {
    case 0x80: { /* divider: rate = 100 MHz / (div + 1), limited to what we can do */
        uint32_t d = a & 0xFFFFFFu;
        uint32_t min_div = BASE_CLOCK / LA_MAX_RATE - 1u;
        L.divider = d < min_div ? min_div : d;
        break;
    }
    case 0x81: /* 16-bit read / delay counts in units of 4 samples */
        L.readcount = ((a & 0xFFFFu) + 1u) * 4u;
        L.delaycount = ((a >> 16) + 1u) * 4u;
        break;
    case 0x83: L.delaycount = (a + 1u) * 4u; break;
    case 0x84: L.readcount = (a + 1u) * 4u; break;
    case 0x82: L.flags = a; break;
    case 0xC0: L.mask = a; break;   /* stage 0 only: parallel trigger */
    case 0xC1: L.value = a; break;
    default: break;                  /* other stages / config: accepted, ignored */
    }
}

void la_rx(const uint8_t *p, unsigned n)
{
    for (unsigned i = 0; i < n; i++) {
        uint8_t b = p[i];
        if (L.cmd_len == 0) {
            if (b & 0x80) {
                L.cmd[0] = b;
                L.cmd_len = 1;
                continue;
            }
            switch (b) {
            case 0x00: reset_state(); break;
            case 0x01: arm(); break;
            case 0x02: send_bytes("1ALS", 4); break;
            case 0x04: metadata(); break;
            default: break; /* XON/XOFF, test mode: ignored */
            }
        } else {
            L.cmd[L.cmd_len++] = b;
            if (L.cmd_len == 5) {
                long_cmd(L.cmd);
                L.cmd_len = 0;
            }
        }
    }
}

void la_poll(void)
{
    if (L.st == ST_ARMED) {
        uint32_t w = written();
        if (w > L.scanned + LA_N - 1024u) {
            L.scanned = w - (LA_N - 1024u); /* don't fall behind the ring */
        }
        const uint32_t m = L.mask & 0xFFFFu, v = L.value & m;
        while (L.scanned < w) {
            if ((buf[L.scanned % LA_N] & m) == v) {
                L.trig = L.scanned;
                L.stop_at = L.trig + L.delaycount;
                L.st = ST_POST;
                break;
            }
            L.scanned++;
            if (!m) {
                break;
            }
        }
    }
    if (L.st == ST_POST) {
        if (written() >= L.stop_at) {
            stop_sampling();
            L.send_left = L.readcount;
            L.send_idx = L.stop_at - 1u; /* newest first */
            L.st = ST_SEND;
        }
    }
    if (L.st == ST_SEND) {
        /* stream in chunks so the SENT decoders keep running */
        uint8_t chunk[256];
        unsigned k = 0;
        const unsigned per = (L.groups & 1u ? 1u : 0u) + (L.groups & 2u ? 1u : 0u);
        while (L.send_left && k + per <= sizeof chunk && tud_cdc_n_write_available(LA_ITF) > k + per) {
            uint16_t s = buf[L.send_idx % LA_N];
            if (L.groups & 1u) {
                chunk[k++] = (uint8_t)s;
            }
            if (L.groups & 2u) {
                chunk[k++] = (uint8_t)(s >> 8);
            }
            L.send_idx--;
            L.send_left--;
        }
        if (k) {
            tud_cdc_n_write(LA_ITF, chunk, k);
            tud_cdc_n_write_flush(LA_ITF);
        }
        if (!L.send_left || !tud_cdc_n_connected(LA_ITF)) {
            L.st = ST_IDLE;
        }
    }
}

bool la_set_port(char name)
{
    GPIO_TypeDef *p;
    switch (name) {
    case 'A': p = GPIOA; break;
    case 'B': p = GPIOB; break;
    case 'C': p = GPIOC; break;
    case 'D': p = GPIOD; break;
    case 'E': p = GPIOE; break;
    default: return false;
    }
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOCEN | RCC_AHB4ENR_GPIODEN |
                    RCC_AHB4ENR_GPIOEEN;
    L.port = p;
    L.port_name = name;
    return true;
}

char la_port(void)
{
    return L.port_name;
}

const char *la_state(void)
{
    static const char *const n[] = {"IDLE", "ARMED", "CAPTURING", "SENDING"};
    return n[L.st];
}

uint32_t la_captures(void)
{
    return L.captures;
}

uint32_t la_rate(void)
{
    return L.rate;
}

void la_debug(uint32_t *isr, uint32_t *wr, uint16_t *first4)
{
    *isr = s_last_isr;
    *wr = s_last_written;
    memcpy(first4, buf, 8);
}

void la_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_D2SRAM1EN | RCC_AHB2ENR_D2SRAM2EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    (void)RCC->AHB1ENR;
    la_set_port('B');
    L.divider = 99; /* 1 MHz */
    L.readcount = L.delaycount = 4096;
    NVIC_SetPriority(DMA2_Stream0_IRQn, 2);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

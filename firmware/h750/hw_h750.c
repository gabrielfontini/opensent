/*
 * OpenSENT hardware layer for the WeAct STM32H750VBT6 core board.
 *
 *   TX   PB10  TIM2_CH3 (AF1)  PWM, period rewritten by DMA1 Stream0 on
 *                               every update event -> jitter-free pulses
 *   RX0  PA0   TIM5_CH1 (AF2)  input capture -> DMA1 Stream1 ring
 *   RX1  PA1   TIM5_CH2 (AF2)  input capture -> DMA1 Stream2 ring
 *   RX2  PB10  EXTI10 on the TX pin, timestamped with TIM5 -> monitors the
 *              real pin level (proves the waveform without a jumper)
 *   DAC  PA4 / PA5  DAC1 OUT1 / OUT2 (0..3.3 V, bridge output)
 *   LED  PE3
 *   USB  PA11/PA12 OTG_FS (CDC)
 *
 * TIM2 and TIM5 are 32-bit timers clocked at 200 MHz (5 ns resolution).
 *
 * SPDX-License-Identifier: MIT
 */
#include <stdio.h>
#include <string.h>

#include "stm32h7xx.h"
#include "tusb.h"

#include "clock.h"
#include "hw.h"
#include "qspi_flash.h"
#include "la.h"

#define TX_RING   32u    /* pulses in the TX DMA ring (two halves) */
#define RX_RING   4096u  /* timestamps per capture ring (power of two) */
#define MON_RING  1024u

#define DMAMUX_REQ_TIM2_UP  22u
#define DMAMUX_REQ_TIM5_CH1 55u
#define DMAMUX_REQ_TIM5_CH2 56u

#define BOOT_MAGIC 0xB007DF00u
#define SYSMEM_BOOT 0x1FF09800u

static volatile uint32_t s_ms;
static char s_uid[25];

static uint32_t tx_ring[TX_RING];
static sent_tx_t *volatile s_tx;
static volatile bool s_tx_running;

static uint32_t rx_ring[2][RX_RING];
static volatile uint32_t rx_laps[2];
static uint32_t rx_consumed[2];

static uint32_t mon_ring[MON_RING];
static volatile uint32_t mon_head;
static uint32_t mon_tail;
static volatile uint32_t mon_ovr;
static bool mon_en;

__attribute__((section(".noinit"))) static volatile uint32_t s_boot_magic;
static volatile uint32_t s_boot_at;

/* ------------------------------------------------------------ gpio */

static void gpio_mode(GPIO_TypeDef *g, unsigned pin, unsigned mode, unsigned af)
{
    g->MODER = (g->MODER & ~(3u << (2 * pin))) | (mode << (2 * pin));
    if (pin < 8) {
        g->AFR[0] = (g->AFR[0] & ~(0xFu << (4 * pin))) | (af << (4 * pin));
    } else {
        g->AFR[1] = (g->AFR[1] & ~(0xFu << (4 * (pin - 8)))) | (af << (4 * (pin - 8)));
    }
}

static void gpio_speed(GPIO_TypeDef *g, unsigned pin, unsigned speed)
{
    g->OSPEEDR = (g->OSPEEDR & ~(3u << (2 * pin))) | (speed << (2 * pin));
}

static void gpio_pull(GPIO_TypeDef *g, unsigned pin, unsigned pull)
{
    g->PUPDR = (g->PUPDR & ~(3u << (2 * pin))) | (pull << (2 * pin));
}

/* ------------------------------------------------------------ basics */

void SysTick_Handler(void)
{
    s_ms++;
}

uint32_t hw_millis(void)
{
    return s_ms;
}

void hw_led(bool on)
{
    GPIOE->BSRR = on ? (1u << 3) : (1u << (3 + 16));
}

const char *hw_board(void)
{
    return "WeAct-H750";
}

const char *hw_uid(void)
{
    return s_uid;
}

const char *hw_pins(void)
{
    return "TX=PB10 RX0=PA0 RX1=PA1 RX2=PB10(monitor) DAC1=PA4 DAC2=PA5 LED=PE3";
}

float hw_dac_max(void)
{
    return 3.3f;
}

void hw_dac(unsigned ch, float volts)
{
    float x = volts / 3.3f * 4095.0f + 0.5f;
    uint32_t v = x < 0 ? 0u : (x > 4095.0f ? 4095u : (uint32_t)x);
    if (ch == 0) {
        DAC1->DHR12R1 = v;
    } else if (ch == 1) {
        DAC1->DHR12R2 = v;
    }
}

/* DAC read-back: DAC1 OUT1/OUT2 are internally connected to ADC2 inputs
 * 16/17. DAC and ADC share VREF+, so the read-back is ratiometric: it
 * verifies the bridge mapping independently of the exact VREF+ value. */
static bool s_adc_ok;

static void delay_us(uint32_t us)
{
    uint32_t n = us * (SystemCoreClock / 4000000u);
    while (n--) {
        __NOP();
    }
}

static void adc_init(void)
{
    RCC->D3CCIPR = (RCC->D3CCIPR & ~RCC_D3CCIPR_ADCSEL) | (2u << RCC_D3CCIPR_ADCSEL_Pos); /* per_ck = HSI 64 MHz */
    RCC->AHB1ENR |= RCC_AHB1ENR_ADC12EN;
    (void)RCC->AHB1ENR;
    ADC12_COMMON->CCR = (2u << ADC_CCR_PRESC_Pos); /* async clock / 4 = 16 MHz */
    ADC2->CR = 0;                                   /* leave deep power down */
    ADC2->CR = ADC_CR_ADVREGEN | (2u << ADC_CR_BOOST_Pos);
    delay_us(50);
    ADC2->CR |= ADC_CR_ADCALLIN;
    ADC2->CR |= ADC_CR_ADCAL;
    for (uint32_t i = 0; (ADC2->CR & ADC_CR_ADCAL) && i < 10000000u; i++) {
    }
    ADC2->ISR = ADC_ISR_ADRDY;
    ADC2->CR |= ADC_CR_ADEN;
    for (uint32_t i = 0; !(ADC2->ISR & ADC_ISR_ADRDY) && i < 10000000u; i++) {
    }
    s_adc_ok = (ADC2->ISR & ADC_ISR_ADRDY) != 0;
    ADC2->CFGR = ADC_CFGR_OVRMOD; /* 16 bit, single, software trigger */
    ADC2->PCSEL = (1u << 16) | (1u << 17);
    ADC2->SMPR2 = (7u << ADC_SMPR2_SMP16_Pos) | (7u << (ADC_SMPR2_SMP16_Pos + 3)); /* 810.5 cycles */
}

float hw_dac_meas(unsigned ch)
{
    if (!s_adc_ok || ch > 1) {
        return -1.0f;
    }
    uint32_t acc = 0;
    const unsigned n = 16;
    ADC2->SQR1 = ((16u + ch) << ADC_SQR1_SQ1_Pos); /* L = 0: one conversion */
    for (unsigned i = 0; i < n; i++) {
        ADC2->ISR = ADC_ISR_EOC;
        ADC2->CR |= ADC_CR_ADSTART;
        for (uint32_t k = 0; !(ADC2->ISR & ADC_ISR_EOC) && k < 1000000u; k++) {
        }
        acc += ADC2->DR;
    }
    return (float)acc / (float)n / 65535.0f * 3.3f;
}

/* ------------------------------------------------------------ console */

void hw_write(const char *s, unsigned n)
{
    if (!tud_cdc_connected()) {
        return;
    }
    uint32_t t0 = s_ms;
    while (n) {
        uint32_t w = tud_cdc_write(s, n);
        s += w;
        n -= w;
        if (n) {
            IWDG1->KR = 0xAAAAu;
            tud_cdc_write_flush();
            tud_task();
            if (s_ms - t0 > 50u) {
                return; /* host is not reading: drop instead of stalling the decoders */
            }
        }
    }
    tud_cdc_write_flush();
}

/* ------------------------------------------------------------ TX */

uint32_t hw_tx_clk(void)
{
    return clock_timer_hz;
}

static void tx_fill(unsigned from, unsigned count)
{
    sent_tx_t *tx = s_tx;
    uint32_t low = 0;
    for (unsigned i = 0; i < count; i++) {
        tx_ring[from + i] = sent_tx_next(tx, &low) - 1u;
    }
    TIM2->CCR3 = low; /* preloaded: applies from the next update */
}

void DMA1_Stream0_IRQHandler(void)
{
    uint32_t isr = DMA1->LISR;
    if (isr & DMA_LISR_HTIF0) {
        DMA1->LIFCR = DMA_LIFCR_CHTIF0;
        tx_fill(0, TX_RING / 2);
    }
    if (isr & DMA_LISR_TCIF0) {
        DMA1->LIFCR = DMA_LIFCR_CTCIF0;
        tx_fill(TX_RING / 2, TX_RING / 2);
    }
    if (isr & (DMA_LISR_TEIF0 | DMA_LISR_DMEIF0 | DMA_LISR_FEIF0)) {
        DMA1->LIFCR = DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;
    }
}

static void tx_idle_high(void)
{
    /* forced inactive OC3REF -> output high through the inverted polarity */
    TIM2->CCMR2 = (TIM2->CCMR2 & ~(TIM_CCMR2_OC3M | TIM_CCMR2_OC3M_3)) | (4u << TIM_CCMR2_OC3M_Pos);
}

void hw_tx_stop(void)
{
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->DIER = 0;
    DMA1_Stream0->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream0->CR & DMA_SxCR_EN) {
    }
    DMA1->LIFCR = 0x3Du;
    tx_idle_high();
    s_tx_running = false;
}

void hw_tx_start(sent_tx_t *tx)
{
    hw_tx_stop();
    s_tx = tx;

    uint32_t low = 0;
    uint32_t p0 = sent_tx_next(tx, &low);
    uint32_t p1 = sent_tx_next(tx, NULL);
    tx_fill(0, TX_RING);

    TIM2->CR1  = TIM_CR1_ARPE;
    TIM2->PSC  = 0;
    TIM2->CNT  = 0;
    TIM2->ARR  = p0 - 1u;
    TIM2->CCR3 = low;
    TIM2->CCMR2 = (TIM2->CCMR2 & ~(TIM_CCMR2_OC3M | TIM_CCMR2_OC3M_3 | TIM_CCMR2_CC3S)) |
                  (6u << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE; /* PWM1 */
    TIM2->EGR  = TIM_EGR_UG; /* load ARR = p0 and CCR3 into the shadow registers */
    TIM2->SR   = 0;
    TIM2->ARR  = p1 - 1u;    /* preload: period of the second pulse */

    DMA1_Stream0->PAR  = (uint32_t)&TIM2->ARR;
    DMA1_Stream0->M0AR = (uint32_t)tx_ring;
    DMA1_Stream0->NDTR = TX_RING;
    DMA1_Stream0->FCR  = 0;
    DMA1_Stream0->CR   = DMA_SxCR_CIRC | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_MSIZE_1 |
                         DMA_SxCR_PSIZE_1 | DMA_SxCR_HTIE | DMA_SxCR_TCIE | DMA_SxCR_PL_1 | DMA_SxCR_PL_0;
    DMAMUX1_Channel0->CCR = DMAMUX_REQ_TIM2_UP;
    DMA1_Stream0->CR |= DMA_SxCR_EN;

    TIM2->DIER = TIM_DIER_UDE;
    s_tx_running = true;
    TIM2->CR1 |= TIM_CR1_CEN; /* CNT = 0 < CCR3: the line falls now, the first sync starts */
}

void hw_tx_lock(void)
{
    NVIC_DisableIRQ(DMA1_Stream0_IRQn);
    __DSB();
    __ISB();
}

void hw_tx_unlock(void)
{
    NVIC_EnableIRQ(DMA1_Stream0_IRQn);
}

void hw_tx_drive(bool open_drain)
{
    if (open_drain) {
        GPIOB->OTYPER |= 1u << 10;
    } else {
        GPIOB->OTYPER &= ~(1u << 10);
    }
}

/* ------------------------------------------------------------ RX */

uint32_t hw_rx_clk(void)
{
    return clock_timer_hz;
}

void DMA1_Stream1_IRQHandler(void)
{
    if (DMA1->LISR & DMA_LISR_TCIF1) {
        DMA1->LIFCR = DMA_LIFCR_CTCIF1;
        rx_laps[0]++;
    }
}

void DMA1_Stream2_IRQHandler(void)
{
    if (DMA1->LISR & DMA_LISR_TCIF2) {
        DMA1->LIFCR = DMA_LIFCR_CTCIF2;
        rx_laps[1]++;
    }
}

void EXTI15_10_IRQHandler(void)
{
    uint32_t t = TIM5->CNT;
    EXTI_D1->PR1 = 1u << 10;
    uint32_t h = mon_head, nh = (h + 1u) % MON_RING;
    if (nh == mon_tail) {
        mon_ovr++;
        return;
    }
    mon_ring[h] = t;
    mon_head = nh;
}

static DMA_Stream_TypeDef *const rx_stream[2] = {DMA1_Stream1, DMA1_Stream2};

static void rx_dma_start(unsigned ch)
{
    DMA_Stream_TypeDef *s = rx_stream[ch];
    s->CR &= ~DMA_SxCR_EN;
    while (s->CR & DMA_SxCR_EN) {
    }
    DMA1->LIFCR = ch == 0 ? (0x3Du << 6) : (0x3Du << 16);
    s->PAR  = ch == 0 ? (uint32_t)&TIM5->CCR1 : (uint32_t)&TIM5->CCR2;
    s->M0AR = (uint32_t)rx_ring[ch];
    s->NDTR = RX_RING;
    s->FCR  = 0;
    s->CR   = DMA_SxCR_CIRC | DMA_SxCR_MINC | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_TCIE |
              DMA_SxCR_PL_1;
    (ch == 0 ? DMAMUX1_Channel1 : DMAMUX1_Channel2)->CCR = ch == 0 ? DMAMUX_REQ_TIM5_CH1 : DMAMUX_REQ_TIM5_CH2;
    rx_laps[ch] = 0;
    rx_consumed[ch] = 0;
    s->CR |= DMA_SxCR_EN;
}

void hw_rx_enable(unsigned ch, bool en, bool invert)
{
    if (ch < 2) {
        uint32_t e = ch == 0 ? TIM_CCER_CC1E : TIM_CCER_CC2E;
        uint32_t p = ch == 0 ? TIM_CCER_CC1P : TIM_CCER_CC2P;
        uint32_t de = ch == 0 ? TIM_DIER_CC1DE : TIM_DIER_CC2DE;
        TIM5->CCER &= ~(e | p);
        TIM5->DIER &= ~de;
        if (en) {
            rx_dma_start(ch);
            TIM5->CCER |= (invert ? 0u : p) | e; /* CCxP = 1: falling edge */
            TIM5->DIER |= de;
        } else {
            rx_stream[ch]->CR &= ~DMA_SxCR_EN;
        }
    } else if (ch == 2) {
        EXTI_D1->IMR1 &= ~(1u << 10);
        EXTI->FTSR1 &= ~(1u << 10);
        EXTI->RTSR1 &= ~(1u << 10);
        mon_head = mon_tail = 0;
        mon_en = en;
        if (en) {
            if (invert) {
                EXTI->RTSR1 |= 1u << 10;
            } else {
                EXTI->FTSR1 |= 1u << 10;
            }
            EXTI_D1->PR1 = 1u << 10;
            EXTI_D1->IMR1 |= 1u << 10;
        }
    }
}

static bool pin_level(unsigned ch);

bool hw_rx_both_edges(unsigned ch, bool both, bool invert)
{
    bool l1 = false, l2 = true;
    if (ch < 2) {
        uint32_t e = ch == 0 ? TIM_CCER_CC1E : TIM_CCER_CC2E;
        uint32_t p = ch == 0 ? TIM_CCER_CC1P : TIM_CCER_CC2P;
        uint32_t np = ch == 0 ? TIM_CCER_CC1NP : TIM_CCER_CC2NP;
        if (!both) {
            TIM5->CCER = (TIM5->CCER & ~(p | np)) | (invert ? 0u : p);
            return pin_level(ch);
        }
        /* arm atomically: capture off, ring flushed, level read, capture on;
         * retry if an edge slipped in between */
        uint32_t f = ch == 0 ? (TIM_SR_CC1IF | TIM_SR_CC1OF) : (TIM_SR_CC2IF | TIM_SR_CC2OF);
        do {
            TIM5->CCER &= ~(e | p | np);
            TIM5->SR = ~f; /* drop a capture (and its DMA request) from before */
            rx_dma_start(ch);
            l1 = pin_level(ch);
            TIM5->CCER |= p | np | e;
            l2 = pin_level(ch);
        } while (l1 != l2);
        return l1;
    }
    EXTI_D1->IMR1 &= ~(1u << 10);
    EXTI->FTSR1 &= ~(1u << 10);
    EXTI->RTSR1 &= ~(1u << 10);
    if (!both) {
        if (invert) {
            EXTI->RTSR1 |= 1u << 10;
        } else {
            EXTI->FTSR1 |= 1u << 10;
        }
        EXTI_D1->PR1 = 1u << 10;
        EXTI_D1->IMR1 |= 1u << 10;
        return pin_level(ch);
    }
    do {
        EXTI_D1->IMR1 &= ~(1u << 10);
        EXTI_D1->PR1 = 1u << 10;
        NVIC_ClearPendingIRQ(EXTI15_10_IRQn); /* an edge from before must not be the first */
        __DSB();
        mon_head = mon_tail = 0;
        l1 = pin_level(ch);
        EXTI->FTSR1 |= 1u << 10;
        EXTI->RTSR1 |= 1u << 10;
        EXTI_D1->PR1 = 1u << 10;
        EXTI_D1->IMR1 |= 1u << 10;
        l2 = pin_level(ch);
    } while (l1 != l2);
    return l1;
}

static bool pin_level(unsigned ch)
{
    if (ch == 0) {
        return (GPIOA->IDR >> 0) & 1u;
    }
    if (ch == 1) {
        return (GPIOA->IDR >> 1) & 1u;
    }
    return (GPIOB->IDR >> 10) & 1u;
}

bool hw_rx_pin(unsigned ch)
{
    return pin_level(ch);
}

unsigned hw_rx_read(unsigned ch, uint32_t *ts, unsigned max, uint32_t *overruns)
{
    *overruns = 0;
    if (ch == 2) {
        unsigned n = 0;
        while (n < max && mon_tail != mon_head) {
            ts[n++] = mon_ring[mon_tail];
            mon_tail = (mon_tail + 1u) % MON_RING;
        }
        if (mon_ovr) {
            __disable_irq();
            *overruns = mon_ovr;
            mon_ovr = 0;
            __enable_irq();
        }
        return n;
    }
    if (ch > 1 || !(rx_stream[ch]->CR & DMA_SxCR_EN)) {
        return 0;
    }
    /* total number of words written = laps * RX_RING + position, read atomically */
    const uint32_t tcif = ch == 0 ? DMA_LISR_TCIF1 : DMA_LISR_TCIF2;
    uint32_t laps, ndtr, f1, f2;
    __disable_irq();
    do {
        f1   = DMA1->LISR & tcif;
        ndtr = rx_stream[ch]->NDTR;
        f2   = DMA1->LISR & tcif;
    } while (f1 != f2);
    laps = rx_laps[ch] + (f1 ? 1u : 0u);
    __enable_irq();
    uint32_t written = laps * RX_RING + (RX_RING - ndtr);
    uint32_t avail = written - rx_consumed[ch];
    if (avail > RX_RING - 64u) {
        *overruns = avail;
        rx_consumed[ch] = written;
        return 0;
    }
    unsigned n = avail < max ? avail : max;
    for (unsigned i = 0; i < n; i++) {
        ts[i] = rx_ring[ch][(rx_consumed[ch] + i) & (RX_RING - 1u)];
    }
    rx_consumed[ch] += n;
    return n;
}

/* Wire-free test of the real RX input path: with nothing connected to the
 * RX pin, toggling its internal pull-up / pull-down (~40 kOhm) moves the
 * pin and the timer captures the edges exactly like an external sensor.
 * The pulses are timed with the cycle counter, interrupts off per frame;
 * the gap between frames is seen as a (long) pause pulse. */
bool hw_rx_pulltest(unsigned ch, sent_tx_t *gen, unsigned frames)
{
    if (ch > 1) {
        return false;
    }
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    const uint32_t shift = 2u * ch;
    const uint32_t up = 1u << shift, down = 2u << shift, mask = 3u << shift;
    const double cyc_per_tick = (double)gen->cfg.tick_ns * 1e-9 * (double)SystemCoreClock;
    const uint32_t low = (uint32_t)(5.0 * cyc_per_tick);
    uint16_t ticks[SENT_MAX_DATA + 5];
    for (unsigned f = 0; f < frames; f++) {
        unsigned k = sent_tx_build_frame(gen, ticks);
        if (k > 0 && ticks[k - 1] >= SENT_PAUSE_MIN && k > 3 && gen->cfg.pause_mode != SENT_PAUSE_OFF) {
            k--; /* the inter-frame gap is the pause */
        }
        __disable_irq();
        uint32_t t = DWT->CYCCNT + 200u;
        while ((int32_t)(DWT->CYCCNT - t) < 0) {
        }
        double acc = 0.0;
        for (unsigned i = 0; i <= k; i++) {
            uint32_t start = t + (uint32_t)acc;
            while ((int32_t)(DWT->CYCCNT - start) < 0) {
            }
            GPIOA->PUPDR = (GPIOA->PUPDR & ~mask) | down; /* falling edge */
            if (i == k) {
                break; /* edge that ends the CRC nibble */
            }
            while ((int32_t)(DWT->CYCCNT - (start + low)) < 0) {
            }
            GPIOA->PUPDR = (GPIOA->PUPDR & ~mask) | up;
            acc += (double)ticks[i] * cyc_per_tick;
        }
        uint32_t end = DWT->CYCCNT + low;
        while ((int32_t)(DWT->CYCCNT - end) < 0) {
        }
        GPIOA->PUPDR = (GPIOA->PUPDR & ~mask) | up;
        __enable_irq();
        IWDG1->KR = 0xAAAAu;
        for (volatile uint32_t d = 0; d < 20000u; d++) { /* ~ 100 us gap: pause */
        }
    }
    GPIOA->PUPDR = (GPIOA->PUPDR & ~mask) | up;
    return true;
}

/* ------------------------------------------------------------ FDCAN1
 * Classic CAN 2.0 on PD0 (RX) / PD1 (TX), AF9, kernel clock HSE 25 MHz,
 * 25 time quanta per bit, sample point 80 %. Message RAM: RX FIFO0 of
 * 8 elements at word 0, TX FIFO of 4 elements at word 32 (8-byte data). */
#define CAN_RAM ((volatile uint32_t *)SRAMCAN_BASE)
#define CAN_RXF0_WORD 0u
#define CAN_TXQ_WORD  32u
static bool s_can_on;

void hw_can_stop(void)
{
    if (!(RCC->APB1HENR & RCC_APB1HENR_FDCANEN)) {
        return;
    }
    FDCAN1->CCCR |= FDCAN_CCCR_INIT;
    s_can_on = false;
}

bool hw_can_start(uint32_t bitrate, bool loopback)
{
    uint32_t presc = 25000000u / (25u * bitrate);
    if (presc < 1u || presc * 25u * bitrate != 25000000u) {
        return false; /* 1M, 500k, 250k, 125k, 100k, 50k ... */
    }
    RCC->D2CCIP1R = (RCC->D2CCIP1R & ~RCC_D2CCIP1R_FDCANSEL) | (0u << RCC_D2CCIP1R_FDCANSEL_Pos); /* HSE */
    RCC->APB1HENR |= RCC_APB1HENR_FDCANEN;
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN;
    (void)RCC->APB1HENR;
    gpio_mode(GPIOD, 0, 2, 9);
    gpio_mode(GPIOD, 1, 2, 9);

    FDCAN1->CCCR |= FDCAN_CCCR_INIT;
    for (uint32_t i = 0; !(FDCAN1->CCCR & FDCAN_CCCR_INIT) && i < 1000000u; i++) {
    }
    FDCAN1->CCCR |= FDCAN_CCCR_CCE;
    FDCAN1->CCCR &= ~(FDCAN_CCCR_TEST | FDCAN_CCCR_MON | FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE | FDCAN_CCCR_DAR);
    FDCAN1->NBTP = ((4u - 1u) << FDCAN_NBTP_NSJW_Pos) | ((presc - 1u) << FDCAN_NBTP_NBRP_Pos) |
                   ((19u - 1u) << FDCAN_NBTP_NTSEG1_Pos) | ((5u - 1u) << FDCAN_NBTP_NTSEG2_Pos);
    FDCAN1->GFC = 0;   /* accept everything into FIFO0 */
    FDCAN1->SIDFC = 0;
    FDCAN1->XIDFC = 0;
    FDCAN1->RXESC = 0; /* 8-byte elements */
    FDCAN1->TXESC = 0;
    FDCAN1->RXF0C = (CAN_RXF0_WORD << FDCAN_RXF0C_F0SA_Pos) | (8u << FDCAN_RXF0C_F0S_Pos);
    FDCAN1->TXBC = (CAN_TXQ_WORD << FDCAN_TXBC_TBSA_Pos) | (4u << FDCAN_TXBC_TFQS_Pos);
    for (unsigned i = 0; i < CAN_TXQ_WORD + 16u; i++) {
        CAN_RAM[i] = 0;
    }
    if (loopback) {
        /* internal loopback: frames are received by our own FIFO0, nothing
         * reaches the pins' bus side (no transceiver needed) */
        FDCAN1->CCCR |= FDCAN_CCCR_TEST | FDCAN_CCCR_MON;
        FDCAN1->TEST |= FDCAN_TEST_LBCK;
    }
    FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;
    for (uint32_t i = 0; (FDCAN1->CCCR & FDCAN_CCCR_INIT) && i < 1000000u; i++) {
    }
    s_can_on = !(FDCAN1->CCCR & FDCAN_CCCR_INIT);
    return s_can_on;
}

bool hw_can_send(uint32_t id, const uint8_t *d, uint8_t len)
{
    if (!s_can_on || (FDCAN1->TXFQS & FDCAN_TXFQS_TFQF)) {
        return false;
    }
    uint32_t idx = (FDCAN1->TXFQS & FDCAN_TXFQS_TFQPI_Msk) >> FDCAN_TXFQS_TFQPI_Pos;
    volatile uint32_t *e = &CAN_RAM[CAN_TXQ_WORD + idx * 4u];
    uint32_t w[2] = {0, 0};
    memcpy(w, d, len > 8 ? 8 : len);
    e[0] = (id & 0x7FFu) << 18;
    e[1] = (uint32_t)(len > 8 ? 8 : len) << 16;
    e[2] = w[0];
    e[3] = w[1];
    FDCAN1->TXBAR = 1u << idx;
    return true;
}

bool hw_can_recv(uint32_t *id, uint8_t *d, uint8_t *len)
{
    if (!s_can_on || !(FDCAN1->RXF0S & FDCAN_RXF0S_F0FL)) {
        return false;
    }
    uint32_t idx = (FDCAN1->RXF0S & FDCAN_RXF0S_F0GI_Msk) >> FDCAN_RXF0S_F0GI_Pos;
    volatile uint32_t *e = &CAN_RAM[CAN_RXF0_WORD + idx * 4u];
    uint32_t r0 = e[0], r1 = e[1], w[2] = {e[2], e[3]};
    *id  = (r0 & (1u << 30)) ? (r0 & 0x1FFFFFFFu) : ((r0 >> 18) & 0x7FFu);
    *len = (uint8_t)((r1 >> 16) & 0xFu);
    if (*len > 8) {
        *len = 8;
    }
    memcpy(d, w, *len);
    FDCAN1->RXF0A = idx;
    return true;
}

void hw_can_status(uint32_t *tec, uint32_t *rec, uint32_t *psr)
{
    uint32_t ecr = s_can_on ? FDCAN1->ECR : 0u;
    *tec = (ecr & FDCAN_ECR_TEC_Msk) >> FDCAN_ECR_TEC_Pos;
    *rec = (ecr & FDCAN_ECR_REC_Msk) >> FDCAN_ECR_REC_Pos;
    *psr = s_can_on ? FDCAN1->PSR : 0u;
}

/* ------------------------------------------------------------ config
 * 4 KB sector 0 of the QSPI flash: "OSC1", length, CRC32, script text */
static uint32_t s_jedec;
static char s_cfg_info[48];
#define CFG_MAGIC 0x3143534Fu /* "OSC1" */

static uint32_t crc32(const uint8_t *p, unsigned n)
{
    uint32_t c = 0xFFFFFFFFu;
    while (n--) {
        c ^= *p++;
        for (int k = 0; k < 8; k++) {
            c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
        }
    }
    return ~c;
}

const char *hw_cfg_info(void)
{
    if (s_jedec) {
        snprintf(s_cfg_info, sizeof s_cfg_info, "QSPI:%06lX", (unsigned long)s_jedec);
    } else {
        snprintf(s_cfg_info, sizeof s_cfg_info, "none");
    }
    return s_cfg_info;
}

int hw_cfg_load(char *buf, unsigned max)
{
    uint32_t h[3];
    if (!s_jedec || !qspi_flash_read(0, h, sizeof h) || h[0] != CFG_MAGIC || h[1] > max || h[1] > 4096u - 12u) {
        return -1;
    }
    if (!qspi_flash_read(12, buf, h[1]) || crc32((const uint8_t *)buf, h[1]) != h[2]) {
        return -1;
    }
    return (int)h[1];
}

bool hw_cfg_store(const char *buf, unsigned len)
{
    if (!s_jedec || len > 4096u - 12u) {
        return false;
    }
    uint32_t h[3] = {CFG_MAGIC, len, crc32((const uint8_t *)buf, len)};
    IWDG1->KR = 0xAAAAu;
    if (!qspi_flash_erase4k(0)) {
        return false;
    }
    if (len == 0) {
        return true; /* erased = no script */
    }
    IWDG1->KR = 0xAAAAu;
    if (!qspi_flash_write(12, buf, len) || !qspi_flash_write(0, h, sizeof h)) {
        return false;
    }
    uint32_t v[3];
    return qspi_flash_read(0, v, sizeof v) && memcmp(v, h, sizeof h) == 0;
}

bool hw_boot_skip(void)
{
    /* K1 (PC13, pressed = high) held while powering up or resetting */
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN;
    (void)RCC->AHB4ENR;
    GPIOC->MODER &= ~(3u << 26);
    GPIOC->PUPDR = (GPIOC->PUPDR & ~(3u << 26)) | (2u << 26);
    for (volatile int i = 0; i < 1000; i++) {
    }
    return (GPIOC->IDR >> 13) & 1u;
}

/* ------------------------------------------------------------ logic analyzer glue */
bool hw_la_port(char name)
{
    return la_set_port(name);
}

const char *hw_la_info(void)
{
    static char b[160];
    uint32_t isr, wr;
    uint16_t f[4];
    la_debug(&isr, &wr, f);
    snprintf(b, sizeof b, "port=%c state=%s captures=%lu rate=%lu max=%lu dma=%lX n=%lu s0=%04X",
             la_port(), la_state(), (unsigned long)la_captures(), (unsigned long)la_rate(),
             (unsigned long)LA_MAX_RATE, (unsigned long)isr, (unsigned long)wr, f[0]);
    return b;
}

/* test signal: 50 % square wave on PB0 (TIM3_CH3) and half that on PB1 (TIM3_CH4, toggle) */
bool hw_la_testsig(uint32_t hz)
{
    RCC->APB1LENR |= RCC_APB1LENR_TIM3EN;
    (void)RCC->APB1LENR;
    TIM3->CR1 = 0;
    if (!hz) {
        gpio_mode(GPIOB, 0, 0, 0);
        gpio_mode(GPIOB, 1, 0, 0);
        return true;
    }
    uint32_t ticks = clock_timer_hz / hz; /* timer clocks per period */
    uint32_t psc = ticks / 65536u;
    uint32_t arr = ticks / (psc + 1u);
    if (hz > clock_timer_hz / 4u || arr < 4u) {
        return false;
    }
    TIM3->PSC = psc;
    TIM3->ARR = arr - 1u;
    TIM3->CCR3 = arr / 2u;
    TIM3->CCR4 = 0;
    TIM3->CCMR2 = (6u << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE | (3u << TIM_CCMR2_OC4M_Pos); /* PWM1, toggle */
    TIM3->CCER = TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM3->EGR = TIM_EGR_UG;
    gpio_mode(GPIOB, 0, 2, 2);
    gpio_mode(GPIOB, 1, 2, 2);
    TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
    return true;
}

/* ------------------------------------------------------------ boot */

void hw_bootloader(void)
{
    s_boot_at = s_ms + 100u; /* let the reply go out first (see hw_poll) */
}

/* Independent watchdog, ~0.5 s: if the firmware ever hangs, the chip
 * resets and every output (TX, bridge DACs, CAN) comes back off / 0 V
 * instead of freezing on the last value. Paused while a debugger halts. */
static void iwdg_start(void)
{
    DBGMCU->APB4FZ1 |= DBGMCU_APB4FZ1_DBG_IWDG1;
    IWDG1->KR  = 0xCCCCu;
    IWDG1->KR  = 0x5555u;
    IWDG1->PR  = 3u;    /* LSI 32 kHz / 32 = 1 kHz */
    IWDG1->RLR = 500u;  /* 500 ms */
    while (IWDG1->SR) {
    }
    IWDG1->KR = 0xAAAAu;
}

bool hw_watchdog_test(void)
{
    __disable_irq();
    for (;;) { /* simulated firmware hang: the IWDG must reset the chip */
    }
    return true;
}

void hw_poll(void)
{
    IWDG1->KR = 0xAAAAu;
    if (s_boot_at && (int32_t)(s_ms - s_boot_at) >= 0) {
        s_boot_magic = BOOT_MAGIC;
        NVIC_SystemReset();
    }
}

/* called first thing in main(), before any clock or peripheral setup */
void hw_early(void)
{
    /* Started by another program (e.g. the ROM bootloader after a DFU
     * "leave") instead of a reset: clocks, NVIC and VTOR are not in reset
     * state. A clean reset is the robust way out; it boots this firmware. */
    if ((RCC->CFGR & RCC_CFGR_SWS) != 0u /* SYSCLK not HSI */ && s_boot_magic != BOOT_MAGIC) {
        NVIC_SystemReset();
    }
    SCB->VTOR = 0x08000000u;
    if (s_boot_magic == BOOT_MAGIC) {
        s_boot_magic = 0;
        uint32_t sp = *(volatile uint32_t *)SYSMEM_BOOT;
        uint32_t pc = *(volatile uint32_t *)(SYSMEM_BOOT + 4u);
        __set_MSP(sp);
        ((void (*)(void))pc)();
    }
}

/* ------------------------------------------------------------ init */

void hw_init(void)
{
    SCB_EnableICache();
    SysTick_Config(SystemCoreClock / 1000u);

    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOEEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    RCC->APB1LENR |= RCC_APB1LENR_TIM2EN | RCC_APB1LENR_TIM5EN | RCC_APB1LENR_DAC12EN;
    RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;
    (void)RCC->APB4ENR;

    const uint32_t *uid = (const uint32_t *)UID_BASE;
    snprintf(s_uid, sizeof s_uid, "%08lX%08lX%08lX", (unsigned long)uid[2], (unsigned long)uid[1],
             (unsigned long)uid[0]);

    /* LED */
    gpio_mode(GPIOE, 3, 1, 0);

    /* TX: PB10 TIM2_CH3, idle high */
    TIM2->CCER = 0;
    TIM2->CCMR2 = 0;
    tx_idle_high();
    TIM2->CCER = TIM_CCER_CC3E | TIM_CCER_CC3P;
    gpio_speed(GPIOB, 10, 1);
    gpio_mode(GPIOB, 10, 2, 1);

    /* RX: TIM5 free running, CH1/CH2 input capture with a 40 ns filter */
    gpio_pull(GPIOA, 0, 1);
    gpio_pull(GPIOA, 1, 1);
    gpio_mode(GPIOA, 0, 2, 2);
    gpio_mode(GPIOA, 1, 2, 2);
    TIM5->PSC = 0;
    TIM5->ARR = 0xFFFFFFFFu;
    TIM5->CCMR1 = (1u << TIM_CCMR1_CC1S_Pos) | (3u << TIM_CCMR1_IC1F_Pos) | (1u << TIM_CCMR1_CC2S_Pos) |
                  (3u << TIM_CCMR1_IC2F_Pos);
    TIM5->EGR = TIM_EGR_UG;
    TIM5->CR1 = TIM_CR1_CEN;

    /* monitor: EXTI10 <- PB10 */
    SYSCFG->EXTICR[2] = (SYSCFG->EXTICR[2] & ~(0xFu << 8)) | (1u << 8);

    /* DAC1 OUT1/OUT2 on PA4/PA5, buffered */
    gpio_mode(GPIOA, 4, 3, 0);
    gpio_mode(GPIOA, 5, 3, 0);
    DAC1->MCR = (1u << DAC_MCR_MODE1_Pos) | (1u << DAC_MCR_MODE2_Pos); /* pin + on-chip, buffered */
    DAC1->DHR12R1 = 0;
    DAC1->DHR12R2 = 0;
    DAC1->CR = DAC_CR_EN1 | DAC_CR_EN2;
    adc_init();
    s_jedec = qspi_flash_init();
    iwdg_start();

    NVIC_SetPriority(EXTI15_10_IRQn, 0);
    NVIC_SetPriority(DMA1_Stream0_IRQn, 1);
    NVIC_SetPriority(DMA1_Stream1_IRQn, 2);
    NVIC_SetPriority(DMA1_Stream2_IRQn, 2);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    NVIC_EnableIRQ(DMA1_Stream2_IRQn);

    /* USB OTG_FS on PA11/PA12, no VBUS sensing */
    gpio_speed(GPIOA, 11, 3);
    gpio_speed(GPIOA, 12, 3);
    gpio_mode(GPIOA, 11, 2, 10);
    gpio_mode(GPIOA, 12, 2, 10);
    RCC->AHB1ENR |= RCC_AHB1ENR_USB2OTGFSEN;
    (void)RCC->AHB1ENR;
    USB2_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBDEN;
    USB2_OTG_FS->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN | USB_OTG_GOTGCTL_BVALOVAL;
}

void OTG_FS_IRQHandler(void)
{
    tud_int_handler(0);
}

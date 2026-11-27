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

void hw_rx_both_edges(unsigned ch, bool both, bool invert)
{
    if (ch < 2) {
        uint32_t p = ch == 0 ? TIM_CCER_CC1P : TIM_CCER_CC2P;
        uint32_t np = ch == 0 ? TIM_CCER_CC1NP : TIM_CCER_CC2NP;
        TIM5->CCER &= ~(p | np);
        TIM5->CCER |= both ? (p | np) : (invert ? 0u : p);
    } else if (ch == 2) {
        EXTI->FTSR1 &= ~(1u << 10);
        EXTI->RTSR1 &= ~(1u << 10);
        if (both || !invert) {
            EXTI->FTSR1 |= 1u << 10;
        }
        if (both || invert) {
            EXTI->RTSR1 |= 1u << 10;
        }
    }
}

bool hw_rx_pin(unsigned ch)
{
    if (ch == 0) {
        return (GPIOA->IDR >> 0) & 1u;
    }
    if (ch == 1) {
        return (GPIOA->IDR >> 1) & 1u;
    }
    return (GPIOB->IDR >> 10) & 1u;
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

/* ------------------------------------------------------------ boot */

void hw_bootloader(void)
{
    s_boot_at = s_ms + 100u; /* let the reply go out first (see hw_poll) */
}

void hw_poll(void)
{
    if (s_boot_at && (int32_t)(s_ms - s_boot_at) >= 0) {
        s_boot_magic = BOOT_MAGIC;
        NVIC_SystemReset();
    }
}

/* called first thing in main(), before any clock or peripheral setup */
void hw_early(void)
{
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

/*
 * Minimal driver for the W25Q64 (8 MB) QSPI NOR flash of the WeAct H750
 * board, single-line indirect mode (simple and plenty fast for a config
 * sector). Pins: CLK PB2 (AF9), NCS PB6 (AF10), IO0 PD11, IO1 PD12,
 * IO2 PE2, IO3 PD13 (AF9; IO2/IO3 held high as /WP and /HOLD).
 * SPDX-License-Identifier: MIT
 */
#include "qspi_flash.h"

#include <string.h>

#include "stm32h7xx.h"

#define CCR_IMODE1  (1u << QUADSPI_CCR_IMODE_Pos)
#define CCR_ADMODE1 (1u << QUADSPI_CCR_ADMODE_Pos)
#define CCR_AD24    (2u << QUADSPI_CCR_ADSIZE_Pos)
#define CCR_DMODE1  (1u << QUADSPI_CCR_DMODE_Pos)
#define CCR_READ    (1u << QUADSPI_CCR_FMODE_Pos)

static bool s_ok;

static void pin(GPIO_TypeDef *g, unsigned p, unsigned af)
{
    g->MODER = (g->MODER & ~(3u << (2 * p))) | (2u << (2 * p));
    g->OSPEEDR |= 3u << (2 * p);
    g->PUPDR = (g->PUPDR & ~(3u << (2 * p))) | (1u << (2 * p));
    if (p < 8) {
        g->AFR[0] = (g->AFR[0] & ~(0xFu << (4 * p))) | (af << (4 * p));
    } else {
        g->AFR[1] = (g->AFR[1] & ~(0xFu << (4 * (p - 8)))) | (af << (4 * (p - 8)));
    }
}

static bool wait(uint32_t flag)
{
    for (uint32_t i = 0; i < 20000000u; i++) {
        if (QUADSPI->SR & flag) {
            return true;
        }
    }
    return false;
}

static bool idle(void)
{
    for (uint32_t i = 0; i < 20000000u; i++) {
        if (!(QUADSPI->SR & QUADSPI_SR_BUSY)) {
            return true;
        }
    }
    QUADSPI->CR |= QUADSPI_CR_ABORT;
    return false;
}

/* instruction [+ 24-bit address] [+ data in or out] */
static bool xfer(uint8_t ins, int32_t addr, uint8_t *rd, const uint8_t *wr, uint32_t n)
{
    if (!idle()) {
        return false;
    }
    QUADSPI->FCR = QUADSPI_FCR_CTCF;
    uint32_t ccr = CCR_IMODE1 | ins;
    if (addr >= 0) {
        ccr |= CCR_ADMODE1 | CCR_AD24;
    }
    if (n) {
        QUADSPI->DLR = n - 1u;
        ccr |= CCR_DMODE1 | (rd ? CCR_READ : 0u);
    }
    QUADSPI->CCR = ccr;
    if (addr >= 0) {
        QUADSPI->AR = (uint32_t)addr;
    }
    volatile uint8_t *dr = (volatile uint8_t *)&QUADSPI->DR;
    for (uint32_t i = 0; i < n; i++) {
        if (rd) {
            if (!wait(QUADSPI_SR_FTF | QUADSPI_SR_TCF)) {
                return false;
            }
            rd[i] = *dr;
        } else {
            if (!wait(QUADSPI_SR_FTF)) {
                return false;
            }
            *dr = wr[i];
        }
    }
    if (!wait(QUADSPI_SR_TCF)) {
        return false;
    }
    QUADSPI->FCR = QUADSPI_FCR_CTCF;
    return true;
}

static bool wait_ready(void)
{
    uint8_t st = 1;
    for (uint32_t i = 0; i < 2000000u; i++) {
        if (!xfer(0x05, -1, &st, NULL, 1)) {
            return false;
        }
        if (!(st & 1u)) {
            return true;
        }
    }
    return false;
}

uint32_t qspi_flash_init(void)
{
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIODEN | RCC_AHB4ENR_GPIOEEN;
    RCC->AHB3ENR |= RCC_AHB3ENR_QSPIEN;
    (void)RCC->AHB3ENR;
    pin(GPIOB, 2, 9);
    pin(GPIOB, 6, 10);
    pin(GPIOD, 11, 9);
    pin(GPIOD, 12, 9);
    /* IO2 (/WP) and IO3 (/HOLD) are not driven in single-line mode: keep them high */
    GPIOE->MODER = (GPIOE->MODER & ~(3u << 4)) | (1u << 4);
    GPIOE->BSRR = 1u << 2;
    GPIOD->MODER = (GPIOD->MODER & ~(3u << 26)) | (1u << 26);
    GPIOD->BSRR = 1u << 13;

    QUADSPI->CR = 0;
    QUADSPI->DCR = (22u << QUADSPI_DCR_FSIZE_Pos) | (3u << QUADSPI_DCR_CSHT_Pos); /* 8 MB */
    QUADSPI->CR = (7u << QUADSPI_CR_PRESCALER_Pos) | QUADSPI_CR_EN;               /* 200 / 8 = 25 MHz */
    uint8_t id[3] = {0, 0, 0};
    xfer(0xAB, -1, NULL, NULL, 0); /* release from power-down, harmless otherwise */
    if (!xfer(0x9F, -1, id, NULL, 3)) {
        return 0;
    }
    uint32_t jedec = ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | id[2];
    s_ok = jedec != 0 && jedec != 0xFFFFFFu;
    return s_ok ? jedec : 0;
}

bool qspi_flash_read(uint32_t addr, void *buf, uint32_t n)
{
    return s_ok && xfer(0x03, (int32_t)addr, buf, NULL, n);
}

bool qspi_flash_erase4k(uint32_t addr)
{
    return s_ok && xfer(0x06, -1, NULL, NULL, 0) && xfer(0x20, (int32_t)addr, NULL, NULL, 0) && wait_ready();
}

bool qspi_flash_write(uint32_t addr, const void *buf, uint32_t n)
{
    const uint8_t *p = buf;
    while (n && s_ok) {
        uint32_t chunk = 256u - (addr & 255u); /* stay inside one page */
        if (chunk > n) {
            chunk = n;
        }
        if (!xfer(0x06, -1, NULL, NULL, 0) || !xfer(0x02, (int32_t)addr, NULL, p, chunk) || !wait_ready()) {
            return false;
        }
        addr += chunk;
        p += chunk;
        n -= chunk;
    }
    return s_ok;
}

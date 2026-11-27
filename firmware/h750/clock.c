/*
 * STM32H750 clock tree for the WeAct core board (25 MHz HSE).
 *   SYSCLK 400 MHz (PLL1P), VOS1, AXI/AHB 200 MHz, APBx 100 MHz,
 *   APB1/APB2 timers 200 MHz, USB 48 MHz from HSI48 trimmed by CRS on SOF.
 * Falls back to HSI 64 MHz as PLL input if the crystal does not start.
 */
#include "clock.h"

#include "stm32h7xx.h"

uint32_t clock_timer_hz = 64000000u;

static bool hse_start(void)
{
    RCC->CR |= RCC_CR_HSEON;
    for (uint32_t i = 0; i < 2000000u; i++) {
        if (RCC->CR & RCC_CR_HSERDY) {
            return true;
        }
    }
    RCC->CR &= ~RCC_CR_HSEON;
    return false;
}

bool clock_init(void)
{
    /* supply from the internal LDO (no SMPS on H750); write-once register */
    PWR->CR3 = (PWR->CR3 & ~(PWR_CR3_SCUEN | PWR_CR3_LDOEN | PWR_CR3_BYPASS)) | PWR_CR3_LDOEN;
    while (!(PWR->CSR1 & PWR_CSR1_ACTVOSRDY)) {
    }
    /* VOS1: up to 400 MHz */
    PWR->D3CR = (PWR->D3CR & ~PWR_D3CR_VOS) | (3u << PWR_D3CR_VOS_Pos);
    while (!(PWR->D3CR & PWR_D3CR_VOSRDY)) {
    }

    const bool hse = hse_start();

    /* PLL1: ref 5 MHz (HSE/5 or HSI/... 64/16 = 4 MHz), VCO 800 MHz, P = 2 */
    RCC->CR &= ~RCC_CR_PLL1ON;
    while (RCC->CR & RCC_CR_PLL1RDY) {
    }
    uint32_t divm = hse ? 5u : 16u;
    uint32_t divn = hse ? 160u : 200u; /* 5*160 = 800, 4*200 = 800 */
    RCC->PLLCKSELR = (RCC->PLLCKSELR & ~(RCC_PLLCKSELR_PLLSRC | RCC_PLLCKSELR_DIVM1)) |
                     (hse ? RCC_PLLCKSELR_PLLSRC_HSE : RCC_PLLCKSELR_PLLSRC_HSI) |
                     (divm << RCC_PLLCKSELR_DIVM1_Pos);
    RCC->PLLCFGR = (RCC->PLLCFGR & ~(RCC_PLLCFGR_PLL1RGE | RCC_PLLCFGR_PLL1VCOSEL | RCC_PLLCFGR_PLL1FRACEN)) |
                   RCC_PLLCFGR_PLL1RGE_2 | /* 4..8 MHz input */
                   RCC_PLLCFGR_DIVP1EN | RCC_PLLCFGR_DIVQ1EN | RCC_PLLCFGR_DIVR1EN;
    RCC->PLL1DIVR = ((divn - 1u) << RCC_PLL1DIVR_N1_Pos) | ((2u - 1u) << RCC_PLL1DIVR_P1_Pos) |
                    ((8u - 1u) << RCC_PLL1DIVR_Q1_Pos) | ((2u - 1u) << RCC_PLL1DIVR_R1_Pos);
    RCC->CR |= RCC_CR_PLL1ON;
    while (!(RCC->CR & RCC_CR_PLL1RDY)) {
    }

    /* flash: 2 wait states, WRHIGHFREQ = 2 for 200 MHz AXI at VOS1 */
    FLASH->ACR = (FLASH->ACR & ~(FLASH_ACR_LATENCY | FLASH_ACR_WRHIGHFREQ)) | FLASH_ACR_LATENCY_2WS |
                 (2u << FLASH_ACR_WRHIGHFREQ_Pos);
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_2WS) {
    }

    /* bus prescalers before the switch */
    RCC->D1CFGR = RCC_D1CFGR_D1CPRE_DIV1 | RCC_D1CFGR_HPRE_DIV2 | RCC_D1CFGR_D1PPRE_DIV2;
    RCC->D2CFGR = RCC_D2CFGR_D2PPRE1_DIV2 | RCC_D2CFGR_D2PPRE2_DIV2;
    RCC->D3CFGR = RCC_D3CFGR_D3PPRE_DIV2;

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL1;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL1) {
    }
    SystemCoreClock = 400000000u;
    SystemD2Clock   = 200000000u;
    clock_timer_hz  = 200000000u; /* APB1 /2 -> timers x2 */

    /* USB: HSI48 kept on frequency by the CRS using the USB SOF */
    RCC->CR |= RCC_CR_HSI48ON;
    while (!(RCC->CR & RCC_CR_HSI48RDY)) {
    }
    RCC->D2CCIP2R = (RCC->D2CCIP2R & ~RCC_D2CCIP2R_USBSEL) | (3u << RCC_D2CCIP2R_USBSEL_Pos);
    RCC->APB1HENR |= RCC_APB1HENR_CRSEN;
    (void)RCC->APB1HENR;
    CRS->CFGR = (CRS->CFGR & ~CRS_CFGR_SYNCSRC) | (2u << CRS_CFGR_SYNCSRC_Pos); /* USB2 OTG FS SOF */
    CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;

    PWR->CR3 |= PWR_CR3_USB33DEN;
    while (!(PWR->CR3 & PWR_CR3_USB33RDY)) {
    }
    return hse;
}

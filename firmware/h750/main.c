/*
 * OpenSENT firmware - STM32H750 entry point.
 * USB CDC (TinyUSB) <-> line protocol <-> instrument application.
 * SPDX-License-Identifier: MIT
 */
#include "stm32h7xx.h"
#include "tusb.h"

#include "app.h"
#include "clock.h"
#include "hw.h"
#include "la.h"

void hw_early(void);
void hw_poll(void);

static char s_line[256];
static unsigned s_len;

static void la_console(void)
{
    while (tud_cdc_n_available(1)) {
        uint8_t b[64];
        uint32_t n = tud_cdc_n_read(1, b, sizeof b);
        la_rx(b, n);
    }
}

static void console_poll(void)
{
    while (tud_cdc_available()) {
        char buf[64];
        uint32_t n = tud_cdc_read(buf, sizeof buf);
        for (uint32_t i = 0; i < n; i++) {
            char c = buf[i];
            if (c == '\n' || c == '\r') {
                s_line[s_len] = '\0';
                if (s_len) {
                    app_line(s_line);
                }
                s_len = 0;
            } else if (s_len < sizeof s_line - 1) {
                s_line[s_len++] = c;
            } else {
                s_len = sizeof s_line - 1; /* overlong: app_line rejects it */
            }
        }
    }
}

int main(void)
{
    hw_early();
    clock_init();
    hw_init();
    tusb_init();
    NVIC_SetPriority(OTG_FS_IRQn, 3);
    la_init();
    app_init();
    for (;;) {
        tud_task();
        console_poll();
        la_console();
        la_poll();
        app_task();
        hw_poll();
    }
}

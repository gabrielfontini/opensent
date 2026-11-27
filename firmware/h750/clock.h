#ifndef OPENSENT_CLOCK_H
#define OPENSENT_CLOCK_H
#include <stdbool.h>
#include <stdint.h>
extern uint32_t clock_timer_hz;
bool clock_init(void); /* returns false if running from HSI (crystal failed) */
#endif

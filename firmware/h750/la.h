#ifndef OPENSENT_LA_H
#define OPENSENT_LA_H
#include <stdbool.h>
#include <stdint.h>
#define LA_MAX_RATE 33333333u /* 100 MHz / 3; 50 MS/s loses samples (DMA from the D3 GPIO port) */ /* highest sample rate offered to the host */
void        la_init(void);
void        la_rx(const uint8_t *p, unsigned n); /* bytes from the SUMP CDC port */
void        la_poll(void);
bool        la_set_port(char name);
char        la_port(void);
const char *la_state(void);
uint32_t    la_captures(void);
uint32_t    la_rate(void);
void        la_debug(uint32_t *isr, uint32_t *wr, uint16_t *first4);
#endif

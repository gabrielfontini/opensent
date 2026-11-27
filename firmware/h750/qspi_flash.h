#ifndef OPENSENT_QSPI_FLASH_H
#define OPENSENT_QSPI_FLASH_H
#include <stdbool.h>
#include <stdint.h>
uint32_t qspi_flash_init(void); /* JEDEC ID, 0 if no flash answered */
bool qspi_flash_read(uint32_t addr, void *buf, uint32_t n);
bool qspi_flash_erase4k(uint32_t addr);
bool qspi_flash_write(uint32_t addr, const void *buf, uint32_t n);
#endif

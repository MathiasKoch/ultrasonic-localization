#ifndef SPI
#define SPI

#include <stdint.h>


void spi_halt(void);

void spi_start(void);

unsigned int spi_is_running(void);

void spi_setup_master(void);

uint8_t spi_write_uint8(uint8_t x);

uint8_t spi_read_uint8(void);

void spi_csn(uint8_t state);

void spi_dac_csn(uint8_t state);

void spi_ce(uint8_t state);

#endif 

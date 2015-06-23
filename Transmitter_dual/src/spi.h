#ifndef SPI
#define SPI

#include <stdint.h>
#include <avr/io.h>

#define DAC_BUF_SIZE 1024

extern uint16_t sync_pattern[DAC_BUF_SIZE];

unsigned int spi_is_running(void);
void spi_setup_master(void);
uint8_t spi_write_uint8(uint8_t x, uint8_t eoq);
void spi_write_uint16(uint16_t x);
void spi_ce(uint8_t state);
void spi_dma_init_tx();

#endif 

#ifndef DAC
#define DAC

#include <avr/io.h>
#include "spi.h"
#include "adc.h"

#define LOW 0
#define HIGH 1

#define SWITCH_1 0
#define SWITCH_2 1
#define SWITCH_3 2
#define SWITCH_4 3
#define SWITCH_ALL 4

#define DAC_UPDATE 0x1000
#define DAC_DAISY_DISABLE 0x9000
#define DAC_CLEAR_ZERO 0xB000
#define DAC_CLEAR_MID 0xC000


uint32_t dac_signal[DAC_BUF_SIZE];

void dac_init();
void dac_load_data();
void dac_start();
void switch_init();
void switch_set(uint8_t sw, uint8_t enable);


#endif
#ifndef ADC
#define ADC

#include <avr/io.h>
#include <arm_math.h>
#include "sync.h"

#define TIE 0x2
#define TEN 0x1
#define CHN (1<<2)

#define PIT_USER_ADC 0
#define PIT_USER_DAC 1

typedef struct {
   	uint16_t CV1, CV2;
   	uint32_t fs;
   	volatile uint16_t cnt;
   	uint16_t bufsize;
   	uint16_t channel;
   	volatile uint8_t cycle;
   	volatile uint8_t cycleCount;
   	q15_t * destination;

} ADC_VALS;  

volatile uint32_t sampleStartGT;
uint8_t _user;

void dma_init();
int adc_calibrate(void);
void adc_init(ADC_VALS * v);
void init_sample_timer(uint32_t sample_freq, uint8_t ie);
void adc_run(uint8_t enable, uint8_t cycle);
void adc_set_compare(uint16_t c1, uint16_t c2);
void set_mux(uint8_t enable, uint8_t n);
void mux_init(void);
void pit_run(uint8_t enable);

#endif
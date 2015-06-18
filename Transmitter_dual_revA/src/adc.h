#ifndef ADC
#define ADC

#include <avr/io.h>
#include <arm_math.h>
#include "sync.h"

#define TIE 0x2
#define TEN 0x1
#define CHN (1<<2)

typedef struct {
   	uint16_t CV1, CV2;
   	uint32_t fs;
   	volatile uint16_t cnt;
   	uint16_t bufsize;
   	uint16_t channel;
   	uint16_t pause_samples;
   	uint16_t thresh;
   	volatile uint8_t cycle;
   	volatile uint8_t cycleCount;
   	q15_t * destination;

} ADC_VALS;  

volatile uint32_t sampleStartGT;

void dma_init();
int adc_calibrate(void);
void adc_init(ADC_VALS * v);
void adc_run(uint8_t enable, uint8_t cycle);
void adc_set_compare(uint16_t c1, uint16_t c2);
void set_mux(uint8_t enable, uint8_t n);
void mux_init(void);

#endif
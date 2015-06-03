#include <avr/io.h>


#define TIE 0x2
#define TEN 0x1

typedef struct {
   	uint16_t CV1, CV2;
   	uint32_t fs;
   	uint16_t cnt;
   	uint16_t bufsize;
   	uint16_t channel;
   	uint16_t pause_samples;
   	uint16_t thresh;
} ADC_VALS;  

int adc_calibrate(void);
void adc_init(ADC_VALS * v);
void adc_set_compare(uint16_t c1, uint16_t c2);
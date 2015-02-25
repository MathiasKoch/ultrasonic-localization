#ifndef _analog_h_included__
#define _analog_h_included__

#include <stdint.h>

void adc_start();
int16_t adc_read(void);

// ADCx_SC1n
#define COCO 7
#define AIEN 6
#define DIFF 5
#define ADCH4 4
#define ADCH3 3
#define ADCH2 2
#define ADCH1 1
#define ADCH0 0

// ADCx_CFG1
#define ADLPC 7
#define ADIV1 6
#define ADIV0 5
#define ADLSMP 4
#define MODE1 3
#define MODE0 2
#define ADICLK1 1
#define ADICLK0 0

// ADCx_CFG2
#define MUXSEL 4
#define ADACKEN 3
#define ADHSC 2
#define ADLSTS1 1
#define ADLSTS0	0

// ADCx_SC2
#define ADACT 7
#define ADTRG 6
#define ACFE 5
#define ACFGT 4
#define ACREN 3
#define DMAEN 2
#define REFSEL1 1
#define REFSEL0 0

// ADCx_SC3
#define CAL 7
#define CALF 6
#define ADCO 3
#define AVGE 2
#define AVGS1 1
#define AVGS0 0

const uint8_t channel2sc1aADC0[]= { // new version, gives directly the sc1a number. 0x1F=31 deactivates the ADC.
    5, 14, 8, 9, 13, 12, 6, 7, 15, 4, 0, 19, 3, 21, // 0-13, we treat them as A0-A13
    5, 14, 8, 9, 13, 12, 6, 7, 15, 4, // 14-23 (A0-A9)
    31, 31, 31, 31, 31, 31, 31, 31, 31, 31, // 24-33
    0, 19, 3, 21, // 34-37 (A10-A13)
    26, 22, 23, 27, 29, 30 // 38-43: temp. sensor, VREF_OUT, A14, bandgap, VREFH, VREFL. A14 isn't connected to anything in Teensy 3.0.
};


#endif

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

// SIM_CLKDIV1
#define OUTDIV23 27
#define OUTDIV22 26
#define OUTDIV21 25
#define OUTDIV20 24

// SIM_SOPT7
#define ADC0ALTTRGEN 7
#define ADC0TRGSEL3 3
#define ADC0TRGSEL2 2
#define ADC0TRGSEL1 1
#define ADC0TRGSEL0 0

// PIT_TCTRLn
#define TIE 1
#define TEN 0


// PDBx_SC
#define LDMOD1 19
#define LDMOD0 18
#define PDBEIE 17
#define SWTRIG 16
#define PDB_DMAEN 15
#define PRESCALER2 14
#define PRESCALER1 13
#define PRESCALER0 12
#define TRGSEL3 11
#define TRGSEL2 10 
#define TRGSEL1 9
#define TRGSEL0 8
#define PDBEN 7
#define PDBIF 6
#define PDBIE 5
#define MULT1 3
#define MULT0 2
#define CONT 1
#define LDOK 0

// PDBx_CHnS
#define CF7 23
#define CF6 22
#define CF5 21
#define CF4 20
#define CF3 19
#define CF2 18
#define CF1 17
#define CF0 16
#define ERR7 7
#define ERR6 6
#define ERR5 5
#define ERR4 4
#define ERR3 3
#define ERR2 2
#define ERR1 1
#define ERR0 0

// PDBx_IDLY
#define IDLY15 15
#define IDLY14 14
#define IDLY13 13
#define IDLY12 12
#define IDLY11 11
#define IDLY10 10
#define IDLY9 9
#define IDLY8 8
#define IDLY7 7
#define IDLY6 6
#define IDLY5 5
#define IDLY4 4
#define IDLY3 3
#define IDLY2 2
#define IDLY1 1
#define IDLY0 0

const uint8_t channel2sc1aADC0[]= { // new version, gives directly the sc1a number. 0x1F=31 deactivates the ADC.
    5, 14, 8, 9, 13, 12, 6, 7, 15, 4, 0, 19, 3, 21, // 0-13, we treat them as A0-A13
    5, 14, 8, 9, 13, 12, 6, 7, 15, 4, // 14-23 (A0-A9)
    31, 31, 31, 31, 31, 31, 31, 31, 31, 31, // 24-33
    0, 19, 3, 21, // 34-37 (A10-A13)
    26, 22, 23, 27, 29, 30 // 38-43: temp. sensor, VREF_OUT, A14, bandgap, VREFH, VREFL. A14 isn't connected to anything in Teensy 3.0.
};


#endif

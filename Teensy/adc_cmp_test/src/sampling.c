#include "WProgram.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <util/delay.h>
#include <usb_serial.h>
#include "sampling.h"
#include "mk20dx128.h"

#include <usb_serial.h>
#include "xprintf.h"

#define BUFSIZE 50

#define PDB_CH0C1_TOS 0x0100
#define PDB_CH0C1_EN 0x01
#define PDB_SC_PDBIF_MASK 0x40u


static volatile uint8_t calibrating;
static volatile uint8_t init_calib;
static volatile int16_t val;

void adc0_isr(void) {
	val = ADC0_RA;			// grab new reading from ADC
}

void pdb_isr(void){
    PDB0_SC &= ~PDB_SC_PDBIF_MASK;  // clear interrupt mask
    
}



#define PDB_CONFIG (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_PDBIE\
    | PDB_SC_CONT | PDB_SC_PRESCALER(0) | PDB_SC_MULT(0))

#define PDB_PERIOD ((F_BUS / 100000)-1)     // Samplerate

void pdbInit() {
    // Enable PDB clock
    SIM_SCGC6 |= SIM_SCGC6_PDB;
    // Timer period
    PDB0_MOD = PDB_PERIOD;
    // Interrupt delay
    PDB0_IDLY = 0;
    // Enable pre-trigger
    PDB0_CH0C1 = PDB_CH0C1_TOS | PDB_CH0C1_EN;
    // PDB0_CH0DLY0 = 0;
    PDB0_SC = PDB_CONFIG;
    // Software trigger (reset and restart counter)
    PDB0_SC |= PDB_SC_SWTRIG;

    PDB0_SC |= PDB_SC_LDOK;
    NVIC_ENABLE_IRQ(IRQ_PDB);
}

void adcCalibrate() {
    uint16_t sum;

    // Begin calibration
    ADC0_SC3 = ADC_SC3_CAL;
    // Wait for calibration
    while (ADC0_SC3 & ADC_SC3_CAL);

    // Plus side gain
    sum = ADC0_CLPS + ADC0_CLP4 + ADC0_CLP3 + ADC0_CLP2 + ADC0_CLP1 + ADC0_CLP0;
    sum = (sum / 2) | 0x8000;
    ADC0_PG = sum;

    // Minus side gain (not used in single-ended mode)
    sum = ADC0_CLMS + ADC0_CLM4 + ADC0_CLM3 + ADC0_CLM2 + ADC0_CLM1 + ADC0_CLM0;
    sum = (sum / 2) | 0x8000;
    ADC0_MG = sum;
}


int main(void){

	//int i =0;
	init_calib = 1;
	pinMode(2,OUTPUT);
	pinMode(A7, INPUT);
	pinMode(A14, OUTPUT);

	pdbInit();





	xdev_out(usb_serial_putchar);
	
	//ADC0_SC3 |= _BV(ADCO);

	__disable_irq();

	ADC0_CFG1 |= ADC_CFG1_ADIV(0) | ADC_CFG1_MODE(2);
	ADC0_CFG2 |= ADC_CFG2_MUXSEL;
	ADC0_SC2 |= ADC_SC2_REFSEL(0);
	adcCalibrate();

	//ADC0_SC2 |= ADC_SC2_ACFE ;// | ADC_SC2_ACREN;
    //*(int16_t *)&(ADC0_CV1) = -26900;

	ADC0_SC1A = ADC_SC1_AIEN | ADC_SC1_ADCH(0b00110);
	NVIC_ENABLE_IRQ(IRQ_ADC0);

	__enable_irq();

	delay(500);
	float value = 0.02;
	analogWrite(A14, (int)(value*2048.0));

	while (1) {
			xprintf("%d\r\n", val);

	}
}

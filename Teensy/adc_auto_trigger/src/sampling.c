#include "WProgram.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <util/delay.h>
#include <usb_serial.h>
#include "sampling.h"

#define BUFSIZE 50

#define HEX(n) (((n) < 10) ? ((n) + '0') : ((n) + 'A' - 10))	

static volatile uint8_t head, tail;
static volatile int16_t buffer[BUFSIZE];
static volatile uint8_t calibrating;
static volatile uint8_t init_calib;


int16_t adc_read(void){
	uint8_t h, t;
	int16_t val;
	do {
		h = head;
		t = tail;			// wait for data in buffer
	} while (h == t);
	if (++t >= BUFSIZE) t = 0;
	val = buffer[t];		// remove 1 sample from buffer
	tail = t;
	return val;
}


void adc0_isr(void) {
	uint8_t h;
	int16_t val;

	val = ADC0_RA;			// grab new reading from ADC
	h = head + 1;
	if (h >= BUFSIZE) h = 0;
	if (h != tail) {		// if the buffer isn't full
		buffer[h] = val;	// put new data into buffer
		head = h;
	}
}

void calibrate(void){

	ADC0_SC3 |= _BV(AVGE) | _BV(AVGS1) | _BV(AVGS0);
	ADC0_CFG2 &= ~_BV(ADHSC) & ~_BV(ADLSTS1) & ~_BV(ADLSTS0);
	ADC0_CFG1 |= _BV(ADLPC) | _BV(ADLSMP);

	__disable_irq();
	calibrating = 1;
	ADC0_SC3 &= ~_BV(CAL);
	ADC0_SC3 |= _BV(CALF);
	ADC0_SC3 |= _BV(CAL);
	__enable_irq();
}

void wait_for_cal(void){
	uint16_t sum;
	while(ADC0_SC3 & _BV(CAL)){

	}
	__disable_irq();
	if(calibrating){
		sum = ADC0_CLPS + ADC0_CLP4 + ADC0_CLP3 + ADC0_CLP2 + ADC0_CLP1 + ADC0_CLP0;
		sum = (sum / 2) | 0x8000;
		ADC0_PG = sum;

		sum = ADC0_CLMS + ADC0_CLM4 + ADC0_CLM3 + ADC0_CLM2 + ADC0_CLM1 + ADC0_CLM0;
		sum = (sum / 2) | 0x8000;
		ADC0_MG = sum;

		calibrating = 0;
	}
	__enable_irq();
	if(init_calib){
		ADC0_SC3 &= ~_BV(AVGE);
		ADC0_CFG2 |= _BV(ADHSC) | _BV(ADLSTS1) | _BV(ADLSTS0);
		ADC0_CFG1 |= _BV(ADLSMP);
		ADC0_CFG1 &= ~_BV(ADLPC);
		init_calib = 0;
	}
}

void recalibrate(void){
	calibrate();
	wait_for_cal();
}

const uint8_t channel2sc1aADC0[]= { // new version, gives directly the sc1a number. 0x1F=31 deactivates the ADC.
    5, 14, 8, 9, 13, 12, 6, 7, 15, 4, 0, 19, 3, 21, // 0-13, we treat them as A0-A13
    5, 14, 8, 9, 13, 12, 6, 7, 15, 4, // 14-23 (A0-A9)
    31, 31, 31, 31, 31, 31, 31, 31, 31, 31, // 24-33
    0, 19, 3, 21, // 34-37 (A10-A13)
    26, 22, 23, 27, 29, 30 // 38-43: temp. sensor, VREF_OUT, A14, bandgap, VREFH, VREFL. A14 isn't connected to anything in Teensy 3.0.
};

int main(void){
	uint16_t val;
	char buf[4];
	init_calib = 1;
	pinMode(13,OUTPUT);
	pinMode(A9, INPUT);
	pinMode(A14, OUTPUT);
    analogWriteResolution(12);

	recalibrate();
	if (calibrating) wait_for_cal();

	//ADC0_SC1A |= _BV(AIEN);
	NVIC_ENABLE_IRQ(IRQ_ADC0);

	ADC0_CFG1 &= ~_BV(MODE1) & ~_BV(ADICLK1) & ~_BV(ADICLK0);	// 12 bit single-ended, and busclock as input clock
	ADC0_CFG1 |= _BV(MODE0);
	ADC0_CFG2 |= _BV(MUXSEL);
	__disable_irq();
	ADC0_SC3 |= _BV(ADCO);
	ADC0_SC1A = channel2sc1aADC0[A9] + _BV(AIEN);
	__enable_irq();
	delay(500);
	float val = 0.47;
	analogWrite(A14, (int)(val*2048.0));

	while (1) {
		val = adc_read();
		buf[0] = HEX((val >> 8) & 15);
		buf[1] = HEX((val >> 4) & 15);
		buf[2] = HEX(val & 15);
		buf[3] = ' ';
		usb_serial_write((unsigned char *)buf, 4);
		usb_serial_flush_output();
	}
}

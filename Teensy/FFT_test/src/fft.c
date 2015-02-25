#include "WProgram.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <util/delay.h>
#include <usb_serial.h>
#define ARM_MATH_CM4
#include <arm_math.h>
#include "fft.h"

#define BUFSIZE 2048

#define HEX(n) (((n) < 10) ? ((n) + '0') : ((n) + 'A' - 10))	

extern float32_t buffer3[BUFSIZE]; 
static volatile uint8_t buf_index;
static q15_t buffer1[BUFSIZE];
static q15_t buffer2[BUFSIZE];
static q15_t h[BUFSIZE];
static volatile uint8_t calibrating;
static volatile uint8_t init_calib;
static volatile uint8_t buf_select;
static volatile uint8_t buf_dsp;

extern const uint8_t channel2sc1aADC0[];

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

void start_adc(void){
	pinMode(A9, INPUT);

	NVIC_ENABLE_IRQ(IRQ_ADC0);

	ADC0_CFG1 &= ~_BV(MODE1) & ~_BV(ADICLK1) & ~_BV(ADICLK0);	// 12 bit single-ended, and busclock as input clock
	ADC0_CFG1 |= _BV(MODE0);
	ADC0_CFG2 |= _BV(MUXSEL);
	__disable_irq();
	ADC0_SC3 |= _BV(ADCO);
	ADC0_SC1A = channel2sc1aADC0[A9] + _BV(AIEN);
	__enable_irq();
}

void set_gain(float gain){
	analogWrite(A14, (int)(gain*2048.0));
}

void set_compare(uint16_t comp_val){
	ADC0_CV1 = comp_val;
}

void enable_compare(uint8_t enable){
	if(enable)
		ADC0_SC2 |= _BV(ACFE);
	else
		ADC0_SC2 &= ~_BV(ACFE);
}

int16_t adc_read(void){
	return 1;
}

void hilbert_init(void){
	int i;
	if(BUFSIZE > 0 && 2*round(BUFSIZE/2) == BUFSIZE){
	  h[1] = 1;
	  h[BUFSIZE/2+1] = 1;
	  for(i = 2; i < BUFSIZE/2; i++){
	  	h[i] = 2;
	  }
	}else if(BUFSIZE>0){
	  h[1] = 1;
	  for(i = 2; i <= (BUFSIZE+1)/2; i++)
	  	h[i] = 2;
	}
}

void hilbert(arm_cfft_radix4_instance_q15 S, arm_cfft_radix4_instance_q15 Si, q15_t buffer){
	
}


void adc0_isr(void) {
	// Disable comparator
	enable_compare(0);

	if(buf_index <= BUFSIZE){
		if(buf_select == 1){
			buffer1[buf_index++] = ADC0_RA << 4;			// put new data into buffer in q15 format
			buffer1[buf_index++] = 0;
		}else{
			buffer2[buf_index++] = ADC0_RA << 4;
			buffer2[buf_index++] = 0;
		}
	}else{
		// Enable comparator, reset buffer index, allow DSP and start buffering in the other buffer
		enable_compare(1);
		buf_index = 0;
		buf_dsp = buf_select;
		buf_select = buf_select==2? 1: 2;
	}
}

int main(void){
	//char buf[4];
	buf_index = 0;
	init_calib = 1;
	buf_dsp = 1;
	pinMode(13, OUTPUT);
	
	/*pinMode(A14, OUTPUT);
    analogWriteResolution(12);

	recalibrate();
	if (calibrating) wait_for_cal();

	start_adc();

	set_gain(0.3);*/
	
	
	//arm_cfft_radix4_instance_q15 S; 
	arm_cfft_radix4_instance_f32 S; 
	//arm_cfft_radix4_instance_q15 Si; 
	arm_cfft_radix4_init_f32(&S, 1024, 0, 1); 
	//arm_cfft_radix4_init_q15(&Si, 256, 1, 1); 
//	hilbert_init();



	while (1) {
		
			//q15_t tmp[BUFSIZE];
		if(buf_dsp == 1){
			arm_cfft_radix4_f32(&S, buffer3);
			//arm_mat_mult_fast_q15(&buffer3, &h, &buffer3, &tmp);
			//arm_cfft_radix4_q15(&Si, &buffer3);

			buf_dsp = 0;
		}
		char buf[10];
		
		int i;
		for(i = 0; i < BUFSIZE; i++){
			sprintf(buf, "%f", buffer3[i]);
			buf[9] = ' ';
			usb_serial_write((unsigned char *)buf, 10);
			usb_serial_flush_output();
		}
		usb_serial_write((unsigned char *)'#', 1);
		usb_serial_flush_output();
		//if(testIndex ==  refIndex) 
			digitalWrite(13, !digitalRead(13));
		delay(100);
	}
}

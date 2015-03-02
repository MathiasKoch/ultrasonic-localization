#include "WProgram.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdio.h>
#include <util/delay.h>
#include <usb_serial.h>
#define ARM_MATH_CM4
#include <arm_math.h>
#include "fft.h"
#include "mk20d7.h"

#define BUFSIZE 512

#define HEX(n) (((n) < 10) ? ((n) + '0') : ((n) + 'A' - 10))	

//extern q15_t buffer3[BUFSIZE]; 
//extern float32_t buffer4[BUFSIZE]; 
static volatile uint16_t buf_index;
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

/*void init_timer(uint32_t sampleRate){
	SIM_SCGC6 |= SIM_SCGC6_PIT;
	PIT_MCR = 0x00;
	NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
	PIT_LDVAL0 = 0x0003E7FF;//(1/sampleRate)/(1/36000000.0) - 1;
	PIT_TCTRL0 |= TEN;
	PIT_TFLG0 |= 1;
}*/

void pdb_init(uint16_t period){
 
  /* Software trigger, error interrupts enabled, continuous mode. */
  PDB0_SC |= _BV(TRGSEL3) | _BV(TRGSEL2) | _BV(TRGSEL1) | _BV(TRGSEL0)
           | _BV(PDBIE) | _BV(CONT) | _BV(MULT1) | _BV(MULT0);
 
  /* Set the PDB modulus. */
  PDB0_MOD = period;
 
  /* Disable PDB Interrupt Delay (i.e. don't generate interrupt). */
  PDB0_IDLY = 0xFFFF;
 
  /* Pretriggers 0 and 1 are both enabled. */
  //PDB0_CH0C1 =  PDB_C1_TOS(0x03) | PDB_C1_EN(0x03);             
 
  /* Clear channel flags. Clear channel sequence error flags. */
  PDB0_CH0S |= _BV(ERR7) | _BV(ERR6) | _BV(ERR5) | _BV(ERR4) | _BV(ERR3) | _BV(ERR2) | _BV(ERR1) | _BV(ERR0);  
 
  /* Channel 0 delay. */
  //PDB0_CH0DLY0 = PDB_DLY_DLY(period/2);
  //PDB0_CH0DLY1 = PDB_DLY_DLY(0xFFFF);
 
  /* Start module. */
  PDB0_SC |= _BV(PDBEN) | _BV(LDOK);
 
  // NOTE: We're not using the Pulse-Out module.
}

void start_adc(void){
	pinMode(A9, INPUT);

	NVIC_ENABLE_IRQ(IRQ_ADC0);
	//SIM_SOPT7 |= _BV(ADC0ALTTRGEN) | _BV(ADC0TRGSEL2);	// Enable PIT timer0 as trigger source
	/* Use PDB for hardware trigger. */
	SIM_SOPT7 = 0;

	ADC0_CFG1 &= ~_BV(MODE1) & ~_BV(ADICLK1) & ~_BV(ADICLK0);	// 12 bit single-ended, and busclock as input clock
	ADC0_CFG1 |= _BV(MODE0);
	ADC0_CFG2 |= _BV(MUXSEL);
	//ADC0_SC3 |= _BV(ADCO);
	ADC0_SC2 |= _BV(ADTRG);
	__disable_irq();
	


	ADC0_SC1A = channel2sc1aADC0[A9] + _BV(AIEN);
	__enable_irq();
}

void set_gain(float gain){
	analogWrite(A14, (int)(gain*2048.0));
}

void set_compare(uint16_t comp_val){
	ADC0_CV1 = comp_val;
}

void enable_compare(int enable){
	__disable_irq();
	if(enable == 1)
		ADC0_SC2 |= _BV(ACFE);
	else
		ADC0_SC2 &= ~_BV(ACFE);
	ADC0_SC1A = channel2sc1aADC0[A9] + _BV(AIEN);
	__enable_irq();
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
	digitalWrite(13, !digitalRead(13));	
	delay(100);
	// Disable comparator
	//enable_compare(0);
		if(buf_index <= BUFSIZE){

			//if(buf_select == 1){
				buffer1[buf_index++] = ADC0_RA;			// put new data into buffer in q15 format
				//buffer1[buf_index++] = 0;
			/*}else{
				buffer2[buf_index++] = ADC0_RA;
				buffer2[buf_index++] = 0;
			}*/
		}else{

			// Enable comparator, reset buffer index, allow DSP and start buffering in the other buffer
			enable_compare(1);
			//buf_index = 0;
			buf_dsp = 1;
			//buf_select = buf_select==2? 1: 2;
		}
}

int main(void){
	char buf[10];
	buf_index = 0;
	init_calib = 1;
	
	pinMode(13, OUTPUT);
	
	pinMode(A14, OUTPUT);
    analogWriteResolution(12);

    // Set busclock = system_clock/2 = 36 MHz
    SIM_CLKDIV1 |= _BV(OUTDIV20);

	recalibrate();
	if (calibrating) wait_for_cal();

	//init_timer(2.0);
	pdb_init(50000);
	start_adc();

	set_gain(0.3);
	//set_compare(2048);
	
	arm_cfft_radix4_instance_q15 S; 
	arm_cfft_radix4_instance_q15 Si; 
	arm_cfft_radix4_init_q15(&S, 1024, 0, 1); 
	arm_cfft_radix4_init_q15(&Si, 1024, 1, 1); 
	hilbert_init();

	//q15_t tmp[BUFSIZE];
	int i;
	buf_dsp = 0;
	

	while (1) {
		if(buf_dsp == 1){
			//arm_float_to_q15(buffer4, tmp, BUFSIZE);
			arm_cfft_radix4_q15(&S, buffer1);

			// __SMLAD <------- SIMD function
			for(i = 0; i<BUFSIZE;i++){
				buffer1[i] *= h[i];
			}

			arm_cfft_radix4_q15(&Si, buffer1);
			buf_dsp = 0;
		}
			
			
			for(i = 0; i < BUFSIZE; i++){
				
				sprintf(buf, "%d", buffer1[i]);
				buf[9] = ' ';
				usb_serial_write((unsigned char *)buf, 10);
				usb_serial_flush_output();
			}
			//usb_serial_write((unsigned char *)'#', 1);
			//usb_serial_flush_output();


			//buf_dsp = 0;
		//}

	}
}

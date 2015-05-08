#include <avr/io.h>
#include "adc.h"


void adc_calibrate(void){

	ADC0_SC3 |= ADC_SC3_AVGE | ADC_SC3_AVGS(3);
	ADC0_CFG2 &= ~ADC_CFG2_ADHSC & ~ADC_CFG2_ADLSTS(3);
	ADC0_CFG1 |= ADC_CFG1_ADLPC | ADC_CFG1_ADLSMP;

	calibrating = 1;
	ADC0_SC3 &= ~ADC_SC3_CAL;
	ADC0_SC3 |= ADC_SC3_CALF;
	ADC0_SC3 |= ADC_SC3_CAL;
}

void adc_wait_for_cal(void){
	uint16_t sum;
	while(ADC0_SC3 & ADC_SC3_CAL){

	}
	if(calibrating){
		sum = ADC0_CLPS + ADC0_CLP4 + ADC0_CLP3 + ADC0_CLP2 + ADC0_CLP1 + ADC0_CLP0;
		sum = (sum / 2) | 0x8000;
		ADC0_PG = sum;

		sum = ADC0_CLMS + ADC0_CLM4 + ADC0_CLM3 + ADC0_CLM2 + ADC0_CLM1 + ADC0_CLM0;
		sum = (sum / 2) | 0x8000;
		ADC0_MG = sum;

		calibrating = 0;
	}
	if(init_calib){
		ADC0_SC3 &= ~ADC_SC3_AVGE;
		ADC0_CFG2 |= ADC_CFG2_ADHSC | ADC_CFG2_ADLSTS(3);
		ADC0_CFG1 |= ADC_CFG1_ADLSMP;
		ADC0_CFG1 &= ~ADC_CFG1_ADLPC;
		init_calib = 0;
	}
}

void adc_recalibrate(void){
	adc_calibrate();
	adc_wait_for_cal();
}

void adc_start(void){
	pinMode(A9, INPUT);

	NVIC_ENABLE_IRQ(IRQ_ADC0);
	//SIM_SOPT7 |= _BV(ADC0ALTTRGEN) | _BV(ADC0TRGSEL2);	// Enable PIT timer0 as trigger source
	/* Use PDB for hardware trigger. */
	SIM_SOPT7 = 0;

	ADC0_CFG1 &= ~ADC_CFG1_MODE(2) & ~ADC_CFG1_ADICLK(3);	// 12 bit single-ended, and busclock as input clock
	ADC0_CFG1 |= ADC_CFG1_MODE(1);
	ADC0_CFG2 |= ADC_CFG2_MUXSEL;
	//ADC0_SC3 |= _BV(ADCO);
	ADC0_SC2 |= ADC_SC2_ADTRG;
	


	ADC0_SC1A = channel2sc1aADC0[A9] + ADC_SC1_AIEN;
}

void adc_set_compare(uint16_t comp_val){
	ADC0_CV1 = comp_val;
}

void adc_enable_compare(int enable){
	if(enable == 1)
		ADC0_SC2 |= ADC_SC2_ACFE;
	else
		ADC0_SC2 &= ~ADC_SC2_ACFE;
	ADC0_SC1A = channel2sc1aADC0[A9] + ADC_SC1_AIEN;
}
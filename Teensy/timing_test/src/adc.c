#include "adc.h"

#include "xprintf.h"

#define TIE 0x2
#define TEN 0x1

ADC_VALS * v;

int adc_calibrate(void){
    ADC1_CFG1 |= (ADC_CFG1_MODE(3)  |       // 16 bits mode
                  ADC_CFG1_ADICLK(1)|       // Input Bus Clock divided by 2 (20-25 MHz out of reset (FEI mode) / 2)
                  ADC_CFG1_ADIV(2)) ;       // Clock divide by 4 (2.5-3 MHz)
    
    ADC1_SC3 |= ADC_SC3_AVGE        |       // Enable HW average
                ADC_SC3_AVGS(3)     |       // Set HW average of 32 samples
                ADC_SC3_CAL;   

    while(ADC1_SC3 & ADC_SC3_CAL);

    if(ADC1_SC3 & ADC_SC3_CALF)   // Check for successful calibration
        return 1; 

    uint16_t sum;
    sum = ADC1_CLPS + ADC1_CLP4 + ADC1_CLP3 + ADC1_CLP2 + ADC1_CLP1 + ADC1_CLP0;
    ADC1_PG = (sum / 2) | 0x8000;

    sum = ADC1_CLMS + ADC1_CLM4 + ADC1_CLM3 + ADC1_CLM2 + ADC1_CLM1 + ADC1_CLM0;
    ADC1_MG = (sum / 2) | 0x8000;
    return 0;
}

void adc_init(ADC_VALS * vals) {
    v = vals;
    SIM_SCGC3 |= SIM_SCGC3_ADC1;
    adc_calibrate();
    v->cnt = 0;
    ADC1_CFG1 = ADC_CFG1_ADIV(1) | ADC_CFG1_MODE(2) | ADC_CFG1_ADICLK(0);
    ADC1_CFG2 = ADC_CFG2_MUXSEL;
    // Voltage ref vcc, hardware trigger, DMA
    ADC1_SC2 |= ADC_SC2_DMAEN;
    //ADC1_SC2 |= ADC_SC2_ACFE | ADC_SC2_ACFGT | ADC_SC2_ACREN | ADC_SC2_DMAEN;
    ADC1_SC3 = 0;
    ADC1_CV1 = v->CV1;     // ~2.2V
    ADC1_CV2 = v->CV2;     // ~1.8V
    NVIC_ENABLE_IRQ(IRQ_ADC1);
    ADC1_SC1A |= ADC_SC1_ADCH(0x1F);

    // PIT 1 for ADC triggering at FS [Hz]
    SIM_SCGC6 |= SIM_SCGC6_PIT;
    PIT_MCR = 0x00;
    NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
    PIT_LDVAL0 = ((F_BUS / v->fs)-1);
    PIT_TCTRL0 = TIE;
    PIT_TCTRL0 |= TEN;
}

void adc_set_compare(uint16_t c1, uint16_t c2){
	v->CV1 = c1;
	v->CV2 = c2;
}


void pit0_isr(){
    PIT_TFLG0 = 1;
    /*if(v->cnt >= (v->bufsize + v->pause_samples)){
        ADC1_CV1 = v->CV1;     // ~2.2V
        ADC1_CV2 = v->CV2;     // ~1.8V
        ADC1_SC2 |= ADC_SC2_ACFE;
        v->cnt = 0;
    }
    if(v->cnt == 1)
        ADC1_SC2 &= ~ADC_SC2_ACFE;

    if(v->cnt < v->bufsize)*/
        ADC1_SC1A = ADC_SC1_AIEN | ADC_SC1_ADCH(v->channel);
    /*else if(v->cnt != 0)
        v->cnt++;*/
}

void adc1_isr(void){
    v->cnt++;
    //xprintf("Here");
   // GPIOD_PDOR ^= 1<<0;
}
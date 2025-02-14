#include "adc.h"



ADC_VALS * v;

void dma_init(q15_t * address) {
    // Enable DMA, DMAMUX clocks
    SIM_SCGC7 |= SIM_SCGC7_DMA;
    SIM_SCGC6 |= SIM_SCGC6_DMAMUX;

    // Use default configuration
    DMA_CR = 0;

    // Source address
    DMA_TCD0_SADDR = &ADC0_RA;
    // Don't change source address
    DMA_TCD0_SOFF = 0;
    DMA_TCD0_SLAST = 0;
    // Destination address
    DMA_TCD0_DADDR = address;
    // Destination offset (2 byte)
    DMA_TCD0_DOFF = 2;
    // Restore destination address after major loop
    DMA_TCD0_DLASTSGA = -sizeof(address);
    // Source and destination size 16 bit
    DMA_TCD0_ATTR = DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_DSIZE(1);
    // Number of bytes to transfer (in each service request)
    DMA_TCD0_NBYTES_MLNO = 2;
    // Set loop counts
    DMA_TCD0_CITER_ELINKNO = sizeof(address) / 2;
    DMA_TCD0_BITER_ELINKNO = sizeof(address) / 2;
    // Enable interrupt (end-of-major loop)
    DMA_TCD0_CSR = DMA_TCD_CSR_INTMAJOR;

    // Set ADC as source (CH 0), enable DMA MUX
    DMAMUX0_CHCFG0 = DMAMUX_DISABLE;
    DMAMUX0_CHCFG0 = DMAMUX_SOURCE_ADC0 | DMAMUX_ENABLE;

    // Enable request input signal for channel 0
    DMA_SERQ = 0;


    // Enable interrupt request
    NVIC_ENABLE_IRQ(IRQ_DMA_CH0);
}

int adc_calibrate(void){
    ADC0_CFG1 |= (ADC_CFG1_MODE(3)  |       // 16 bits mode
                  ADC_CFG1_ADICLK(1)|       // Input Bus Clock divided by 2 (20-25 MHz out of reset (FEI mode) / 2)
                  ADC_CFG1_ADIV(2)) ;       // Clock divide by 4 (2.5-3 MHz)
    
    ADC0_SC3 |= ADC_SC3_AVGE        |       // Enable HW average
                ADC_SC3_AVGS(3)     |       // Set HW average of 32 samples
                ADC_SC3_CAL;   

    while(ADC0_SC3 & ADC_SC3_CAL);

    if(ADC0_SC3 & ADC_SC3_CALF)   // Check for successful calibration
        return 1; 

    uint16_t sum;
    sum = ADC0_CLPS + ADC0_CLP4 + ADC0_CLP3 + ADC0_CLP2 + ADC0_CLP1 + ADC0_CLP0;
    ADC0_PG = (sum / 2) | 0x8000;

    sum = ADC0_CLMS + ADC0_CLM4 + ADC0_CLM3 + ADC0_CLM2 + ADC0_CLM1 + ADC0_CLM0;
    ADC0_MG = (sum / 2) | 0x8000;
    return 0;
}

void adc_init(ADC_VALS * vals) {
    v = vals;
    SIM_SCGC6 |= SIM_SCGC6_ADC0;
    adc_calibrate();
    v->cnt = 0;
    v->thresh = (v->bufsize + v->pause_samples);
    ADC0_CFG1 = ADC_CFG1_ADIV(1) | ADC_CFG1_MODE(2) | ADC_CFG1_ADICLK(0);
    ADC0_CFG2 = ADC_CFG2_MUXSEL;
    //ADC0_SC2 |= ADC_SC2_DMAEN;
    ADC0_SC2 |= ADC_SC2_ACFE | ADC_SC2_ACFGT | ADC_SC2_ACREN | ADC_SC2_DMAEN;
    ADC0_SC3 = 0;
    ADC0_CV1 = v->CV1;     // ~2.2V
    ADC0_CV2 = v->CV2;     // ~1.8V
    NVIC_ENABLE_IRQ(IRQ_ADC0);
    ADC0_SC1A |= ADC_SC1_ADCH(0x1F);    // Disable ADC

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
    if(v->cnt >= v->thresh){
        ADC0_CV1 = v->CV1;     // ~ mean + 0.2V
        ADC0_CV2 = v->CV2;     // ~ mean - 0.2V
        ADC0_SC2 |= ADC_SC2_ACFE;
        v->cnt = 0;
    }
    if(v->cnt == 1)
        ADC0_SC2 &= ~ADC_SC2_ACFE;

    if(v->cnt < v->bufsize)
        ADC0_SC1A = ADC_SC1_AIEN | ADC_SC1_ADCH(v->channel);
    else if(v->cnt != 0)
        v->cnt++;
}

void adc0_isr(void){
    //GPIOD_PDOR ^= 1<<0;
    v->cnt++;
}
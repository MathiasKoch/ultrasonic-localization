#include "adc.h"
#include "xprintf.h"

ADC_VALS * v;

void dma_init() {
    // Enable DMA, DMAMUX clocks
    SIM_SCGC7 |= SIM_SCGC7_DMA;
    SIM_SCGC6 |= SIM_SCGC6_DMAMUX;

    // Use default configuration
    DMA_CR = 0;

    // Source address
    DMA_TCD4_SADDR = &ADC1_RA;
    // Don't change source address
    DMA_TCD4_SOFF = 0;
    DMA_TCD4_SLAST = 0;
    // Destination address
    DMA_TCD4_DADDR = v->destination;
    // Destination offset (2 byte)
    DMA_TCD4_DOFF = 2;
    // Restore destination address after major loop
    DMA_TCD4_DLASTSGA = -v->bufsize*2;
    // Source and destination size 16 bit
    DMA_TCD4_ATTR = DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_DSIZE(1);
    // Number of bytes to transfer (in each service request)
    DMA_TCD4_NBYTES_MLNO = 2;
    // Set loop counts
    DMA_TCD4_CITER_ELINKNO = v->bufsize;
    DMA_TCD4_BITER_ELINKNO = v->bufsize;
    // Enable interrupt (end-of-major loop)
    DMA_TCD4_CSR = DMA_TCD_CSR_INTMAJOR;

    // Set ADC as source (CH 0), enable DMA MUX
    DMAMUX0_CHCFG4 = DMAMUX_DISABLE;
    DMAMUX0_CHCFG4 = DMAMUX_SOURCE_ADC1 | DMAMUX_ENABLE;

    // Enable request input signal for channel 0
    DMA_SERQ = 4;


    // Enable interrupt request
    NVIC_ENABLE_IRQ(IRQ_DMA_CH4);
}

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
    xprintf("Initializing ADC module\t\t - \t");

    v = vals;
    SIM_SCGC3 |= SIM_SCGC3_ADC1;
    mux_init();
    adc_calibrate();
    v->cnt = 0;
    v->cycleCount = 0x00;
    ADC1_CFG1 = ADC_CFG1_ADIV(1) | ADC_CFG1_MODE(2) | ADC_CFG1_ADICLK(0);
    ADC1_CFG2 = ADC_CFG2_MUXSEL;
    //ADC1_SC2 |= ADC_SC2_DMAEN;
    ADC1_SC2 |= ADC_SC2_ACFE | ADC_SC2_ACFGT | ADC_SC2_ACREN | ADC_SC2_DMAEN;
    ADC1_SC3 = 0;
    ADC1_CV1 = v->CV1;     // ~2.2V
    ADC1_CV2 = v->CV2;     // ~1.8V
    NVIC_ENABLE_IRQ(IRQ_ADC1);
    ADC1_SC1A |= ADC_SC1_ADCH(0x1F);    // Disable ADC

    init_sample_timer(v->fs, PIT_USER_ADC);

    dma_init();
    pinMode(24,OUTPUT);
    xprintf("Done\r\n");

}

void init_sample_timer(uint32_t sample_freq, uint8_t user){
    // PIT 0 for ADC triggering at FS [Hz]
    // TODO: Separate ADC and DAC timers.
    SIM_SCGC6 |= SIM_SCGC6_PIT;
    PIT_MCR = 0x00;
    PIT_TCTRL0 &= ~TEN;
    NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
    PIT_LDVAL0 = ((F_BUS / sample_freq)-1);
    PIT_TCTRL0 = TIE;
    _user = user;
}

void adc_run(uint8_t enable, uint8_t cycle){
    if(enable == 1){
        init_sample_timer(v->fs, PIT_USER_ADC);
        ADC1_CV1 = v->CV1;     // ~ mean + 0.2V
        ADC1_CV2 = v->CV2;     // ~ mean - 0.2V
        ADC1_SC2 |= ADC_SC2_ACFE;
        v->cnt = 0;
        v->cycle = cycle;
        v->cycleCount = 0x00;
        set_mux(1,0x00);
        pit_run(1);
    }
}

void pit_run(uint8_t enable){
    if(enable == 1){
        PIT_TCTRL0 |= TEN;
    }else{
        PIT_TCTRL0 &= ~TEN;
    }
}

void adc_set_compare(uint16_t c1, uint16_t c2){
	v->CV1 = c1;
	v->CV2 = c2;
}


void pit0_isr(){
    PIT_TFLG0 = 1;
    if(_user == PIT_USER_DAC){
        DMA_TCD0_CSR |= DMA_TCD_CSR_START;
    }else{
        if(v->cnt == 1){
            __disable_irq();
            sampleStartGT = PIT_CVAL2;
            __enable_irq();
            v->cycle = 0;
            ADC1_SC2 &= ~ADC_SC2_ACFE;
            v->cnt++;
        }
        if(v->cycle)
            set_mux(1,v->cycleCount++%0x04); 
        ADC1_SC1A = ADC_SC1_AIEN | ADC_SC1_ADCH(v->channel);
    }
}

void set_mux(uint8_t enable, uint8_t n){
    // TODO: Take dip switch positions into account
    if(enable == 1){
        GPIOA_PDOR |= 1<<4;
        /*if((n & 0x1))
            GPIOE_PCOR = 1;
        else
            GPIOE_PSOR = 1;
        if((n & 0x2))
            GPIOB_PCOR = 19;
        else
            GPIOB_PSOR = 19;*/
        GPIOB_PDOR |= (1<<19);
        GPIOE_PDOR |= (1<<1);

    }else{
        GPIOA_PDOR &= ~(1<<4);
    }
}

void mux_init(){
    if((SIM_SCGC5 & SIM_SCGC5_PORTA) == 0)
       SIM_SCGC5 |= SIM_SCGC5_PORTA;
    if((SIM_SCGC5 & SIM_SCGC5_PORTB) == 0)
       SIM_SCGC5 |= SIM_SCGC5_PORTB;
    if((SIM_SCGC5 & SIM_SCGC5_PORTE) == 0)
       SIM_SCGC5 |= SIM_SCGC5_PORTE;
   // TODO: Fix this pinMode
    /*PORTA_PCR4 |= PORT_PCR_MUX(1) | PORT_PCR_PE;
    GPIOA_PDDR |= 1<<4;                 // Enable mux
    GPIOA_PDOR &= ~(1<<4);              // Init to disabled mux, active low*/
    pinMode(33,OUTPUT);

    PORTE_PCR1 |= PORT_PCR_MUX(1);      // Mux A0
    GPIOE_PDDR |= 1<<1;
    PORTB_PCR19 |= PORT_PCR_MUX(1);     // Mux A1
    GPIOB_PDDR |= 1<<19;   

    set_mux(1, 0x00);
}

void adc1_isr(void){    
    v->cnt++;
}
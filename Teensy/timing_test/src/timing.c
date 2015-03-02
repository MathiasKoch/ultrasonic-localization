
#include <avr/io.h>
#include <common.h>

#define PDB_CH0C1_TOS 0x0100
#define PDB_CH0C1_EN 0x01


uint16_t samples[256];
uint16_t samples2[256];
int bufferFlag;


static const uint8_t channel2sc1a[] = {
    5, 14, 8, 9, 13, 12, 6, 7, 15, 4,
    0, 19, 3, 21, 26, 22
};

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

/*
    ADC_CFG1_ADIV(2)         Divide ratio = 4 (F_BUS = 48 MHz => ADCK = 12 MHz)
    ADC_CFG1_MODE(2)         Single ended 10 bit mode
    ADC_CFG1_ADLSMP          Long sample time
*/
#define ADC_CONFIG1 (ADC_CFG1_ADIV(1) | ADC_CFG1_MODE(2) | ADC_CFG1_ADLSMP)

/*
    ADC_CFG2_MUXSEL          Select channels ADxxb
    ADC_CFG2_ADLSTS(3)       Shortest long sample time
*/
#define ADC_CONFIG2 (ADC_CFG2_MUXSEL | ADC_CFG2_ADLSTS(3))


void adcInit() {
    ADC0_CFG1 = ADC_CONFIG1;
    ADC0_CFG2 = ADC_CONFIG2;
    // Voltage ref vcc, hardware trigger, DMA
    ADC0_SC2 = ADC_SC2_REFSEL(0) | ADC_SC2_ADTRG | ADC_SC2_DMAEN;

    adcCalibrate();


    ADC0_SC1A = ADC_SC1_AIEN |channel2sc1a[3];
}

/*
    PDB_SC_TRGSEL(15)        Select software trigger
    PDB_SC_PDBEN             PDB enable
    PDB_SC_PDBIE             Interrupt enable
    PDB_SC_CONT              Continuous mode
    PDB_SC_PRESCALER(0)      Prescaler = 1
    PDB_SC_MULT(0)           Prescaler multiplication factor = 1
*/
#define PDB_CONFIG (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN \
    | PDB_SC_CONT | PDB_SC_PRESCALER(0) | PDB_SC_MULT(0))

#define PDB_PERIOD ((F_BUS / 2 / 200000)-1)

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
    PDB0_SC = PDB_CONFIG | PDB_SC_LDOK;
    // Software trigger (reset and restart counter)
    PDB0_SC |= PDB_SC_SWTRIG;
}

void dmaInit() {
    bufferFlag = 0;
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
    DMA_TCD0_DADDR = samples;
    // Destination offset (2 byte)
    DMA_TCD0_DOFF = 0;
    // Restore destination address after major loop
    DMA_TCD0_DLASTSGA = -sizeof(samples);
    // Source and destination size 16 bit
    DMA_TCD0_ATTR = DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_DSIZE(1);
    // Number of bytes to transfer (in each service request)
    DMA_TCD0_NBYTES_MLNO = 2;
    // Set loop counts
    DMA_TCD0_CITER_ELINKNO = sizeof(samples) / 2;
    DMA_TCD0_BITER_ELINKNO = sizeof(samples) / 2;
    // Enable interrupt (end-of-major loop)
    DMA_TCD0_CSR = DMA_TCD_CSR_INTMAJOR;

    // Set ADC as source (CH 1), enable DMA MUX
    DMAMUX0_CHCFG0 = DMAMUX_DISABLE;
    DMAMUX0_CHCFG0 = DMAMUX_SOURCE_ADC0 | DMAMUX_ENABLE;

    // Enable request input signal for channel 1
    DMA_SERQ = 0;


    // Enable interrupt request
    NVIC_ENABLE_IRQ(IRQ_DMA_CH0);
}

void dacInit(void){

}

void pgaInit(void){
    // Run PGA in normal power mode and enable PGA
    ADC0_PGA |= ADC0_PGA_PGALPB | ADC0_PGA_PGAEN;
}

void cmpInit(void){
    // Enable compare function, set greater than and enable range function
    ADC0_SC2 |= ADC_SC2_ACFE | ADC_SC2_ACFGT | ADC_SC2_ACREN;
}

void setCmp(uint16_t cmp){
    
}


// Remember DAC gain!
void setGain(uint8_t gain){
    /* PGA gain = 2^(uint8_t gain) */
    if(gain <= 6)
        ADC0_PGA |= ADC_PGA_PGAG(gain);
}

void dma_ch0_isr() {
    // Clear interrupt request for channel 1
    if(bufferFlag == 0){
        DMA_TCD0_DADDR = samples2;
        DMA_TCD0_DLASTSGA = -sizeof(samples2);
        bufferFlag = 1;
    }else{
        DMA_TCD0_DADDR = samples;
        DMA_TCD0_DLASTSGA = -sizeof(samples);
        bufferFlag = 0;
    }
    DMA_CINT = 0;
}

int main(void){
    pinMode(13, OUTPUT);
    pinMode(15, OUTPUT);

    // Set busclock = system_clock/2 = 36 MHz
    SIM_CLKDIV1 |= SIM_CLKDIV1_OUTDIV2(1);

    pgaInit();
    adcInit();
    cmpInit();
    pdbInit();
    dmaInit();
    dacInit();

    //rfInit();

    while(1){

    }
}
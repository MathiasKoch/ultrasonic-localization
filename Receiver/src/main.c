#include <avr/io.h>
#define ARM_MATH_CM4
#include <arm_math.h>
//#include <common.h>
#include <util/delay.h>
#include <usb_serial.h>
#include <stdio.h>
#include "nrf24.h"
#include "xprintf.h"
#include "frequency_calc.h"


#define PDB_CH0C1_TOS 0x0100
#define PDB_CH0C1_EN 0x01
#define PDB_SC_PDBIF_MASK 0x40u
#define TIE 0x2
#define TEN 0x1
#define uint16_MAX 65535

//#define DEBUG_RF
//#define DEBUG_FFT
//#define DEBUG_SAMPLE

#define BUFSIZE 512

uint8_t tx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
uint8_t rx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
volatile uint16_t clk;

extern q15_t buffer3[BUFSIZE]; 
static q15_t buffer1[BUFSIZE];
static q15_t h[BUFSIZE/2];
static q15_t samples[BUFSIZE/2];
static q15_t samples2[BUFSIZE/2];
uint32_t elapsedTime;
int bufferFlag = 0;
volatile uint16_t clk_transmit = 0;
uint8_t data_out[3];


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

    ADC0_SC1A = ADC_SC1_AIEN | channel2sc1a[3];
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

#define PDB_PERIOD ((F_BUS / 2 / 200000)-1)     // Samplerate

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

    // Set ADC as source (CH 0), enable DMA MUX
    DMAMUX0_CHCFG0 = DMAMUX_DISABLE;
    DMAMUX0_CHCFG0 = DMAMUX_SOURCE_ADC0 | DMAMUX_ENABLE;

    // Enable request input signal for channel 0
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

void enableCmp(uint8_t enable){
    if(enable == 0x0)
        ADC0_SC2 &= ~ADC_SC2_ACFE;
    else
        ADC0_SC2 |= ADC_SC2_ACFE;
}

void setCmp(uint16_t cmp){
    ADC0_CV1 = 2048 + cmp;
    ADC0_CV2 = 2048 - cmp;
}

void cmpInit(void){
    // Enable compare function, set greater than and enable range function
    ADC0_SC2 |= ADC_SC2_ACFGT | ADC_SC2_ACREN;
    setCmp(256);
    enableCmp(0x1);
}

// Remember DAC gain!
void setGain(uint8_t gain){
    /* PGA gain = 2^(uint8_t gain) */
    if(gain <= 6)
        ADC0_PGA |= ADC0_PGA_PGAG(gain);
}

void pdb_isr(void){
    elapsedTime += 1;
    PDB0_SC &= ~PDB_SC_PDBIF_MASK;  // clear interrupt mask
    //cycle_flags = 0;
    return;
}

void dma_ch0_isr(void){
    // Clear interrupt request for channel 0
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
    return;
}

void pit0_isr(){
    PIT_TFLG0 = 1;
    if(clk++ == uint16_MAX){
        clk = 0;
    }
    return;
}

#define CLOCK_PERIOD ((F_BUS / 1000000)-1)    // 1 Mhz = 1 us

void clock_init(void){
    SIM_SCGC6 |= SIM_SCGC6_PIT;
    PIT_MCR = 0x00;
    NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
    PIT_LDVAL0 = CLOCK_PERIOD;
    PIT_TCTRL0 = TIE;
    PIT_TCTRL0 |= TEN;
    PIT_TFLG0 |= 1;
}

void hilbert_init(void){ // Q2.14 format
    uint16_t i;
    uint16_t n = BUFSIZE/2;
    arm_fill_q15(0, h, n);
    h[0] = 16384;
    if(n > 0 && 2*floor(n/2) == n){
        h[n/2] = 16384;
        for(i = 1; i < n/2; i++)
            h[i] = 32767;
    }else if(n>0){
        for(i = 1; i < (n+1)/2; i++)
            h[i] = 32767;
    }
}

void sync_clock(){
    NVIC_DISABLE_IRQ(IRQ_PIT_CH0);
    clk_transmit = clk;
    NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
    data_out[0] = 'c';
    data_out[1] = (clk_transmit >> 8) & 0xFF;
    data_out[2] = clk_transmit & 0xFF;
    nrf24_send(data_out);        
    while(nrf24_isSending());

    uint8_t temp = nrf24_lastMessageStatus();
    uint8_t retrans_cnt = nrf24_retransmissionCount();
    if(temp == NRF24_TRANSMISSON_OK){ 
        
        clk_transmit += retrans_cnt * 251;
        #ifdef DEBUG_RF
            xprintf("Receiver >> Contacted beacon successfully in %u retransmits\r\n", retrans_cnt);
        #endif
        

    }else if(temp == NRF24_MESSAGE_LOST){
        #ifdef DEBUG_RF
            xprintf("Receiver >>   !!!   Contact to beacon lost with %u retransmits\r\n", retrans_cnt);
        #endif
    }
    nrf24_powerUpRx();
}



int main(){
    SIM_CLKDIV1 |= SIM_CLKDIV1_OUTDIV2(1);

    //pgaInit();
    adcInit();
    cmpInit();
    pdbInit();
    dmaInit();
    //dacInit();


    /* Init timers and RF */
    nrf24_init();
    clock_init();
    
    nrf24_config(1,3);
 
    /* Set the device addresses */
    nrf24_tx_address(tx_address);
    nrf24_rx_address(rx_address);
    
    xdev_out(usb_serial_putchar);
    #ifdef DEBUG_RF
        /* Init the xprintf library */
        xprintf("\r\n> Device setup to receive\r\n");
    #endif


    arm_cfft_radix4_instance_q15 S; 
    arm_cfft_radix4_instance_q15 Si; 
    arm_cfft_radix4_init_q15(&S, 1024, 0, 1); 
    arm_cfft_radix4_init_q15(&Si, 1024, 1, 1); 
    hilbert_init();


    uint8_t data_in[3];
    

    uint16_t delay_time = 0;
    int old_bufferFlag = bufferFlag;
    q15_t frequencies[BUFSIZE/2];
    while(1){    


        if(bufferFlag != old_bufferFlag){
            uint16_t o = 0;
            // Interlace ADC values for complex fft
            for(o = 0; o<BUFSIZE;o+=2){
                if(bufferFlag == 0)
                    buffer1[o] = samples[o/2];
                else
                    buffer1[o] = samples2[o/2];
                buffer1[o+1] = 0;
            }
            arm_cfft_radix4_q15(&S, buffer1);                        // Input: Q1.15, Output: Q11.5
            arm_shift_q15(buffer1, 3, buffer1, BUFSIZE);
            arm_cmplx_mult_real_q15(buffer1, h, buffer1, BUFSIZE/2); // Input: Q8.8 & Q2.14, Output: Q10.22 --SAT--> Q1.15
            arm_cfft_radix4_q15(&Si, buffer1);                       // Outputs Q12.4

            

            frequency_calc(buffer1, frequencies, BUFSIZE);
            /*
                if(o > 0){
                    frequencies[o/2] = angles[o/2] - angles[(o/2)-1];
                }*/
            // Sync clock 
            sync_clock();
            // Calculate starting period of burst and save timestamp

            #ifdef DEBUG_FFT
                for(o = 0; o<BUFSIZE;o++){
                    if(o%16==0 && o>0)
                        xprintf(";\r\n");
                    xprintf("%d", buffer1[o]);
                    if(o!=BUFSIZE-1)
                        xprintf(", ");
                }
            #endif
        }

        _delay_us(300);
        if(nrf24_dataReady()){
            nrf24_getData(data_in);
            if(data_in[0] == 'c'){
                NVIC_DISABLE_IRQ(IRQ_PIT_CH0);
                delay_time = clk_transmit > clk? ((uint16_MAX + clk - clk_transmit) / 2) : ((clk - clk_transmit) / 2);
                clk = (((uint16_t)data_in[1] << 8) | data_in[2]) + delay_time;
                NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
            }else if(data_in[0] == 'r'){
                NVIC_DISABLE_IRQ(IRQ_PIT_CH0);
                clk -= data_in[1] * 125;
                NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
                #ifdef DEBUG_RF
                    xprintf("Receiver >> Clock synced with %u [us] delay time based on %u retransmits\r\n\r\n", delay_time - data_in[1] * 125, data_in[1]);
                #endif

                // Calculate time of flight and distance here!


            }                      
        }
        old_bufferFlag = bufferFlag;
    }
}
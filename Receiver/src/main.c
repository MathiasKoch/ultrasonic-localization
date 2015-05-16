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
#define DEBUG_FFT
//#define DEBUG_SAMPLE

#define BUFSIZE 2048
#define NUMTAPS 16

uint8_t tx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
uint8_t rx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
volatile uint16_t clk;

extern q15_t buffer3[BUFSIZE]; 
static q15_t buffer1[BUFSIZE/2];
static q15_t h[BUFSIZE/2];
static q15_t samples[BUFSIZE/2];
static q15_t samples2[BUFSIZE/2];
static q15_t fir_state[BUFSIZE/2 + NUMTAPS];
const q15_t fir_coeffs[NUMTAPS] = {112,243,618,1293,2217,3225,4089,4587,4587,4089,3225,2217,1293,618,243,112};
static volatile int bufferFlag = 0;
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
    //ADC0_SC3 = ADC_SC3_AVGE | ADC_SC3_AVGS(0);
    ADC0_SC3 |= ADC_SC3_ADCO;

    ADC0_SC1A = ADC_SC1_AIEN  | channel2sc1a[7];     // Input on A3
    //NVIC_ENABLE_IRQ(IRQ_ADC0);
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

#define PDB_PERIOD ((F_BUS / 400000)-1)     // Samplerate

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
    //NVIC_ENABLE_IRQ(IRQ_PDB);
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
    DMA_TCD0_DOFF = 2;
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

// Remember DAC gain!
void setGain(uint16_t gain){

    *(int16_t *)&(DAC0_DAT0L) = gain;
    /* PGA gain = 2^(uint8_t gain) */
    /*if(gain <= 6)
        ADC0_PGA |= ADC0_PGA_PGAG(gain);*/
}

void dacInit(void){
    SIM_SCGC2 |= SIM_SCGC2_DAC0; // enable DAC clock
    DAC0_C0 = DAC_C0_DACEN | DAC_C0_DACRFS;
    setGain(700);
}

void pgaInit(void){
    // Run PGA in normal power mode and enable PGA
    ADC0_PGA |= ADC0_PGA_PGALPB | ADC0_PGA_PGAEN;
}

void enableCmp(uint8_t enable){
    //if(enable == 0x0)
    //    ADC0_SC2 &= ~ADC_SC2_ACFE;
    //else
        ADC0_SC2 |= ADC_SC2_ACFE;
}

void setCmp(uint16_t cmp){
    ADC0_CV1 = 600;
    //ADC0_CV2 = 200;
}

void cmpInit(void){
    // Enable compare function, set greater than and enable range function
    //enableCmp(0x1);
    ADC0_SC2 |= ADC_SC2_ACFGT | ADC_SC2_ACFE;// | ADC_SC2_ACREN;
    //*(int16_t *)&(ADC0_CV1) = 600;
    ADC0_CV1 = 1000;
    //*(int16_t *)&(ADC0_CV2) = 100;
    //setCmp(256);
}

void pdb_isr(void){
    PDB0_SC &= ~PDB_SC_PDBIF_MASK;  // clear interrupt mask
    //cycle_flags = 0;
    return;
}

void dma_ch0_isr(void){
    // Clear interrupt request for channel 0
    /*if(bufferFlag == 0){
        DMA_TCD0_DADDR = samples2;
        DMA_TCD0_DLASTSGA = -sizeof(samples2);
        bufferFlag = 1;
    }else{
        DMA_TCD0_DADDR = samples;
        DMA_TCD0_DLASTSGA = -sizeof(samples);
        bufferFlag = 0;
    }*/


    bufferFlag = 1;
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

    SIM_SCGC5 |= SIM_SCGC5_PORTD;
    PORTD_PCR0 |= PORT_PCR_MUX(1);
    GPIOD_PDDR |= 1<<0; 

    //pgaInit();
    cmpInit();
    adcInit();
    pdbInit();
    dmaInit();
    dacInit();

    clock_init();

    /* Init timers and RF */
    nrf24_init();
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
    arm_fir_instance_q15 fir;
    arm_cfft_radix4_init_q15(&S, 1024, 0, 1); 
    arm_cfft_radix4_init_q15(&Si, 1024, 1, 1);
    arm_fir_init_q15(&fir, NUMTAPS, (q15_t *) &fir_coeffs[0], &fir_state[0], BUFSIZE/2);
    hilbert_init();


    uint8_t data_in[3];
    
    uint32_t i = 0;
    uint16_t delay_time = 0;
    q15_t angles[BUFSIZE/2];
    q15_t angles_filtered[BUFSIZE/2];
    
    q15_t frequencies[BUFSIZE/8];
    q15_t frequencies_finished[(BUFSIZE/8)-1];
    while(1){    

        if(ADC0_SC1A & ADC_SC1_COCO){
            GPIOD_PDOR ^= 1<<0;
            //ADC0_RA;
        }
        //if(){
        /*if(i++%900000 == 0){
            uint16_t o;
            bufferFlag = 0;
            /*for(o = 0; o<BUFSIZE;o+=2){
                buffer1[o] = samples[o/2] << 5;
                buffer1[o+1] = 0;
            }*/
            // Interlace ADC values for complex fft
            /*for(o = 0; o<BUFSIZE;o+=2){
                if(bufferFlag == 0)
                    buffer1[o] = samples[o/2] << 5;
                else
                    buffer1[o] = samples2[o/2] << 5;
                buffer1[o+1] = 0;
            }
            for(o = 0; o<BUFSIZE;o++){
                buffer1[o] = buffer3[o];    
            }*/
               
           /* arm_cfft_radix4_q15(&S, buffer1);                        // Input: Q1.15, Output: Q11.5
            arm_shift_q15(buffer1, 3, buffer1, BUFSIZE);
            arm_cmplx_mult_real_q15(buffer1, h, buffer1, BUFSIZE/2); // Input: Q8.8 & Q2.14, Output: Q10.22 --SAT--> Q1.15
            arm_cfft_radix4_q15(&Si, buffer1);                       // Input: Q1.15, Output: Q10.6



            arm_shift_q15(buffer1, 7, buffer1, BUFSIZE);
            atan2_fp(buffer1, angles, BUFSIZE);                      // Input: Q3.13, Output: Q??.??
            arm_shift_q15(angles, -6, angles, BUFSIZE/2);
            unwrap_fp(angles, angles_filtered, BUFSIZE/2);

            arm_fir_q15(&fir, (q15_t *) &angles_filtered[0], (q15_t *) &angles_filtered[0], BUFSIZE/2);
            /*
            **  Optimize these two loops by, eg. combining them to a single loop with no in-between buffer.
            */
           /* for(o = 0; o < ((BUFSIZE/2)-3); o+=4){
                frequencies[o/4] = angles_filtered[o] + angles_filtered[o+1] + angles_filtered[o+2] + angles_filtered[o+3];
            }

            for(o = 1; o < BUFSIZE/8; o++){
                frequencies_finished[o-1] = frequencies[o] - frequencies[o-1];
            }
            uint16_t index = change_detect(frequencies_finished, BUFSIZE/8);

            // Sync clock 
            sync_clock();
            // Calculate starting period of burst and save timestamp

            #ifdef DEBUG_FFT
                for(o = 0; o<BUFSIZE;o++){
                    xprintf("%d,", buffer1[o]);
                }
                xprintf("#");
//                xprintf("#%d",index);
            #endif
        }*/
        //delay(10);
        //_delay_us(300);
        /*if(nrf24_dataReady()){
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
        }*/

    }
}
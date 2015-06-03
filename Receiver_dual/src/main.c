
#include "mk20dx128.h"
#define ARM_MATH_CM4
#include <arm_math.h>
//#include <common.h>
//#include <stdio.h>
#include "nrf24.h"
#include "frequency_calc.h"
#include "adc.h"
#include "sync.h"

#define uint16_MAX 65535
#define FS 600000
#define BUFSIZE 1000
#define NUMTAPS 16

//#define DEBUG_RF
//#define DEBUG_FFT
#define DEBUG_SAMPLE

#if defined(DEBUG_RF) || defined(DEBUG_FFT) || defined(DEBUG_SAMPLE)
    #define DEBUG
    #include <usb_serial.h>
    #include "xprintf.h"
#endif


uint8_t tx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
uint8_t rx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};


extern q15_t buffer3[2000];
extern q15_t w1_cs[2000];
extern q15_t w2_cs[2000];
//static q15_t buffer1[BUFSIZE/2];
static q15_t samples[BUFSIZE];
//static q15_t samples2[BUFSIZE/2];
static volatile int bufferFlag = 0;


void dmaInit() {
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

void setGain(uint16_t gain){
    *(int16_t *)&(DAC0_DAT0L) = gain;
    /* PGA gain = 2^(uint8_t gain) */
    /*if(gain <= 6)
        ADC0_PGA |= ADC0_PGA_PGAG(gain);*/
}

void dacInit(void){
    SIM_SCGC2 |= SIM_SCGC2_DAC0; // enable DAC clock
    DAC0_C0 = DAC_C0_DACEN | DAC_C0_DACRFS;
    setGain(500);
}

void dma_ch0_isr(void){
    DMA_CINT = 0;
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
}

void handleRF(){
    uint8_t data_in[RF_PACKET_SIZE];
    receivedRF = 0;

    if(nrf24_dataReady()){
        nrf24_getData(data_in);
        switch(data_in[0]){
            case 'f':
                if(sync.mode == SYNC_MODE_MASTER)
                    enableFastSync();
                break;
            case 'r':
                if(sync.mode == SYNC_MODE_SLAVE)
                    //resetTimeSync();
                break;
            case 's':
                if(sync.mode == SYNC_MODE_SLAVE)
                    calcTimeSync(data_in);
                break;
        }
    }
}

int main(){
    SIM_CLKDIV1 |= SIM_CLKDIV1_OUTDIV2(1);

    SIM_SCGC5 |= SIM_SCGC5_PORTB;
    PORTB_PCR17 |= PORT_PCR_MUX(1);
    GPIOB_PDDR |= 1<<17;


    


    ADC_VALS adc;

    adc.bufsize = BUFSIZE;
    adc.fs = FS;
    adc.channel = 0x06;
    adc.pause_samples = 1024;
    adc.CV1 = 868;//771;
    adc.CV2 = 682;//407;

    adc_init(&adc);
    dmaInit();
    dacInit();
    sync_init(SYNC_MODE_MASTER, DMAMUX_SOURCE_PORTB);


    /* Init timers and RF */
    nrf24_init();
    nrf24_config(1,RF_PACKET_SIZE);
    /* Set the device addresses */
    nrf24_tx_address(tx_address);
    nrf24_rx_address(rx_address);
    
 
    
    #ifdef DEBUG
        xdev_out(usb_serial_putchar);
    #endif
    q15_t Qi1[2000];
    q15_t Qi2[2000];
    q15_t phase1[1000];
    q15_t phase2[1000];
    q15_t phase_diff[1000];


    uint16_t index = 0;
    
    while(1){
        //if(receivedRF == 1)
          //  handleRF();
        if(bufferFlag == 1){
            bufferFlag = 0;
            arm_cmplx_mult_real_q15(w1_cs, buffer3, Qi1, 1000);
            arm_cmplx_mult_real_q15(w2_cs, buffer3, Qi2, 1000);
            atan2_fp(Qi1, phase1, 2000);  
            atan2_fp(Qi2, phase2, 2000);  
            arm_sub_q15(phase2,phase1,phase_diff, 1000);
            if(phase_diff[500] == 0){
                index = 500;
            }else if(phase_diff[500] < 0){
                for(index = 500; index > 0; index--){
                    if(phase_diff[index] > 0)
                        break;
                }
            }else{
                for(index = 500; index < 1000; index++){
                    if(phase_diff[index] < 0)
                        break;
                }
            }
            //xprintf("%d\r\n", index);
        }
        delay(10);     
    }
}
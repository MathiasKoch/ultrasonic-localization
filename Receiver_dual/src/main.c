
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

void setGain(uint16_t gain){
    *(int16_t *)&(DAC0_DAT0L) = gain;
    /* PGA gain = 2^(uint8_t gain) */
    /*if(gain <= 6)
        ADC0_PGA |= ADC0_PGA_PGAG(gain);*/
}

void dac_init(void){
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
                enableFastSync();
                break;
            case 'r':
                //resetTimeSync();
                break;
            case 's':
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
    dma_init(samples);
    dac_init();

    PORTB_PCR0 |= PORT_PCR_MUX(1) | PORT_PCR_IRQC(2);
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
    q15_t Qi1[BUFSIZE*2];
    q15_t Qi2[BUFSIZE*2];
    q15_t phase1[BUFSIZE];
    q15_t phase2[BUFSIZE];
    q15_t phase_diff[BUFSIZE];


    uint16_t index = 0;
    
    while(1){
        //if(receivedRF == 1)
          //  handleRF();
        if(bufferFlag == 1){
            bufferFlag = 0;
            arm_cmplx_mult_real_q15(w1_cs, buffer3, Qi1, BUFSIZE);
            arm_cmplx_mult_real_q15(w2_cs, buffer3, Qi2, BUFSIZE);
            atan2_fp(Qi1, phase1, BUFSIZE*2);  
            atan2_fp(Qi2, phase2, BUFSIZE*2);  
            arm_sub_q15(phase2,phase1,phase_diff, BUFSIZE);
            if(phase_diff[BUFSIZE/2] == 0){
                index = BUFSIZE/2;
            }else if(phase_diff[BUFSIZE/2] < 0){
                for(index = BUFSIZE/2; index > 0; index--){
                    if(phase_diff[index] > 0)
                        break;
                }
            }else{
                for(index = BUFSIZE/2; index < BUFSIZE; index++){
                    if(phase_diff[index] < 0)
                        break;
                }
            }
            //xprintf("%d\r\n", index);
        }
        delay(10);     
    }
}
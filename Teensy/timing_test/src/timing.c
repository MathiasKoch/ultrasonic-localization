
#include "mk20dx128.h"
#define ARM_MATH_CM4
#include <arm_math.h>
//#include <common.h>
#include <usb_serial.h>
#include "xprintf.h"
#include "WProgram.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include "adc.h"

#define BUFSIZE 256
#define FS 800000

q15_t samples[BUFSIZE];
static volatile int16_t val;



void dmaInit() {
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
    DMA_TCD4_DADDR = samples;
    // Destination offset (2 byte)
    DMA_TCD4_DOFF = 2;
    // Restore destination address after major loop
    DMA_TCD4_DLASTSGA = -sizeof(samples);
    // Source and destination size 16 bit
    DMA_TCD4_ATTR = DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_DSIZE(1);
    // Number of bytes to transfer (in each service request)
    DMA_TCD4_NBYTES_MLNO = 2;
    // Set loop counts
    DMA_TCD4_CITER_ELINKNO = sizeof(samples) / 2;
    DMA_TCD4_BITER_ELINKNO = sizeof(samples) / 2;
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



void dma_ch4_isr(void) {
    // Clear interrupt request for channel 1
    /*if(bufferFlag == 0){
        DMA_TCD0_DADDR = samples2;
        DMA_TCD0_DLASTSGA = -sizeof(samples2);
        bufferFlag = 1;
    }else{
        DMA_TCD0_DADDR = samples;
        DMA_TCD0_DLASTSGA = -sizeof(samples);
        bufferFlag = 0;
    }*/
        xprintf("here\r\n");

    DMA_CINT = 4;
}


int main(void){
    // Set busclock = system_clock/2 = 36 MHz
    SIM_CLKDIV1 |= SIM_CLKDIV1_OUTDIV2(1);
    pinMode(2,OUTPUT);
    pinMode(14,OUTPUT);
    pinMode(A6, INPUT);
    pinMode(A14, OUTPUT);
    analogWriteResolution(12);

    ADC_VALS adc;

    adc.bufsize = BUFSIZE;
    adc.fs = FS;
    adc.channel = 0x06;
    adc.pause_samples = 4 * BUFSIZE;
    adc.CV1 = 680;
    adc.CV2 = 560;

    adc_init(&adc);
    dmaInit();

    xdev_out(usb_serial_putchar);


    float value = 0.40;
    analogWrite(A14, (int)(value*2048.0));
    int i = 0;
    while(1){
        delay(10);

       /* for(i=0;i<BUFSIZE;i+=8)
            xprintf("%d, %d, %d, %d, %d, %d, %d, %d\r\n", samples[i],samples[i+1],samples[i+2],samples[i+3],samples[i+4],samples[i+5],samples[i+6],samples[i+7]);
        xprintf("\r\n\r\n\r\n\r\n");*/
            //xprintf("%d\r\n",F_BUS);
    }
}
/*  OctoWS2811 - High Performance WS2811 LED Display Library
    http://www.pjrc.com/teensy/td_libs_OctoWS2811.html
    Copyright (c) 2013 Paul Stoffregen, PJRC.COM, LLC

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "neopixel.h"


uint16_t neo_stripLen;
void * neo_frameBuffer;
void * neo_drawBuffer;
#define WS2811_TIMING_T0H  60
#define WS2811_TIMING_T1H  176

static const uint8_t ones = 1<<5;
static volatile uint8_t update_in_progress = 0;
static uint32_t update_completed_at = 0;

uint32_t neo_micros(){
	uint32_t count, current, istatus;

	__disable_irq();
	current = SYST_CVR;
	count = systick_millis_count;
	istatus = SCB_ICSR;	// bit 26 indicates if systick exception pending
	__enable_irq();
	 //systick_current = current;
	 //systick_count = count;
	 //systick_istatus = istatus & SCB_ICSR_PENDSTSET ? 1 : 0;
	if ((istatus & SCB_ICSR_PENDSTSET) && current > 50) count++;
	current = ((F_CPU / 1000) - 1) - current;
	return count * 1000 + current / (F_CPU / 1000000);
}


void neo_init(uint32_t numPerStrip, void *frameBuf, void *drawBuf)
{
	neo_stripLen = numPerStrip;
	neo_frameBuffer = frameBuf;
	neo_drawBuffer = drawBuf;

	uint32_t bufsize, frequency;

	bufsize = neo_stripLen*24;

	// set up the buffers
	memset(neo_frameBuffer, 0, bufsize);
	if (neo_drawBuffer) {
		memset(neo_drawBuffer, 0, bufsize);
	} else {
		neo_drawBuffer = neo_frameBuffer;
	}
	
	// configure the 8 output pins
	GPIOD_PCOR |= 1<<5;
	PORTD_PCR5 |= PORT_PCR_MUX(1);
    GPIOD_PDDR |= 1<<5;


	// create the two waveforms for WS2811 low and high bits
	frequency = 800000;
	analogWriteResolution(8);
	analogWriteFrequency(3, frequency);																// FIX THESE
	analogWriteFrequency(4, frequency);
	analogWrite(3, WS2811_TIMING_T0H);
	analogWrite(4, WS2811_TIMING_T1H);

	// pin 16 triggers DMA(port B) on rising edge (configure for pin 3's waveform)
	CORE_PIN16_CONFIG = PORT_PCR_IRQC(1)|PORT_PCR_MUX(3);

	// pin 15 triggers DMA(port C) on falling edge of low duty waveform
	// pin 15 and 16 must be connected by the user: 16 is output, 15 is input
	GPIOC_PDDR &= ~(0<<0); 	// Pin 15 as input
	CORE_PIN15_CONFIG = PORT_PCR_IRQC(2)|PORT_PCR_MUX(1);

	// pin 4 triggers DMA(port A) on falling edge of high duty waveform
	CORE_PIN4_CONFIG = PORT_PCR_IRQC(2)|PORT_PCR_MUX(3);


	// Enable DMA, DMAMUX clocks
    SIM_SCGC7 |= SIM_SCGC7_DMA;
    SIM_SCGC6 |= SIM_SCGC6_DMAMUX;


	// DMA channel #1 sets WS2811 high at the beginning of each cycle
    DMA_TCD1_SADDR = &ones;
    DMA_TCD1_SOFF = 0;
    DMA_TCD1_ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);
    DMA_TCD1_NBYTES_MLNO = 1;
    DMA_TCD1_SLAST = 0;
    DMA_TCD1_DADDR = &GPIOD_PSOR;
    DMA_TCD1_DOFF = 0;
    DMA_TCD1_CITER_ELINKNO = bufsize;
    DMA_TCD1_DLASTSGA = 0;
    DMA_TCD1_CSR = DMA_TCD_CSR_DREQ;
    DMA_TCD1_BITER_ELINKNO = bufsize;






	// DMA channel #2 writes the pixel data at 20% of the cycle
	DMA_TCD2_SADDR = neo_frameBuffer;
    DMA_TCD2_SOFF = 1;
    DMA_TCD2_ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);
    DMA_TCD2_NBYTES_MLNO = 1;
    DMA_TCD2_SLAST = -bufsize;
    DMA_TCD2_DADDR = &GPIOD_PDOR;
    DMA_TCD2_DOFF = 0;
    DMA_TCD2_CITER_ELINKNO = bufsize;
    DMA_TCD2_DLASTSGA = 0;
    DMA_TCD2_CSR = DMA_TCD_CSR_DREQ;
    DMA_TCD2_BITER_ELINKNO = bufsize;


	// DMA channel #3 clear all the pins low at 48% of the cycle
	DMA_TCD3_SADDR = &ones;
    DMA_TCD3_SOFF = 0;
    DMA_TCD3_ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);
    DMA_TCD3_NBYTES_MLNO = 1;
    DMA_TCD3_SLAST = 0;
    DMA_TCD3_DADDR = &GPIOD_PCOR;
    DMA_TCD3_DOFF = 0;
    DMA_TCD3_CITER_ELINKNO = bufsize;
    DMA_TCD3_DLASTSGA = 0;
    DMA_TCD3_CSR = DMA_TCD_CSR_DREQ | DMA_TCD_CSR_INTMAJOR;
    DMA_TCD3_BITER_ELINKNO = bufsize;

#ifdef __MK20DX256__
	MCM_CR = MCM_CR_SRAMLAP(1) | MCM_CR_SRAMUAP(0);													// CHECK THIS
	AXBS_PRS0 = 0x1032;
#endif

	// route the edge detect interrupts to trigger the 3 channels

	DMAMUX0_CHCFG1 = DMAMUX_DISABLE;
    DMAMUX0_CHCFG1 = DMAMUX_SOURCE_PORTB | DMAMUX_ENABLE;
    DMAMUX0_CHCFG2 = DMAMUX_DISABLE;
    DMAMUX0_CHCFG2 = DMAMUX_SOURCE_PORTC | DMAMUX_ENABLE;
    DMAMUX0_CHCFG3 = DMAMUX_DISABLE;
    DMAMUX0_CHCFG3 = DMAMUX_SOURCE_PORTA | DMAMUX_ENABLE;

    // Enable interrupt request
    NVIC_ENABLE_IRQ(IRQ_DMA_CH3);
}

void dma_ch3_isr(void){
	DMA_CINT = 3;
	update_completed_at = neo_micros();		 															
	update_in_progress = 0;
}

int neo_busy(void)
{
	//if (DMA_ERQ & 0xE) return 1;
	if (update_in_progress) return 1;
	// busy for 50 us after the done interrupt, for WS2811 reset
	if (neo_micros() - update_completed_at < 50) return 1;												
	return 0;
}

void neo_show(void)
{
	uint32_t cv, sc;

	// wait for any prior DMA operation
	while (update_in_progress) ; 
	// it's ok to copy the drawing buffer to the frame buffer
	// during the 50us WS2811 reset time
	if (neo_drawBuffer != neo_frameBuffer) {
		// TODO: this could be faster with DMA, especially if the
		// buffers are 32 bit aligned... but does it matter?
		memcpy(neo_frameBuffer, neo_drawBuffer, neo_stripLen * 24);
	}
	// wait for WS2811 reset
	while (neo_micros() - update_completed_at < 50) ;													

	// ok to start, but we must be very careful to begin
	// without any prior 3 x 800kHz DMA requests pending
	sc = FTM1_SC;
	cv = FTM1_C1V;
	NVIC_DISABLE_IRQ(IRQ_DMA_CH3);																			
	// CAUTION: this code is timing critical.  Any editing should be
	// tested by verifying the oscilloscope trigger pulse at the end
	// always occurs while both waveforms are still low.  Simply
	// counting CPU cycles does not take into account other complex
	// factors, like flash cache misses and bus arbitration from USB
	// or other DMA.  Testing should be done with the oscilloscope
	// display set at infinite persistence and a variety of other I/O
	// performed to create realistic bus usage.  Even then, you really
	// should not mess with this timing critical code!
	update_in_progress = 1;
	while (FTM1_CNT <= cv) ; 
	while (FTM1_CNT > cv) ; // wait for beginning of an 800 kHz cycle
	while (FTM1_CNT < cv) ;
	FTM1_SC = sc & 0xE7;	// stop FTM1 timer (hopefully before it rolls over)
	//digitalWriteFast(1, HIGH); // oscilloscope trigger
	PORTB_ISFR = (1<<0);    // clear any prior rising edge
	PORTC_ISFR = (1<<0);	// clear any prior low duty falling edge
	PORTA_ISFR = (1<<13);	// clear any prior high duty falling edge
	DMA_SERQ = 0x01;																					
	DMA_SERQ = 0x02;		// enable all 3 DMA channels
	DMA_SERQ = 0x03;
	FTM1_SC = sc;		// restart FTM1 timer
	//digitalWriteFast(1, LOW);
	NVIC_ENABLE_IRQ(IRQ_DMA_CH3);																				
}

void neo_setPixel(uint32_t num, int color)
{
	uint32_t strip, offset, mask;
	uint8_t bit, *p;
	

	color = ((color<<8)&0xFF0000) | ((color>>8)&0x00FF00) | (color&0x0000FF);

	strip = 5;  // Pin 20 on Teensy
	offset = num;
	bit = (1<<strip);
	p = ((uint8_t *)neo_drawBuffer) + offset * 24;
	for (mask = (1<<23) ; mask ; mask >>= 1) {														
		if (color & mask) {
			*p++ |= bit;
		} else {
			*p++ &= ~bit;
		}
	}
}

int neo_getPixel(uint32_t num)
{
	uint32_t strip, offset, mask;
	uint8_t bit, *p;
	int color=0;

	strip = 5;
	offset = num;
	bit = (1<<strip);
	p = ((uint8_t *)neo_drawBuffer) + offset * 24;
	for (mask = (1<<23) ; mask ; mask >>= 1) {
		if (*p++ & bit) color |= mask;
	}
	color = ((color<<8)&0xFF0000) | ((color>>8)&0x00FF00) | (color&0x0000FF);
	return color;
}



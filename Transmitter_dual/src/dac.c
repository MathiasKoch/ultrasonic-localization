#include "dac.h"
#include "xprintf.h"

#define FS_DAC 512000

void dac_init(){
    xprintf("Initializing DAC module\t\t - \t");

	dac_load_data();
    spi_setup_master();
    //switch_init();
    spi_write_uint16(DAC_DAISY_DISABLE);      
	init_sample_timer(FS_DAC, PIT_USER_DAC);
	pit_run(0);
    spi_dma_init_tx();
    xprintf("Done\r\n");
}

void dac_enable(){
	init_sample_timer(FS_DAC, PIT_USER_DAC);
}

void dac_load_data(){
	uint16_t i;
	for(i = 0; i < DAC_BUF_SIZE; i++)
		dac_signal[i] = (sync_pattern[i] | DAC_UPDATE | SPI_PUSHR_PCS(0x08) | SPI_PUSHR_CTAS(0)); // PCS3 and end of queue	
}

void dac_start(){
    pit_run(1);
}



void switch_init(){
	if ((SIM_SCGC5 & SIM_SCGC5_PORTD) == 0) 
		SIM_SCGC5 |= SIM_SCGC5_PORTD;
	if ((SIM_SCGC5 & SIM_SCGC5_PORTB) == 0) 
		SIM_SCGC5 |= SIM_SCGC5_PORTB;
    PORTD_PCR7 |= PORT_PCR_MUX(1);		// Enable 1
    PORTD_PCR4 |= PORT_PCR_MUX(1);		// Enable 2
    PORTD_PCR0 |= PORT_PCR_MUX(1);		// Enable 3
    PORTB_PCR17 |= PORT_PCR_MUX(1);		// Enable 4
    switch_set(SWITCH_ALL, 0);
}

void switch_set(uint8_t sw, uint8_t enable){
	// TODO: Exclude by dip switches!
	if(sw == SWITCH_1)
		if(enable == 1)
			GPIOD_PDOR |= enable<<7;
		else
			GPIOD_PDOR &= ~(enable<<7);
	else if(sw == SWITCH_2)
		if(enable == 1)
			GPIOD_PDOR |= enable<<4;
		else
			GPIOD_PDOR &= ~(enable<<4);
	else if(sw == SWITCH_3)
		if(enable == 1)
			GPIOD_PDOR |= enable<<0;
		else
			GPIOD_PDOR &= ~(enable<<0);
	else if(sw == SWITCH_4)
		if(enable == 1)
			GPIOB_PDOR |= enable<<17;
		else
			GPIOB_PDOR &= ~(enable<<17);
	else if(sw == SWITCH_ALL){
		if(enable == 1){
			GPIOD_PDOR |= enable<<7;
			GPIOD_PDOR |= enable<<4;
			GPIOD_PDOR |= enable<<0;
			GPIOB_PDOR |= enable<<17;
		}else{
			GPIOD_PDOR &= ~(enable<<7);
			GPIOD_PDOR &= ~(enable<<4);
			GPIOD_PDOR &= ~(enable<<0);
			GPIOB_PDOR &= ~(enable<<17);
		}
	}
}
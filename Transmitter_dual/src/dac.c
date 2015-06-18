#include "dac.h"
#include "xprintf.h"

#define FS 10000

void dac_init(){
	dac_load_data();
    spi_setup_master();
    switch_init();
    //init_sample_timer(FS, 0);
    spi_write_uint16(DAC_DAISY_DISABLE, 1);      
    


}

void dac_load_data(){
	uint16_t i;
	for(i = 0; i < DAC_BUF_SIZE; i++)
		dac_signal[i] = (sync_pattern[i] | DAC_UPDATE) | SPI_PUSHR_PCS(0x08) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_EOQ; // PCS3 and end of queue
	//dac_signal[DAC_BUF_SIZE-1] = sync_pattern[DAC_BUF_SIZE-1] | DAC_UPDATE | SPI_PUSHR_PCS(0x08) | SPI_PUSHR_EOQ | SPI_PUSHR_CTAS(0); // PCS3 and end of queue
	
}

void dac_start(){

	xprintf("DAC started \r\n");

    spi_dma_init_tx(dac_signal);
	
	


	SPI0_MCR &= ~SPI_MCR_HALT;
    while( !(SPI0_SR & SPI_SR_EOQF))
    {}
    SPI0_SR |=  SPI_SR_EOQF | SPI_SR_TCF ;
    SPI0_MCR |= SPI_MCR_HALT;
    //DMA_CERQ = 0;
	
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
#include <avr/io.h>
#include "dac.h"
#include "spi.h"


void dac_init() 
{
    spi_setup_master();
    spi_dac_csn(HIGH);  
    spi_start();  
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
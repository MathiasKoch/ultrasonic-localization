#include "gpio.h"
#include "xprintf.h"

void gpio_init(){
    if((SIM_SCGC5 & SIM_SCGC5_PORTC) == 0)
	   SIM_SCGC5 |= SIM_SCGC5_PORTC;
    if((SIM_SCGC5 & SIM_SCGC5_PORTB) == 0)
       SIM_SCGC5 |= SIM_SCGC5_PORTB;
    if((SIM_SCGC5 & SIM_SCGC5_PORTE) == 0)
       SIM_SCGC5 |= SIM_SCGC5_PORTE;
    NVIC_ENABLE_IRQ(IRQ_PORTB);
    NVIC_ENABLE_IRQ(IRQ_PORTC);
    NVIC_ENABLE_IRQ(IRQ_PORTE);
    GPIOB_PDDR &= ~(1<<18);
    GPIOE_PDDR &= ~(1<<0);
    GPIOC_PDDR &= ~(1<<8);
    GPIOB_PDDR &= ~(1<<1);
    PORTB_PCR18 = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x0b);    // DIP 3
    PORTE_PCR0 = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x0b);    // DIP 2
    PORTC_PCR8 = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x0b);    // DIP 1
    PORTB_PCR1 = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x09);    // Calibrate
}
void portb_isr(void){
    if(GPIOB_PDIR & 1<<1){
        startCalibrate();
        delay(200);
        PORTB_PCR1 |= 1<<24;
    }else{
        delay(400);
        PORTB_PCR18 |= 1<<24;
        if((GPIOB_PDIR & 1<<18))
            xprintf("Dip 3 changed to HIGH\r\n");
        else
            xprintf("Dip 3 changed to LOW\r\n");
    }
    
}

void portc_isr(void){
    delay(200);
    PORTC_PCR8 |= 1<<24;
    if((GPIOC_PDIR & 1<<8))
        xprintf("Dip 1 changed to HIGH\r\n");
    else
        xprintf("Dip 1 changed to LOW\r\n");
}

void porte_isr(void){
    delay(200);
    PORTE_PCR0 |= 1<<24;
    if((GPIOE_PDIR & 1<<0))
        xprintf("Dip 2 changed to HIGH\r\n");
    else
        xprintf("Dip 2 changed to LOW\r\n");
    
}
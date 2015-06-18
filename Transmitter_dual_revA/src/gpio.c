#include "gpio.h"
#include "xprintf.h"

void gpio_init(){
    if((SIM_SCGC5 & SIM_SCGC5_PORTC) == 0)
	   SIM_SCGC5 |= SIM_SCGC5_PORTC;
    if((SIM_SCGC5 & SIM_SCGC5_PORTB) == 0)
       SIM_SCGC5 |= SIM_SCGC5_PORTB;
    if((SIM_SCGC5 & SIM_SCGC5_PORTE) == 0)
       SIM_SCGC5 |= SIM_SCGC5_PORTE;
    PORTB_PCR18 |= PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x0b);    // DIP 3
    PORTE_PCR0 |= PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x0b);    // DIP 2
    PORTC_PCR8 |= PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x0b);    // DIP 1
    PORTB_PCR1 |= PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x09);    // Calibrate
    NVIC_ENABLE_IRQ(IRQ_PORTB);
    NVIC_ENABLE_IRQ(IRQ_PORTC);
    NVIC_ENABLE_IRQ(IRQ_PORTE);

    dip1 = (GPIOC_PDIR & 1<<8);
    dip2 = (GPIOE_PDIR & 1<<0);
    dip3 = (GPIOB_PDIR & 1<<18);
}
void portb_isr(void){
    if(GPIOB_PDIR & 1<<1){
        PORTB_PCR1 |= 1<<24;
        startCalibrate();
    }
    if((GPIOB_PDIR & 1<<18) != dip3){
        PORTB_PCR18 |= 1<<24;
        dip3 = (GPIOB_PDIR & 1<<18);
        if(dip3)
            xprintf("Dip 3 changed to HIGH\r\n");
        else
            xprintf("Dip 3 changed to LOW\r\n");
    }
    delay(100);
}

void porte_isr(void){
    if((GPIOE_PDIR & 1<<0) != dip2){
        PORTE_PCR0 |= 1<<24;
        dip2 = (GPIOE_PDIR & 1<<0);
        if(dip2)
            xprintf("Dip 2 changed to HIGH\r\n");
        else
            xprintf("Dip 2 changed to LOW\r\n");
    }
    delay(100);
}

void portc_isr(void){
    if((GPIOC_PDIR & 1<<8) != dip1){
        PORTC_PCR8 |= 1<<24;
        dip1 = (GPIOC_PDIR & 1<<8);
        if(dip1)
            xprintf("Dip 1 changed to HIGH\r\n");
        else
            xprintf("Dip 1 changed to LOW\r\n");
    }
    delay(100);
}
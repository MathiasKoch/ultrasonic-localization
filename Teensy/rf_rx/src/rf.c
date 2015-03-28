#include <avr/io.h>
#define ARM_MATH_CM4
#include <arm_math.h>
//#include <common.h>
#include "nrf24.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include <usb_serial.h>
#include <stdio.h>

#include "xprintf.h"

#define TIE 0x2
#define TEN 0x1
#define uint16_MAX 65535


uint8_t temp;
uint8_t data_in[3];
uint8_t data_out[3];
uint8_t tx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
uint8_t rx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
volatile uint16_t clk;
uint16_t delay_time;
uint16_t clk_transmit;
/* ------------------------------------------------------------------------- */

void pit0_isr(){
    PIT_TFLG0 = 1;
    if(clk++ == uint16_MAX){
        clk = 0;
        GPIOD_PDOR ^= 1<<0;
    }
    return;
}

#define PIT_PERIOD ((F_BUS / 1000000)-1)    // 1 Mhz = 1 us

void pit_initialize(void){
    SIM_SCGC6 |= SIM_SCGC6_PIT;
    PIT_MCR = 0x00;
    NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
    PIT_LDVAL0 = PIT_PERIOD;
    PIT_TCTRL0 = TIE;
    PIT_TCTRL0 |= TEN;
    PIT_TFLG0 |= 1;
}



int main(){

    /* init the xprintf library */
    xdev_out(usb_serial_putchar);

    /* simple greeting message */
    xprintf("\r\n> RX device ready\r\n");

    SIM_SCGC5 |= SIM_SCGC5_PORTD;
    PORTD_PCR0 |= PORT_PCR_MUX(1);
    GPIOD_PDDR |= 1<<0; 
    PORTD_PCR1 |= PORT_PCR_MUX(1);
    GPIOD_PDDR |= 1<<1; 
    PORTD_PCR2 |= PORT_PCR_MUX(1);
    GPIOD_PDDR |= 1<<2; 
    /* init hardware pins */
    pit_initialize();

    nrf24_init();
    
    nrf24_config(1,3);
 
    nrf24_tx_address(tx_address);
    nrf24_rx_address(rx_address);
    while(1){    
        NVIC_DISABLE_IRQ(IRQ_PIT_CH0);
        clk_transmit = clk;
        NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
        data_out[0] = 'c';
        data_out[1] = (clk_transmit >> 8) & 0xFF;
        data_out[2] = clk_transmit & 0xFF;
        nrf24_send(data_out);        
        
        while(nrf24_isSending());

        temp = nrf24_lastMessageStatus();

        if(temp == NRF24_TRANSMISSON_OK){ 

            nrf24_powerUpRx();
            _delay_us(400);            
            if(nrf24_dataReady()){
                NVIC_DISABLE_IRQ(IRQ_PIT_CH0);
                delay_time = ((clk - clk_transmit) / 2);
                NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
                nrf24_getData(data_in);
                if(data_in[0] == 'c'){
                    NVIC_DISABLE_IRQ(IRQ_PIT_CH0);
                    clk = (((uint16_t)data_in[1] << 8) | data_in[2]) + delay_time;
                    NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
                    GPIOD_PDOR ^= 1<<1;

                }                      
            }

        }else if(temp == NRF24_MESSAGE_LOST){
                    GPIOD_PDOR ^= 1<<2;

        }
        _delay_ms(3);
        
    }
}

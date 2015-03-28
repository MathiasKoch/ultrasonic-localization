#include <avr/io.h>
#define ARM_MATH_CM4
#include <arm_math.h>
//#include <common.h>
#include "nrf24.h"
#include <util/delay.h>
#include <usb_serial.h>
#include <stdio.h>
#include "xprintf.h"

#define TIE 0x2
#define TEN 0x1
#define uint16_MAX 65535

uint8_t temp;
uint8_t data_in[3];
uint8_t data_out[3];
uint8_t rx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
uint8_t tx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
volatile uint16_t clk;

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


int main()
{

    /* init the xprintf library */
    xdev_out(usb_serial_putchar);

    /* simple greeting message */
    xprintf("\r\n> RX device ready\r\n");

    SIM_SCGC5 |= SIM_SCGC5_PORTD;
    PORTD_PCR0 |= PORT_PCR_MUX(1);
    GPIOD_PDDR |= 1<<0; 

    /* init hardware pins */
    nrf24_init();
    pit_initialize();
    
    /* Channel #2 , payload length: 4 */
    nrf24_config(1,3);
 
    /* Set the device addresses */
    nrf24_tx_address(tx_address);
    nrf24_rx_address(rx_address);

    while(1){    
        nrf24_powerUpRx();
        _delay_us(400);
        if(nrf24_dataReady()){
            nrf24_getData(data_in);      
            if(data_in[0] == 'c'){
                data_out[0] = 'c';
                NVIC_DISABLE_IRQ(IRQ_PIT_CH0);
                data_out[1] = (clk >> 8) & 0xFF;
                data_out[2] = clk & 0xFF;
                NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
                nrf24_send(data_out);
                while(nrf24_isSending());
                temp = nrf24_lastMessageStatus();

                if(temp == NRF24_TRANSMISSON_OK){ 

                }else if(temp == NRF24_MESSAGE_LOST){

                }
            }

        }
        _delay_ms(3);
    }
}
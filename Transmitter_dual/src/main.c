#include <avr/io.h>
#define ARM_MATH_CM4
#include <arm_math.h>
//#include <common.h>
#include <util/delay.h>
#include <usb_serial.h>
#include <stdio.h>
#include "nrf24.h"
#include "xprintf.h"
#include "sync.h"

#define MODE_BRAKE 0
#define MODE_TRANSMIT 1
#define MODE_SAMPLE_0 2
#define MODE_SAMPLE_90 3
#define MODE_SAMPLE_180 4
#define MODE_SAMPLE_270 5


uint8_t rx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
uint8_t tx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
uint8_t calibrated = 0;


//void portb_isr(){
    // Calibrate button clicked
//}


void handleRF(){
    uint8_t data_in[RF_PACKET_SIZE];
    receivedRF = 0;

    if(nrf24_dataReady()){
        nrf24_getData(data_in);
        switch(data_in[0]){
            case 's':
                calcTimeSync(data_in);
                break;
            case 'r':
                resetTimeSync();
                break;
        }
    }
}

void calibrate(){
    calibrated = 1;
}


int main(){
    SIM_CLKDIV1 |= SIM_CLKDIV1_OUTDIV2(1);

    SIM_SCGC5 |= SIM_SCGC5_PORTB;
    //PORTB_PCR1 |= PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x09);

    /* Init the xprintf library */
    xdev_out(usb_serial_putchar);


    SIM_SCGC5 |= SIM_SCGC5_PORTD;
    PORTD_PCR0 |= PORT_PCR_MUX(1) | PORT_PCR_IRQC(2);
    sync_init(SYNC_MODE_SLAVE, DMAMUX_SOURCE_PORTD);

    /* Init timers and RF */
    nrf24_init();
    
    
    nrf24_config(1,RF_PACKET_SIZE);
 
    /* Set the device addresses */
    nrf24_tx_address(tx_address);
    nrf24_rx_address(rx_address);



    while(1){
        if(receivedRF == 1)
            handleRF();    
        
        delay(10);
        if(!calibrated)
            calibrate();
    }
}
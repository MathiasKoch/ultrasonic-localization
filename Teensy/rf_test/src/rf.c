#include <avr/io.h>
#define ARM_MATH_CM4
#include <arm_math.h>
//#include <common.h>
#include "nrf24.h"
#include <util/delay.h>
#include <usb_serial.h>
#include <stdio.h>
#include "xprintf.h"


uint8_t temp;
uint8_t data_array[4];
uint8_t data_array2[4];
uint8_t rx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
uint8_t tx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
uint8_t q = 0;

int main(void){

    xdev_out(usb_serial_putchar);

    /* simple greeting message */
    xprintf("\r\n> TX device ready\r\n");
    
    /* init hardware pins */
    nrf24_init();
    
    /* Channel #1 , payload length: 4 */
    nrf24_config(1,4);

    /* Set the device addresses */
    nrf24_tx_address(tx_address);
    nrf24_rx_address(rx_address);    

    while(1)
    {                
        /* Fill the data buffer */
        data_array[0] = 0x11;
        data_array[1] = 0xAA;
        data_array[2] = 0x55;
        data_array[3] = q++;                                    

        nrf24_send(data_array);        
        
        while(nrf24_isSending());

        temp = nrf24_lastMessageStatus();

        if(temp == NRF24_TRANSMISSON_OK)
        {                    
            xprintf("> Tranmission went OK\r\n");
            xprintf("> ");
            xprintf("%2X ",data_array[0]);
            xprintf("%2X ",data_array[1]);
            xprintf("%2X ",data_array[2]);
            xprintf("%2X\r\n",data_array[3]);
        }
        else if(temp == NRF24_MESSAGE_LOST)
        {                    
            xprintf("> Message is lost ...\r\n");  
            uint8_t status = nrf24_getStatus();
            xprintf("\r\n");
        }
        
        /* Retranmission count indicates the tranmission quality */
        //temp = nrf24_retransmissionCount();
        //xprintf("> Retransmission count: %d\r\n\r\n",temp);

        /* Optionally, go back to RX mode ... */
        /*nrf24_powerUpRx();

        if(nrf24_dataReady())
        {
            nrf24_getData(data_array2);
            if(data_array == data_array2){
                xprintf("> Ping-back received with true data!\r\n\r\n");
            }else{
                xprintf("> Ping-back received with false data!\r\n\r\n");
            }
        }*/

        /* Or you might want to power down after TX */
        // nrf24_powerDown();            

        /* Wait a little ... */
        _delay_ms(100);
    }
}
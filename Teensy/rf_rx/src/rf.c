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
uint8_t q = 0;
uint8_t data_array[4];
uint8_t tx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
uint8_t rx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
/* ------------------------------------------------------------------------- */
int main()
{

    /* init the xprintf library */
    xdev_out(usb_serial_putchar);

    /* simple greeting message */
    xprintf("\r\n> RX device ready\r\n");

    /* init hardware pins */
    nrf24_init();
    
    /* Channel #2 , payload length: 4 */
    nrf24_config(1,4);
 
    /* Set the device addresses */
    nrf24_tx_address(tx_address);
    nrf24_rx_address(rx_address);

    while(1)
    {    
        if(nrf24_dataReady())
        {
            nrf24_getData(data_array);        
            xprintf("> ");
            xprintf("%2X ",data_array[0]);
            xprintf("%2X ",data_array[1]);
            xprintf("%2X ",data_array[2]);
            xprintf("%2X\r\n",data_array[3]);
        }
    }
}
/* ------------------------------------------------------------------------- */
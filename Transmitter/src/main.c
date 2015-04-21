#include <avr/io.h>
#define ARM_MATH_CM4
#include <arm_math.h>
//#include <common.h>
#include <util/delay.h>
#include <usb_serial.h>
#include <stdio.h>
#include "nrf24.h"
#include "xprintf.h"

#define PDB_CH0C1_TOS 0x0100
#define PDB_CH0C1_EN 0x01

#define TIE 0x2
#define TEN 0x1

#define uint16_MAX 65535

// pin defines
#define PWM 6
#define HBRO_E 8
#define MUX_1 17
#define MUX_2 16

#define MODE_BRAKE 0
#define MODE_TRANSMIT 1
#define MODE_SAMPLE_0 2
#define MODE_SAMPLE_90 3
#define MODE_SAMPLE_180 4
#define MODE_SAMPLE_270 5


uint8_t rx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
uint8_t tx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
volatile uint16_t clk;
uint8_t calibrated = 0;

void pit0_isr(){
    PIT_TFLG0 = 1;
    if(clk++ == uint16_MAX){
        clk = 0;
    }
    return;
}

#define CLOCK_PERIOD ((F_BUS / 1000000)-1)    // 1 Mhz = 1 us

void clock_init(void){
    SIM_SCGC6 |= SIM_SCGC6_PIT;
    PIT_MCR = 0x00;
    NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
    PIT_LDVAL0 = CLOCK_PERIOD;
    PIT_TCTRL0 = TIE;
    PIT_TCTRL0 |= TEN;
    PIT_TFLG0 |= 1;
}


void set_mode(int mode){
    switch(mode){
        case MODE_BRAKE:
            break;

        case MODE_TRANSMIT:
            break;

        case MODE_SAMPLE_0:
            break;

        case MODE_SAMPLE_90:
            break;

        case MODE_SAMPLE_180:
            break;

        case MODE_SAMPLE_270:
            break;
    }
}

void pit1_isr(){
    PIT_TFLG1 = 1;
    digitalWrite(PWM, !digitalRead(PWM));
    return;
}

#define PWM_PERIOD ((F_BUS / 40000)-1)    // 1 Mhz = 1 us

void pwm_init(void){
    SIM_SCGC6 |= SIM_SCGC6_PIT;
    PIT_MCR = 0x00;
    NVIC_ENABLE_IRQ(IRQ_PIT_CH1);
    PIT_LDVAL1 = PWM_PERIOD;
    PIT_TCTRL1 = TIE;
    PIT_TCTRL1 |= TEN;
    PIT_TFLG1 |= 1;
}


void calibrate(){
    // Disable H-bridge
    set_mode(MODE_SAMPLE_0);


}

uint8_t sync_timer(){
    uint8_t data_out[3];
    
    data_out[0] = 'c';
    NVIC_DISABLE_IRQ(IRQ_PIT_CH0);
    data_out[1] = (clk >> 8) & 0xFF;
    data_out[2] = clk & 0xFF;
    NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
    nrf24_send(data_out);
    while(nrf24_isSending());
    uint8_t temp = nrf24_lastMessageStatus();

    if(temp == NRF24_TRANSMISSON_OK){ 
        return 1;
    }else if(temp == NRF24_MESSAGE_LOST){
        return 0;
    }
    return 0;
}

void transmit(uint8_t total_periods, uint8_t shift_period){

}


int main(){
    SIM_CLKDIV1 |= SIM_CLKDIV1_OUTDIV2(1);

    /* Init the xprintf library */
    xdev_out(usb_serial_putchar);

    /* Init timers and RF */
    pwm_init();
    nrf24_init();
    clock_init();
    
    nrf24_config(1,3);
 
    /* Set the device addresses */
    nrf24_tx_address(tx_address);
    nrf24_rx_address(rx_address);

    /* Init mode to transmit mode */
    set_mode(MODE_TRANSMIT);

    xprintf("\r\n> Device setup to transmit\r\n");

    uint8_t data_in[3];
    while(1){    
        nrf24_powerUpRx();
        _delay_us(400);
        if(nrf24_dataReady()){
            nrf24_getData(data_in);   
            switch(data_in[0]){
                case 'c':
                    if(sync_timer())
                        transmit(40, 20);
                    break;

                default:
                    break;
            }   
        }

        if(!calibrated)
            calibrate();
    }
}

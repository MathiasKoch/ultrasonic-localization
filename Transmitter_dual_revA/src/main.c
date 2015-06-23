#include <avr/io.h>
#define ARM_MATH_CM4
#include <arm_math.h>
#include <usb_serial.h>
#include <stdint.h>
#include <stdio.h>
#include "nrf24.h"
#include "xprintf.h"
#include "sync.h"
#include "adc.h"
#include "frequency_calc.h"
#include "gpio.h"
#include "dac.h"
#include "neopixel.h"

#define ADD_CHAN 0xF0

volatile uint8_t registering;
uint8_t bufferFlag = 0;
uint8_t LED_mode;

DMAMEM int displayMemory[16*6];
int drawingMemory[16*6];

void lptmr_isr(){
    LPTMR0_CSR = LPTMR_TCF;
    LPTMR0_CSR &= ~LPTMR_TEN;
    if(mode == MODE_CALIBRATE_MASTER){  // Used for ultrasonic timeout
        // TODO: Timeout occured, invalid address check? and move on to next device.. special case considering passCount
        requestTransmit(calibCount);
    }else{  // Used for registering
        if(registering == 1){
            registering = 0;
            uint8_t count = 1;
            for(count = 1; count < positions.beaconCount+1; count++){
                if(positions.address[count][0] != count)
                    break;
            }

            positions.address[0][0] = count;
            positions.address[0][1] = ADD_CHAN;
            positions.address[0][2] = ADD_CHAN;
            positions.address[0][3] = ADD_CHAN;
            positions.address[0][4] = ADD_CHAN;  
            nrf24_rx_address(positions.address[0]);
        }

        uint8_t data_out[RF_PACKET_SIZE];
        nrf24_tx_address(broadCastAddress);    
        data_out[0] = 'w';
        data_out[1] = positions.address[0][0];
        data_out[2] = 't';
        nrf24_send(data_out);        
        while(nrf24_isSending());
        nrf24_powerUpRx();
    }
}

void clearAddresses(){
    uint8_t count;
    positions.beaconCount = 0;
    for(count = 1; count < MAX_ADDRESSES; count++){
       positions.address[count][0] = 0; 
    }
}

void addAddress(uint8_t add, uint8_t type){
    positions.address[add][0] = add;
    positions.address[add][1] = ADD_CHAN;
    positions.address[add][2] = ADD_CHAN;
    positions.address[add][3] = ADD_CHAN;
    positions.address[add][4] = ADD_CHAN;
    positions.type[add] = type;
    positions.beaconCount++;
}

void transmit(uint8_t add){
    uint32_t timestamp;
    uint8_t count;

    for(count = 1; count < positions.beaconCount; count++){
        if(add == positions.address[count][0]){
            break;
        }
    }

    // TODO: Enable analog switches

    // TODO: Transmitting DAC signal - SPI


    timestamp = calculateGT(PIT_CVAL2 + 1000);  // 1ms for half a heartbeat - TODO: Make this 1ms variable dependant on something

    uint8_t data_out[RF_PACKET_SIZE];
    nrf24_tx_address(positions.address[count]);    
    data_out[0] = 'e';
    data_out[1] = (timestamp>>24)&0xFF;
    data_out[2] = (timestamp>>16)&0xFF;
    data_out[3] = (timestamp>>8)&0xFF;
    data_out[4] = timestamp & 0xFF;
    nrf24_send(data_out);        
    while(nrf24_isSending());
    nrf24_powerUpRx();
}

void handleRF(){
    uint8_t data_in[RF_PACKET_SIZE];
    receivedRF = 0;
    uint8_t count;
    if(nrf24_dataReady()){
        nrf24_getData(data_in);
        switch(data_in[0]){
            case 's':   // Time sync message
                calcTimeSync(data_in);
                break;
            case 'r':   // Reset time sync
                resetTimeSync();
                break;
            case 'f':   // Enable fast sync
                enableFastSync();
                break;
            case 'c':   // Calibrate
                xprintf("Received 'start calibrate'\r\n");
                mode = MODE_WAIT;
                calibrateStartAddress = data_in[1];
                clearPositionData();
                sync_init(SYNC_MODE_SLAVE, DMAMUX_SOURCE_PORTD);
                break;
            case 'w':   // Device registration
                if(data_in[1] == 0){
                    startDeviceTimer(positions.address[0][0]);
                    clearAddresses();
                }else{
                    xprintf("Received address: %x, from existing device of type %c\r\n", data_in[1], data_in[2]);
                    addAddress(data_in[1], data_in[2]);
                }
                break;
            case 't':   // Request transmit ultrasonic
                transmit(data_in[1]);
                break;
            case 'd':   // Receive measured distance
                for(count = 1; count < positions.beaconCount; count++){
                    if(data_in[1] == positions.address[count][0]){
                        positions.r[count] = (data_in[2]<<24) + (data_in[3]<<16) + (data_in[4]<<8) + data_in[5];
                        break;
                    }
                }
                break;
            case 'p':   // Receive measured x, y, z position
                receivePosition(data_in);
                break;
            case 'e':   // Transmit time from slave
                ultrasonicTransmitTime = (data_in[1]<<24) + (data_in[2]<<16) + (data_in[3]<<8) + data_in[4];
                break;
            case 'm':   // Receive master mode
                receiveMaster(data_in);
                break;
        }
    }
}

void dma_ch0_isr(void){ // Fires on done sampling
    DMA_CINT = 0;
    bufferFlag = 1;
}

void registerDevice(){
    // TODO: Transmitter or Receiver? and if receiver, am i first? (master)
    uint8_t data_out[RF_PACKET_SIZE];

    clearAddresses();
    nrf24_tx_address(broadCastAddress);
    data_out[0] = 'w';
    data_out[1] = 0;
    data_out[2] = 't';
    nrf24_send(data_out);        
    while(nrf24_isSending());
    nrf24_powerUpRx();
    registering = 1;
    LED_mode = LED_REGISTERING;
    startDeviceTimer(MAX_ADDRESSES);

    while(registering == 1){
        if(receivedRF == 1)
            handleRF(); 
        //neo_show();
    }
}

void handleNotification(){
    switch(LED_mode){
        case LED_CALIBRATING:
            break;
        case LED_NOT_CALIBRATED:
            break;
        case LED_RUNNING:
            break;
        case LED_ERROR:
            break;
        case LED_WAIT:
            break;
        case LED_REGISTERING:
            neo_setPixel(0, 0x000020);
            break;
    }
    neo_show();
}

int main(){
    SIM_CLKDIV1 |= SIM_CLKDIV1_OUTDIV2(1);
    calibrated = 0;
    LED_mode = LED_NOT_CALIBRATED;
    mode = MODE_WAIT;

    switch_init();

    /* Init the xprintf library */
    xdev_out(usb_serial_putchar);

    ADC_VALS adc;

    adc.bufsize = BUFSIZE;
    adc.fs = FS;
    adc.channel = 0x05; // PTC9
    adc.pause_samples = 1024;
    adc.CV1 = 868;//771;
    adc.CV2 = 682;//407;
    adc.cycle = 1;
    adc.destination = samples;

    adc_init(&adc);

    //dac_init();

    //neo_init(16, displayMemory, drawingMemory);
    //neo_show();

    /* Init RF */
    nrf24_init();
    broadCastAddress[0] = ADD_CHAN;
    broadCastAddress[1] = ADD_CHAN;
    broadCastAddress[2] = ADD_CHAN;
    broadCastAddress[3] = ADD_CHAN;
    broadCastAddress[4] = ADD_CHAN;

    
    nrf24_config(1,RF_PACKET_SIZE);
    nrf24_rx_address(broadCastAddress);
    nrf24_rx_broadcast_address(broadCastAddress);
 
    /* Init sync protocol */
    SIM_SCGC5 |= SIM_SCGC5_PORTB;
    PORTB_PCR0 |= PORT_PCR_MUX(1) | PORT_PCR_IRQC(2);
    //sync_init(SYNC_MODE_MASTER, DMAMUX_SOURCE_PORTB);

    registerDevice();

    passCount = 0;
    calibCount = 1;

    //gpio_init();


    while(1){
        if(receivedRF == 1)
            handleRF();    

        /*switch(mode){
            case MODE_CALIBRATE_MASTER:
                xprintf("MODE_MASTER\r\n");
               /* if(passCount == 0){
                    //passCalibrateMaster();   
                }
                if(bufferFlag == 1){
                    bufferFlag = 0;
                    calculateDistance();
                    if(passCount == 1){
                        positions.x[0] = positions.r[passCount];
                        positions.y[0] = 0;
                        positions.z[0] = 0;
                        announcePosition();
                        passCalibrateMaster();
                    }else if(passCount == 2){
                        requestTransmit(calibrateStartAddress);
                        calculatePosition();
                        announcePosition();
                        passCalibrateMaster();
                    }else if(calibCount-1 < passCount){
                        requestTransmit(calibCount);
                    }else{
                        calculatePosition();
                        announcePosition();
                        if(passCount < positions.beaconCount-1)
                            passCalibrateMaster();
                    }
                }*/
       /*         break;
            case MODE_WAIT:
                break;
            default:
                xprintf("NOT MODE_MASTER\r\n");
                break;
        }
        uint8_t data_out[RF_PACKET_SIZE];
        data_out[0] = 'w';
    data_out[1] = 0;
    data_out[2] = 't';
    nrf24_send(data_out); 
        xprintf("ALIVE %d\r\n",(nrf24_getStatus() & (1 << TX_DS)));

        //xprintf("\r\n----- ADDRESS: %x \t SYNC GT: %ld\r\n\r\n", positions.address[0][0], sync.GTm);

        //handleNotification();   */   

        delay(10);
    }
}
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
#include "spi.h"

#define ADD_CHAN 0xF0

volatile uint8_t registering;
volatile uint8_t bufferFlag;
uint8_t new_transmit_time;
uint8_t LED_mode;
uint16_t LED_counter; 

DMAMEM int displayMemory[16*6];
int drawingMemory[16*6];

void lptmr_isr();
void clearAddresses();
void addAddress(uint8_t add, uint8_t type);
void transmit(uint8_t add);
void handleRF();
void dma_ch4_isr(void);
void registerDevice();
void handleNotification();

void lptmr_isr(){
    LPTMR0_CSR = LPTMR_TCF;
    LPTMR0_CSR &= ~LPTMR_TEN;
    if(mode == MODE_CALIBRATE_MASTER){  // Used for ultrasonic timeout
        // TODO: Timeout occured, invalid address check? and move on to next device.. special case considering passCount
        //requestTransmit(calibCount);
        pit_run(0);
        xprintf("Ultrasonic timeout occurred\r\n");
        new_transmit_time = 0;
        bufferFlag = 0;
        mode = MODE_WAIT;
    }else{  // Used for registering
        uint8_t data_out[RF_PACKET_SIZE];
        if(registering == 1){
            registering = 0;
            uint8_t count, count2, found;
            found = 1;
            xprintf("\tFound %d devices on network:\r\n", positions.beaconCount);
            for(count = 1; count < positions.beaconCount+1; count++){
                if(positions.type[count] == 't')
                    xprintf("\t%d. Transmitter on address: 0x%x \r\n", count, positions.address[count][0]);
                else
                    xprintf("\t%d. Receiver on address: 0x%x \r\n", count, positions.address[count][0]);
            }
            for(count = 1; count < positions.beaconCount+1; count++){
                for(count2 = 1; count2 < positions.beaconCount+1; count2++){
                    found = 1;
                    if(positions.address[count2][0] == count){
                        found = 0;
                        break;
                    }
                }
                if(found == 1)
                    break;
            }
            positions.address[0][0] = count;
            positions.address[0][1] = ADD_CHAN;
            positions.address[0][2] = ADD_CHAN;
            positions.address[0][3] = ADD_CHAN;
            positions.address[0][4] = ADD_CHAN;
            nrf24_rx_address(positions.address[0]);
            xprintf("\r\nRegistered on network address: 0x%x\r\n\r\n",positions.address[0][0]);
            LED_mode = LED_NOT_CALIBRATED;
            data_out[3] = 'n';
        }else{
            data_out[3] = 0;
        }
        data_out[0] = 'w';
        data_out[1] = positions.address[0][0];
        data_out[2] = 't';
        nrf24_broadcast(data_out);        
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
    positions.beaconCount++;
    positions.address[positions.beaconCount][0] = add;
    positions.address[positions.beaconCount][1] = ADD_CHAN;
    positions.address[positions.beaconCount][2] = ADD_CHAN;
    positions.address[positions.beaconCount][3] = ADD_CHAN;
    positions.address[positions.beaconCount][4] = ADD_CHAN;
    positions.type[positions.beaconCount] = type;
}

//#define FREQ_SEP (1/((F2-F1)*2) * 1e6)

void transmit(uint8_t add){
    int FREQ_SEP = (1/((F2-F1)*2) * 1e6);
    uint8_t count;
    for(count = 0; count < positions.beaconCount; count++){
        if(add == positions.address[count][0]){
            break;
        }
    }
    if(count == 0){
        xprintf("No other known beacon on the network!\r\n");
    }else{// if(in_sync > IN_SYNC_NUMBER){
            //xprintf("Transmitting!!\r\n");
            while(radioBusy == 1);
            dac_enable();

            //switch_set(SWITCH_ALL, 1);
            __disable_irq()
            // 1000 us = 1 ms for half a heartbeat (To get epoch time) - TODO: Make this 1ms variable dependant on something
            uint32_t timestamp = calculateGT((MAX_US - PIT_CVAL2) + FREQ_SEP);  
            dac_start();
            __enable_irq();

            _delay_us(FREQ_SEP*2);

            uint8_t data_out[RF_PACKET_SIZE];
            data_out[0] = 'e';
            data_out[1] = (timestamp>>24)&0xFF;
            data_out[2] = (timestamp>>16)&0xFF;
            data_out[3] = (timestamp>>8)&0xFF;
            data_out[4] = timestamp & 0xFF;    
            nrf24_broadcast(data_out);      
    /*}else{
        xprintf("OUT OF SYNC!!\r\n");

        uint8_t data_out[RF_PACKET_SIZE];

        data_out[0] = 'o';
        data_out[1] = positions.address[0][0];
        nrf24_send(data_out, positions.address[count]);  */    
    }  

    mode = MODE_WAIT;    
}

void handleRF(){
    uint8_t data_in[RF_PACKET_SIZE];
    receivedRF = 0;
    uint8_t count;
    if(nrf24_dataReady()){
        nrf24_getData(data_in);
        switch(data_in[0]){
            case 's':   // Time sync message
                if(sync.mode == SYNC_MODE_SLAVE)
                    calcTimeSync(data_in);
                break;
            case 'r':   // Reset time sync
                xprintf("  !!!  Received reset timesync\r\n");
                resetTimeSync();
                break;
            case 'f':   // Enable fast sync
                if(sync.mode == SYNC_MODE_MASTER)
                    enableFastSync();
                break;
            case 'c':   // Calibrate
                //mode = MODE_CALIBRATE;
                LED_mode = LED_CALIBRATING;
                calibrateStartAddress = data_in[1];
                clearPositionData();
                break;
            case 'w':   // Device registration
                if(data_in[1] == 0 && data_in[2] == 'g'){
                    startDeviceTimer(positions.address[0][0]);
                    clearAddresses();
                }else if(data_in[2] == 't' || data_in[2] == 'r'){
                    if(!registering){
                        if(data_in[2] == 't' && data_in[3] == 'n')
                            xprintf("New transmitter joined the network on address 0x%x\r\n", data_in[1]);
                        else if(data_in[3] == 'n')
                            xprintf("New receiver joined the network on address 0x%x\r\n", data_in[1]);
                        if(sync.mode == SYNC_MODE_MASTER)
                            enableFastSync();
                    }
                    addAddress(data_in[1], data_in[2]);
                }
                break;
            case 'u':   // Request transmit ultrasonic
                transmit(data_in[1]);
                break;
            case 'd':   // Received measured distance
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
                new_transmit_time = 1;
                break;
            case 'm':   // Receive master mode
                receiveMaster(data_in);
                break;
            case 'o':
                if(mode == MODE_CALIBRATE_MASTER){
                    LPTMR0_CSR = LPTMR_TCF;
                    LPTMR0_CSR &= ~LPTMR_TEN;
                    pit_run(0);
                    delay(200);
                    xprintf("OUT OF SYNC!\t");
                    requestTransmit(data_in[1]);
                }
        }
    }
}

void dma_ch4_isr(void){ // Fires on done sampling
    pit_run(0);
    //set_mux(0,0x00);    // Disable mux
    bufferFlag = 1;
    DMA_CINT = 4;
}

void registerDevice(){
    // TODO: Transmitter or Receiver? and if receiver, am i first? (master)
    uint8_t data_out[RF_PACKET_SIZE];
    xprintf("\r\nRegistering device on network\r\n");
    clearAddresses();
    data_out[0] = 'w';
    data_out[1] = 0;
    data_out[2] = 'g';
    nrf24_broadcast(data_out);        
    registering = 1;
    LED_mode = LED_REGISTERING;
    startDeviceTimer(MAX_ADDRESSES);

    while(registering == 1){
        if(receivedRF == 1)
            handleRF(); 
        handleNotification();
    }
}

void handleNotification(){
    uint8_t i;
    switch(LED_mode){
        // TODO: Make LEDs run of timer to make 'living' design patterns
        case LED_CALIBRATING:
            for(i = 0; i<16;i++)
                neo_setPixel(i, 0x002000);
            break;
        case LED_NOT_CALIBRATED:
            for(i = 0; i<16;i++)
                neo_setPixel(i, 0x331D00);
            break;
        case LED_RUNNING:
            break;
        case LED_ERROR:
            for(i = 0; i<16;i++)
                neo_setPixel(i, 0x500000);
            break;
        case LED_WAIT:
            for(i = 0; i<16;i++)
                neo_setPixel(i, 0x200020);
            break;
        case LED_REGISTERING:
            for(i = 0; i<16;i++)
                neo_setPixel(i, 0x000020);
            break;
    }
    neo_show();
}

int main(){
    // TODO: Add __disable_irq() & __enable_irq() where its missing
    // TODO: Set IRQ priorities, NVIC_SetPriority(IRQn_t IRQn, uint32_t priority)
    SIM_CLKDIV1 |= SIM_CLKDIV1_OUTDIV2(1);
    calibrated = 0;
    LED_mode = LED_NOT_CALIBRATED;
    //mode = MODE_WAIT;

    /* Init the xprintf library */
    xdev_out(usb_serial_putchar);
    delay(2000);
    xprintf("\033[2J\033[1;1H");
    xprintf("\r\n############################################");
    xprintf("\r\n##                                        ##");
    xprintf("\r\n##              INITIALIZING              ##");
    xprintf("\r\n##                                        ##");
    xprintf("\r\n############################################\r\n\r\n");


    ADC_VALS adc;

    adc.bufsize = BUFSIZE;
    adc.fs = FS;
    adc.channel = 0x05; // PTC9
    adc.CV1 = 560;//868;//771;
    adc.CV2 = 460;//682;//407;    
    adc.cycle = 0;
    adc.destination = samples;

    dac_init();

    adc_init(&adc);


    neo_init(16, displayMemory, drawingMemory);
    handleNotification();

    /* Init RF */
    nrf24_init();
    broadCastAddress[0] = ADD_CHAN;
    broadCastAddress[1] = ADD_CHAN;
    broadCastAddress[2] = ADD_CHAN;
    broadCastAddress[3] = ADD_CHAN;
    broadCastAddress[4] = ADD_CHAN;
    
    nrf24_config(1,RF_PACKET_SIZE);
    nrf24_rx_address(broadCastAddress);             // Must set rx address once to get bytes 1-4 of the broadcast address due to sharing.
    nrf24_rx_broadcast_address(broadCastAddress);
    delay(100);

    /* Init sync protocol */
    SIM_SCGC5 |= SIM_SCGC5_PORTD;
    PORTD_PCR3 |= PORT_PCR_MUX(1) | PORT_PCR_IRQC(2);
    sync_init(SYNC_MODE_SLAVE, DMAMUX_SOURCE_PORTD);

    registerDevice();

    passCount = 0;
    calibCount = 1; 
    firstPress = 0;
    bufferFlag = 0;
    new_transmit_time = 0;
    calib_transmit_flag = 0;

    gpio_init();

    while(1){
        if(receivedRF == 1)
            handleRF();    

        switch(mode){
            case MODE_CALIBRATE_MASTER:
                if(passCount == 0){
                    passCalibrateMaster();
                }
                if(calib_transmit_flag == 1 && sync.fastSyncEnabled == 0){
                    calib_transmit_flag = 0;
                    requestTransmit(lastMasterAddress);
                }

                
                /*if(firstPress==1){
                    firstPress = 0;
                    LED_mode = LED_CALIBRATING;
                    requestTransmit(broadCastAddress[0]);
                }*/
                break;
            case MODE_WAIT:
                if(in_sync > IN_SYNC_NUMBER || sync.mode == SYNC_MODE_MASTER)
                    LED_mode = LED_WAIT;
                else
                    LED_mode = LED_ERROR;
                break;
        }
        if(bufferFlag == 1 && new_transmit_time == 1){
            new_transmit_time = 0;
            bufferFlag = 0;
            LPTMR0_CSR = LPTMR_TCF;
            LPTMR0_CSR &= ~LPTMR_TEN;

            calculateDistance(samples);
            if(calibrate_avg < CALIBRATE_AVG_NUMBER){
                delay(5);
                requestTransmit(last_request);
            }else{
                xprintf("AVG distance after avg of %d is %d\r\n",calibrate_avg, positions.r[passCount]/calibrate_avg);
                if(passCount == 1){
                    positions.x[0] = positions.r[passCount]/calibrate_avg;
                    positions.y[0] = 0;
                    positions.z[0] = 0;
                    announcePosition();
                    passCalibrateMaster();
                }else if(passCount == 2){
                    if(calibCount++ == (1+calibrate_avg)){
                        positions.r[passCount-1] = positions.r[passCount];
                        requestTransmit(calibrateStartAddress);
                    }else{
                        calculatePosition();
                        announcePosition();
                        uint8_t count;
                        xprintf("Known beacon positions [x,y,z]:\r\n");
                        for(count = 0; count < positions.beaconCount+1; count++){
                            xprintf("\t 0x%x - [%5d,%5d,%5d]\r\n", positions.address[count][0], positions.x[count], positions.y[count], positions.z[count]);
                        }
                    }
                }/*else if(calibCount-1 < passCount){
                    requestTransmit(calibCount);
                }else{
                    calculatePosition();
                    announcePosition();
                    if(passCount < positions.beaconCount-1)
                        passCalibrateMaster();
                }*/
            }
            mode = MODE_WAIT;
        }
        nrf24_runQueue();
        handleNotification(); 
        delay(1);
    }
}
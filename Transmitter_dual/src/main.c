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
#include "adc.h"
#include "frequency_calc.h"

#define MAX_ADDRESSES 10

#define ULTRASONIC_TIMEOUT 1

#define C 0.00034029    // m/us

#define MODE_BRAKE 0
#define MODE_TRANSMIT 1
#define MODE_SAMPLE_0 2
#define MODE_SAMPLE_90 3
#define MODE_SAMPLE_180 4
#define MODE_SAMPLE_270 5
#define MODE_CALIBRATE 6
#define MODE_CALIBRATE_MASTER 7
#define MODE_WAIT 8

#define LPTMR_TEN 1<<0
#define LPTMR_TIE 1<<6
#define LPTMR_TCF 1<<7
#define LPTMR_PBYP 1<<2

#define FS 600000
#define BUFSIZE 1000

volatile uint8_t dip1, dip2, dip3;

typedef struct {
    // Index 0 is always self!
    uint8_t x[MAX_ADDRESSES];
    uint8_t y[MAX_ADDRESSES];
    uint8_t z[MAX_ADDRESSES];
    uint32_t r[MAX_ADDRESSES]; 
    uint8_t address[MAX_ADDRESSES][5];
    uint8_t beaconCount;
} geoInfo;

uint8_t broadCastAddress[5] = {0xFF,0xFF,0xFF,0xFF,0xFF};

extern q15_t w1_cs[BUFSIZE*2];
extern q15_t w2_cs[BUFSIZE*2];

uint8_t mode;
volatile uint8_t registering;
uint8_t calibrated = 0;
uint8_t calibCount = 1;
uint8_t bufferFlag = 0;
geoInfo positions;

uint32_t ultrasonicTransmitTime;

static q15_t samples[BUFSIZE];

void clearPositionData(){
    uint8_t count;
    for(count = 0; count < MAX_ADDRESSES; count++){
        positions.x[count] = 0;
        positions.y[count] = 0;
        positions.z[count] = 0;
        positions.r[count] = 0;
    }
}

void startCalibrate(){
    if(mode != MODE_CALIBRATE || mode != MODE_CALIBRATE_MASTER){
        uint8_t data_out[RF_PACKET_SIZE];
        mode = MODE_CALIBRATE_MASTER;
        calibCount = 1;
        nrf24_tx_address(broadCastAddress);
        data_out[0] = 'c';
        nrf24_send(data_out);        
        while(nrf24_isSending());
        nrf24_powerUpRx();
        clearPositionData();
    }else{
        // TODO: Already in calibrate mode. What to do? Re-start calibrate sequence?
    }
}

void startDeviceTimer(uint8_t seconds){
    LPTMR0_CMR = ((1000 / 2) * seconds);
    NVIC_ENABLE_IRQ(IRQ_LPTMR);
    LPTMR0_PSR = 1<<0; // LPO 1kHz as source with prescaler of 2
    LPTMR0_PSR |= LPTMR_PBYP;
    LPTMR0_CSR = LPTMR_TIE;
    LPTMR0_CSR |= LPTMR_TEN;
}


void requestTransmit(uint8_t addressNumber){
    uint8_t data_out[RF_PACKET_SIZE];
    if(addressNumber < positions.beaconCount){
        nrf24_tx_address(positions.address[addressNumber]);
        data_out[0] = 't';
        data_out[1] = positions.address[0][4];
        nrf24_send(data_out);        
        while(nrf24_isSending());
        nrf24_powerUpRx();
        startDeviceTimer(ULTRASONIC_TIMEOUT);
    }else{
        adc_run(0);
        // TODO: Ready to either calculate positions or pass on listener/master role.
        sync_init(SYNC_MODE_SLAVE, DMAMUX_SOURCE_PORTD); // Enter slave mode
        mode = MODE_CALIBRATE;
    }
}

void portd_isr(void){
    if(GPIOD_PDIR & 1<<2){
        startCalibrate();
    }
    if((GPIOD_PDIR & 1<<4) != dip1){
        dip1 = (GPIOD_PDIR & 1<<4);
        if(dip1)
            xprintf("Dip 1 changed to HIGH\r\n");
        else
            xprintf("Dip 1 changed to LOW\r\n");
    }
    if((GPIOD_PDIR & 1<<7) != dip2){
        dip2 = (GPIOD_PDIR & 1<<7);
        if(dip2)
            xprintf("Dip 2 changed to HIGH\r\n");
        else
            xprintf("Dip 2 changed to LOW\r\n");
    }
    if((GPIOD_PDIR & 1<<3) != dip3){
        dip3 = (GPIOD_PDIR & 1<<3);
        if(dip3)
            xprintf("Dip 3 changed to HIGH\r\n");
        else
            xprintf("Dip 3 changed to LOW\r\n");
    }
    delay(100);
}

void lptmr_isr(){
    LPTMR0_CSR = LPTMR_TCF;
    LPTMR0_CSR &= ~LPTMR_TEN;
    if(mode == MODE_CALIBRATE_MASTER){
        // TODO: Timeout occured, invalid address check? and move on to next device
        requestTransmit(calibCount++);
    }else{
        if(registering == 1){
            registering = 0;
            positions.address[0][4] = positions.beaconCount;   // TODO: FIX THIS! knowAddressCount does not necessarily mean unique address!
        }
        uint8_t data_out[RF_PACKET_SIZE];
        nrf24_tx_address(broadCastAddress);    
        data_out[0] = 'w';
        data_out[1] = positions.address[0][4];
        nrf24_send(data_out);        
        while(nrf24_isSending());
        nrf24_powerUpRx();
    }
}

void clearAddresses(){
    positions.beaconCount = 0;
}

void addAddress(uint8_t add){
    positions.address[++positions.beaconCount][0] = 0x0D;
    positions.address[positions.beaconCount][1] = 0x0D;
    positions.address[positions.beaconCount][2] = 0x0D;
    positions.address[positions.beaconCount][3] = 0x0D;
    positions.address[positions.beaconCount][4] = add;
}

void transmit(uint8_t add){

    uint32_t timestamp;
    uint8_t count;

    for(count = 1; count < positions.beaconCount; count++){
        if(add == positions.address[count][4]){
            break;
        }
    }

    // TODO: Transmitting DAC signal - SPI

    timestamp = calculateGT(PIT_CVAL2 + 1000);       // TODO: Make this 1ms variable dependant on something

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
            case 'c':   // Calibrate
                mode = MODE_CALIBRATE;
                clearPositionData();
                sync_init(SYNC_MODE_SLAVE, DMAMUX_SOURCE_PORTD);
                break;
            case 'w':   // Device registration
                if(data_in[1] == 0){
                    startDeviceTimer(positions.address[0][4]);
                    clearAddresses();
                }else{
                    addAddress(data_in[1]);
                }
                break;
            case 't':   // Request transmit ultrasonic
                transmit(data_in[1]);
                break;
            case 'd':   // Receive measured distance
                for(count = 1; count < positions.beaconCount; count++){
                    if(data_in[1] == positions.address[count][4]){
                        positions.r[count] = (data_in[2]<<24) + (data_in[3]<<16) + (data_in[4]<<8) + data_in[5];
                        break;
                    }
                }
                break;
            case 'p':   // Receive measured x, y, z position
                break;
            case 'e':   // Transmit time from slave
                ultrasonicTransmitTime = (data_in[1]<<24) + (data_in[2]<<16) + (data_in[3]<<8) + data_in[4];
                break;
        }
    }
}

void dma_ch0_isr(void){ // Fires on done sampling
    DMA_CINT = 0;
    bufferFlag = 1;
}

void calculateDistance(){
    q15_t Qi1[BUFSIZE*2];
    q15_t Qi2[BUFSIZE*2];
    q15_t phase1[BUFSIZE];
    q15_t phase2[BUFSIZE];
    q15_t phase_diff[BUFSIZE];
    uint16_t index = 0;
    arm_cmplx_mult_real_q15(w1_cs, samples, Qi1, BUFSIZE);
    arm_cmplx_mult_real_q15(w2_cs, samples, Qi2, BUFSIZE);
    atan2_fp(Qi1, phase1, BUFSIZE*2);  
    atan2_fp(Qi2, phase2, BUFSIZE*2);  
    arm_sub_q15(phase2,phase1,phase_diff, BUFSIZE);
    if(phase_diff[BUFSIZE/2] == 0){
        index = BUFSIZE/2;
    }else if(phase_diff[BUFSIZE/2] < 0){
        for(index = BUFSIZE/2; index > 0; index--){
            if(phase_diff[index] > 0)
                break;
        }
    }else{
        for(index = BUFSIZE/2; index < BUFSIZE; index++){
            if(phase_diff[index] < 0)
                break;
        }
    }

    positions.r[calibCount] = (sampleStartGT - ultrasonicTransmitTime) / C;    
}

void registerDevice(){
    uint8_t data_out[RF_PACKET_SIZE];

    clearAddresses();
    data_out[0] = 'w';
    data_out[1] = 0;
    nrf24_send(data_out);        
    while(nrf24_isSending());
    nrf24_powerUpRx();
    registering = 1;
    startDeviceTimer(MAX_ADDRESSES);
    while(registering == 1){
        if(receivedRF == 1)
            handleRF(); 
    }
}


int main(){
    SIM_CLKDIV1 |= SIM_CLKDIV1_OUTDIV2(1);

    SIM_SCGC5 |= SIM_SCGC5_PORTD | SIM_SCGC5_PORTA;
    PORTD_PCR3 |= PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x0b);
    PORTD_PCR7 |= PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x0b);
    PORTD_PCR4 |= PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x0b);
    PORTD_PCR2 |= PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x09);
    NVIC_ENABLE_IRQ(IRQ_PORTD);

    dip1 = (GPIOD_PDIR & 1<<4);
    dip2 = (GPIOD_PDIR & 1<<7);
    dip3 = (GPIOD_PDIR & 1<<3);

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

    /* Init RF */
    nrf24_init();
    
    nrf24_config(1,RF_PACKET_SIZE);
    nrf24_tx_address(broadCastAddress);
    // TODO: Register both broadcast RX and personal rx address
    nrf24_rx_address(broadCastAddress);
 
    registerDevice();

    /* Init sync protocol */
    SIM_SCGC5 |= SIM_SCGC5_PORTD;
    PORTD_PCR0 |= PORT_PCR_MUX(1) | PORT_PCR_IRQC(2);
    sync_init(SYNC_MODE_SLAVE, DMAMUX_SOURCE_PORTD);

    while(1){
        if(receivedRF == 1)
            handleRF();    

        switch(mode){
            case MODE_CALIBRATE:
                // TODO: Change indicator to reflect calibrate mode
                break;
            case MODE_CALIBRATE_MASTER:
                if(calibCount == 1){
                    sync_init(SYNC_MODE_MASTER, DMAMUX_SOURCE_PORTD);
                    enableFastSync();
                    delay(2500); // TODO: Too much time???
                    adc_run(1);
                    requestTransmit(calibCount++);
                }
                if(bufferFlag == 1){
                    bufferFlag = 0;
                    calculateDistance();
                    uint8_t data_out[RF_PACKET_SIZE];
                    nrf24_tx_address(positions.address[calibCount]);
                    data_out[0] = 'd';
                    data_out[1] = positions.address[0][4];
                    data_out[2] = (positions.r[calibCount] >> 24) & 0xFF;
                    data_out[3] = (positions.r[calibCount] >> 16) & 0xFF;
                    data_out[4] = (positions.r[calibCount] >> 8) & 0xFF;
                    data_out[5] = positions.r[calibCount] & 0xFF;
                    nrf24_send(data_out);
                    while(nrf24_isSending());
                    nrf24_powerUpRx();
                    requestTransmit(calibCount++);
                }
                break;
            case MODE_WAIT:
                break;
        }        
        delay(10);
    }
}
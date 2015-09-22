#include "calibrate.h"
#include "xprintf.h"
#include <stdio.h>

void startDeviceTimer(uint8_t seconds){
    if((SIM_SCGC5 & SIM_SCGC5_LPTIMER) == 0)
        SIM_SCGC5|=SIM_SCGC5_LPTIMER;
    LPTMR0_CMR = (500 * seconds);
    NVIC_ENABLE_IRQ(IRQ_LPTMR);
    LPTMR0_PSR = 1<<0; // LPO 1kHz as source with prescaler of 2
    LPTMR0_PSR |= LPTMR_PBYP;
    LPTMR0_CSR = LPTMR_TIE;
    LPTMR0_CSR |= LPTMR_TEN;
}

void clearPositionData(){
    uint8_t count;
    for(count = 0; count < MAX_ADDRESSES; count++){
        positions.x[count] = 0;
        positions.y[count] = 0;
        positions.z[count] = 0;
        positions.r[count] = 0;
    }
}

void receiveMaster(uint8_t * data){
    passCount = data[2];
    lastMasterAddress = data[1];
    calibCount = 1;
    xprintf("Received MASTER\r\n");
    mode = MODE_CALIBRATE_MASTER;
    sync_init(SYNC_MODE_MASTER, DMAMUX_SOURCE_PORTD);
    enableFastSync();
    calib_transmit_flag = 1;
    last_request = 0;
}

void passCalibrateMaster(){
    uint8_t data_out[RF_PACKET_SIZE];
    data_out[0] = 'm';
    data_out[1] = positions.address[0][0];
    data_out[2] = ++passCount;
    nrf24_send(data_out, positions.address[passCount%(positions.beaconCount+1)]);
    uint8_t temp = nrf24_lastMessageStatus();
    if(temp == NRF24_MESSAGE_LOST){
        xprintf("MESSAGE LOST!!!!!\r\n");
    }else if(temp == NRF24_TRANSMISSION_OK){
        uint8_t cnt = nrf24_retransmissionCount();
        xprintf("Successfully passed master with %d retries\r\n", cnt);
    }
    xprintf("Passing master to device address: 0x%x\r\n", positions.address[passCount%(positions.beaconCount + 1)][0]);
    sync_init(SYNC_MODE_SLAVE, DMAMUX_SOURCE_PORTD); // Enter slave mode
    mode = MODE_WAIT;
}

void requestTransmit(uint8_t addressNumber){
    calibCount++;
    uint8_t count;
    for(count = 0; count < positions.beaconCount; count++){
        if(addressNumber == positions.address[count][0]){
            break;
        }
    }
    if(count == 0){
        xprintf("No other beacons on the network!\r\n");
    }else{
        if(last_request != count){
            calibrate_avg = 0;
            positions.r[passCount] = 0;
        }
        last_request = count;
        uint8_t data_out[RF_PACKET_SIZE];
        xprintf("Requesting transmit from device address 0x%x\r\n",positions.address[count][0]);
        adc_run(1,0);
        //delay(1);
        mode = MODE_CALIBRATE_MASTER;
        data_out[0] = 'u';
        data_out[1] = positions.address[0][0];
        nrf24_send(data_out, positions.address[count]);        
              
        startDeviceTimer(ULTRASONIC_TIMEOUT);
    }
}

void announcePosition(){
    uint8_t data_out[RF_PACKET_SIZE];
    data_out[0] = 'p';
    data_out[1] = positions.address[0][0];
    data_out[2] = (positions.x[0]>>24)&0xFF;
    data_out[3] = (positions.x[0]>>16)&0xFF;
    data_out[4] = (positions.x[0]>>8)&0xFF;
    data_out[5] = positions.x[0]&0xFF;
    data_out[6] = (positions.y[0]>>24)&0xFF;
    data_out[7] = (positions.y[0]>>16)&0xFF;
    data_out[8] = (positions.y[0]>>8)&0xFF;
    data_out[9] = positions.y[0]&0xFF;
    data_out[10] = (positions.z[0]>>24)&0xFF;
    data_out[11] = (positions.z[0]>>16)&0xFF;
    data_out[12] = (positions.z[0]>>8)&0xFF;
    data_out[13] = positions.z[0]&0xFF;
    nrf24_broadcast(data_out);        
    calibrated = 1;    
}

void receivePosition(uint8_t * data){
    uint8_t count;
    for(count = 0; count < positions.beaconCount; count++){
        if(data[1] == positions.address[count][0])
            break;
    }
    if(count == 0){
        xprintf("No other beacons on the network! - Received position data of myself?? \r\n");
    }else{
        positions.x[count] = (data[2]<<24) + (data[3]<<16) + (data[4]<<8) + data[5];
        positions.y[count] = (data[6]<<24) + (data[7]<<16) + (data[8]<<8) + data[9];
        positions.z[count] = (data[10]<<24) + (data[11]<<16) + (data[12]<<8) + data[13];
    }
}

void startCalibrate(){
    if(mode != MODE_CALIBRATE_MASTER){
        sync_init(SYNC_MODE_MASTER, DMAMUX_SOURCE_PORTD);
        /*passCount = 0;
        calibCount = 1;
        last_request = 0;
        //firstPress = 1;    

        uint8_t data_out[RF_PACKET_SIZE];
        data_out[0] = 'c';
        data_out[1] = positions.address[0][0];
        nrf24_broadcast(data_out);        
        clearPositionData();
        announcePosition();

        mode = MODE_CALIBRATE_MASTER;*/
    }else{
        xprintf("Already calibrating dude, CHILL!\r\n");

        // TODO: Already in calibrate mode. What to do? Re-start calibrate sequence?
    }
}

void calculatePosition(){
    if(passCount == 2){
        positions.r[1] = positions.r[1]/CALIBRATE_AVG_NUMBER;
        positions.r[2] = positions.r[2]/CALIBRATE_AVG_NUMBER;
        double dx = (double)((int)positions.x[calibrateStartAddress] - (int)positions.x[lastMasterAddress]);
        double dy = (double)((int)positions.y[calibrateStartAddress] - (int)positions.y[lastMasterAddress]);
        double d = sqrt(dx * dx + dy * dy);
        
        double a = ((double)((int)positions.r[1] * (int)positions.r[1] - (int)positions.r[2] * (int)positions.r[2]) + d * d) / (2 * d);
        double h = sqrt((double)((int)positions.r[1] * (int)positions.r[1]) - a * a);

        double cx2 = (int)positions.x[lastMasterAddress] + a * dx / d;
        double cy2 = (int)positions.y[lastMasterAddress] + a * dy / d;

        positions.x[0] = (uint32_t)round(cx2 + h * dy / d);
        positions.y[0] = (uint32_t)round(cy2 - h * dx / d);
        /* TODO: Select the right +- value
            positions.x[0] = (cx2 + h * (positions.y[calibrateStartAddress] - positions.y[lastMasterAddress]) / d);
            positions.y[0] = (cy2 - h * (positions.x[calibrateStartAddress] - positions.x[lastMasterAddress]) / d);
         */
    }else{
        // TODO: Find 3 shortest distances and multilaterate from them
    }
}


/*
 *  Calculates the distance, based on TDOF method using sampled sync pattern and received transmission time.
 *
 *  Params:
 *      q15_t * adcs            -   Sampled values of sync signal on receiver side.
 *  
 *  Others:
 *      w1_cs, w2_cs            -   Phasor of frequency 39750 Hz and 40250 Hz, created in MATLAB (extern from phasors.c)
 *      sampleStartGT           -   Timestamp of samling window start, in global time (Saved in adc.c ISR)
 *      ultrasonicTransmitTime  -   Timestamp of ultrasonic transmission on transmitter side, in global time (Received on RF in main.c)
 *      FS                      -   Samplerate of adcs signal on receiver side, in Hz (512000 Hz) (Defined in calibrate.h)
 */
void calculateDistance(q15_t * adcs){
    double C_M = (331.5 + (0.6 * TEMP)) * 1e-3;
    float32_t adc_f[BUFSIZE];
    float32_t mean;
    double adc_d[BUFSIZE];
    double phase1, phase2, t_e;
    uint16_t count;

    arm_shift_q15(adcs, 5, adcs, BUFSIZE);
    arm_q15_to_float(adcs, adc_f, BUFSIZE);
    arm_mean_f32(adc_f, BUFSIZE, &mean);
    for(count = 0; count < BUFSIZE; count++){
        adc_d[count] = (double)(adc_f[count] - mean);
    }
    phase1 = dot_product_atan2(w1_cs, adc_d, BUFSIZE);        
    phase2 = dot_product_atan2(w2_cs, adc_d, BUFSIZE); 
    t_e = ((phase1 - phase2) / (2 * M_PI * (F1 - F2)));

    // 1e6 to make s to us conversion.
    // add (1/FS * 1e6 * BUFSIZE/2), to get the window center time.
    // (MAX_US - sampleStartGT), to compensate for timer counting downwards.
    uint32_t epochReceivedTime = calculateGT((MAX_US - sampleStartGT) + (uint32_t)(( (((double)1/(double)FS) * BUFSIZE/2) + t_e) * 1e6));

    positions.r[passCount] += (uint32_t)((double)(epochReceivedTime - ultrasonicTransmitTime) * C_M) - BEACON_OFFSET;   
    calibrate_avg++;
    xprintf("Distance measured: %d\r\n",(uint32_t)((double)(epochReceivedTime - ultrasonicTransmitTime) * C_M) - BEACON_OFFSET);

}
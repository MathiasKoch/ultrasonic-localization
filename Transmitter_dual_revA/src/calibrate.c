#include "calibrate.h"
#include "xprintf.h"

void startDeviceTimer(uint8_t seconds){
    if((SIM_SCGC5 & SIM_SCGC5_LPTIMER) == 0)
        SIM_SCGC5|=SIM_SCGC5_LPTIMER;
    LPTMR0_CMR = ((1000 / 2) * seconds);
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
    mode = MODE_CALIBRATE_MASTER;
    sync_init(SYNC_MODE_MASTER, DMAMUX_SOURCE_PORTD);
    enableFastSync();
    delay(2500);
    if(passCount < 3)
        requestTransmit(lastMasterAddress);
    else
        requestTransmit(calibCount);
}

void passCalibrateMaster(){
    uint8_t data_out[RF_PACKET_SIZE];
    mode = MODE_WAIT;
    adc_run(0,1);
    sync_init(SYNC_MODE_SLAVE, DMAMUX_SOURCE_PORTD); // Enter slave mode
    nrf24_tx_address(positions.address[100]);// TODO: Find the next master?
    data_out[0] = 'm';
    data_out[1] = positions.address[0][0];
    data_out[2] = ++passCount;
    nrf24_send(data_out);
    while(nrf24_isSending());
    nrf24_powerUpRx();
}

void requestTransmit(uint8_t addressNumber){
    calibCount++;
    uint8_t data_out[RF_PACKET_SIZE];
    if(addressNumber < positions.beaconCount){
        adc_run(1,1);
        nrf24_tx_address(positions.address[addressNumber]);
        data_out[0] = 't';
        data_out[1] = positions.address[0][0];
        nrf24_send(data_out);        
        while(nrf24_isSending());
        nrf24_powerUpRx();
        startDeviceTimer(ULTRASONIC_TIMEOUT);
    }
}

void announcePosition(){
    uint8_t data_out[RF_PACKET_SIZE];
    nrf24_tx_address(broadCastAddress);    
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
    nrf24_send(data_out);        
    while(nrf24_isSending());
    nrf24_powerUpRx();
    calibrated = 1;    
}

void receivePosition(uint8_t * data){
    uint8_t count;
    for(count = 1; count < positions.beaconCount; count++){
        if(data[1] == positions.address[count][0])
            break;
    }
    positions.x[count] = (data[2]<<24) + (data[3]<<16) + (data[4]<<8) + data[5];
    positions.y[count] = (data[6]<<24) + (data[7]<<16) + (data[8]<<8) + data[9];
    positions.z[count] = (data[10]<<24) + (data[11]<<16) + (data[12]<<8) + data[13];
}

void startCalibrate(){    
    if(mode != MODE_CALIBRATE_MASTER){
        mode = MODE_CALIBRATE_MASTER;
        uint8_t data_out[RF_PACKET_SIZE];
        calibCount = 1;
        passCount = 0;
        nrf24_tx_address(broadCastAddress);
        data_out[0] = 'c';
        data_out[1] = positions.address[0][0];
        nrf24_send(data_out);        
        while(nrf24_isSending()){};
        nrf24_powerUpRx();
        clearPositionData();
        announcePosition();
    }else{
        // TODO: Already in calibrate mode. What to do? Re-start calibrate sequence?
    }
}

void calculatePosition(){
    if(passCount == 2){
        uint32_t dx = positions.x[calibrateStartAddress] - positions.x[lastMasterAddress];
        uint32_t dy = positions.y[calibrateStartAddress] - positions.y[lastMasterAddress];
        uint32_t d = isqrt32(dx * dx + dy * dy);

        uint32_t a = (positions.r[1] * positions.r[1] - positions.r[2] * positions.r[2] + d * d) / (2 * d);
        uint32_t h = isqrt32(positions.r[1] * positions.r[1] - a * a);

        uint32_t cx2 = positions.x[lastMasterAddress] + a * (positions.x[calibrateStartAddress] - positions.x[lastMasterAddress]) / d;
        uint32_t cy2 = positions.y[lastMasterAddress] + a * (positions.y[calibrateStartAddress] - positions.y[lastMasterAddress]) / d;

        positions.x[0] = (cx2 + h * (positions.y[calibrateStartAddress] - positions.y[lastMasterAddress]) / d);
        positions.y[0] = (cy2 - h * (positions.x[calibrateStartAddress] - positions.x[lastMasterAddress]) / d);
        // TODO: Select the right +- value
        /*
        x2 = (cx2 - h * (positions.y[calibrateStartAddress] - positions.y[lastMasterAddress]) / d);
        y2 = (cy2 + h * (positions.x[calibrateStartAddress] - positions.x[lastMasterAddress]) / d);
*/

    }else{
        // TODO: Find 3 shortest distances and multilaterate from them
    }
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
    sampleStartGT += 1/FS * 1000000 * index;

    positions.r[passCount] = ((sampleStartGT - ultrasonicTransmitTime) * C) + (BEACON_OFFSET*2);    
}
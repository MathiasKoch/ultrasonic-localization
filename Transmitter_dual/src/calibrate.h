#ifndef CALIBRATE
#define CALIBRATE

#include <avr/io.h>
#include "frequency_calc.h"
#include "adc.h"
#include "nrf24.h"

#define LPTMR_TEN 1<<0
#define LPTMR_TIE 1<<6
#define LPTMR_TCF 1<<7
#define LPTMR_PBYP 1<<2

#define MODE_BRAKE 0
#define MODE_TRANSMIT 1
#define MODE_SAMPLE_0 2
#define MODE_SAMPLE_90 3
#define MODE_SAMPLE_180 4
#define MODE_SAMPLE_270 5
#define MODE_CALIBRATE_MASTER 6
#define MODE_WAIT 7
#define MODE_CALIBRATE 8

#define MAX_ADDRESSES 4

#define FS 512000
#define F1 39750
#define F2 40250
#define BUFSIZE 1024

#define CALIBRATE_AVG_NUMBER 10

#define TEMP 24


#define BEACON_OFFSET 165	// mm
#define ULTRASONIC_TIMEOUT 4

typedef struct {
    // Index 0 is always self!
    uint32_t x[MAX_ADDRESSES];
    uint32_t y[MAX_ADDRESSES];
    uint32_t z[MAX_ADDRESSES];
    uint32_t r[MAX_ADDRESSES]; 
    uint8_t address[MAX_ADDRESSES][5];
    uint8_t beaconCount;
    uint8_t type[MAX_ADDRESSES];
} geoInfo;

extern double w1_cs[BUFSIZE*2];
extern double w2_cs[BUFSIZE*2];

static q15_t samples[BUFSIZE];



uint8_t passCount;
uint8_t calibrateStartAddress;
uint8_t lastMasterAddress;
geoInfo positions;
volatile uint8_t mode;
uint8_t calibCount;
uint8_t calibrated;
uint8_t firstPress;
uint8_t calib_transmit_flag;
uint8_t last_request;
uint8_t calibrate_avg;

uint32_t ultrasonicTransmitTime;

void startDeviceTimer(uint8_t seconds);
void startCalibrate();
void calculatePosition();
void calculateDistance(q15_t * adcs);
void clearPositionData();
void announcePosition();
void receivePosition(uint8_t * data);
void receiveMaster(uint8_t * data);
void passCalibrateMaster();
void requestTransmit(uint8_t addressNumber);

#endif /* CALIBRATE */
#ifndef SYNC
#define SYNC

#include <avr/io.h>
#include "nrf24.h"

#define TST_SIZE 4
#define MIN_TST_ENTRIES 3
#define MAX_AVG_DIFF 3
#define MAX_US 4000000000LL
#define FAST_SYNC_COUNT 5
#define SYNC_OFFSET 2

#define SYNC_MODE_MASTER 0
#define SYNC_MODE_SLAVE 1

#define RF_PACKET_SIZE 14

#define TIE 0x2
#define TEN 0x1
#define CHN (1<<2)

typedef struct {
// Slave mode:
    uint32_t GT[TST_SIZE];
    uint32_t LT[TST_SIZE];
    uint32_t nLT;
    uint32_t _LT;
    uint32_t DIFF[TST_SIZE];
    uint8_t VALID[TST_SIZE];
    uint64_t SKEW;
    uint32_t OFFSET;
    uint8_t n;

// Master mode:
    volatile uint32_t GTm; 
    uint32_t nGT;
    volatile uint8_t sent;
    uint32_t SCP_normal;
    uint32_t SCP_fast;

// Shared:
    volatile uint32_t i;
    uint32_t im;
    uint8_t mode;
    volatile uint8_t fastSyncCounter;
    uint8_t fastSyncEnabled;

} TimeSync; 

TimeSync sync;

volatile uint8_t receivedRF;

void sync_init(uint8_t mode, uint8_t source);
void sync_dma_init(uint8_t source);
void sync_clock_init(void);
void enableFastSync(void);
void disableFastSync(void);
void requestFastSync(void);
void resetTimeSync(void);
void calcTimeSync(uint8_t * data);
uint32_t calculateGT(uint32_t timeS);

#endif
#include "sync.h"
#include "xprintf.h"
#include "nRF24L01.h"
#include <stdio.h>


void sync_init(uint8_t mode, uint8_t source){
    if(mode != sync.mode){
        xprintf("Initializing Sync module\t - \t");
    	sync.mode = mode;
    	sync.i = 1;
    	sync.SCP_normal =  ((F_BUS * SUPER_CYCLE_NORMAL_SEC)-1);
        sync.SCP_fast = ((F_BUS / 8) - 1);
        sync.SKEW = 0;
        sync.OFFSET = 0;
        sync._LT = 0;
        sync.n = 0;

    	sync_dma_init(source);
    	sync_clock_init();
        xprintf("Done\t");
        if(mode == SYNC_MODE_MASTER)
            xprintf("MASTER\r\n");
        else{
            in_sync = 0;
            xprintf("SLAVE\r\n");
        }
    }else{
        sync.SKEW = 0;
        sync.OFFSET = 0;
        sync._LT = 0;
        sync.n = 0;
        sync.i = 1;
    }
}

void pit2_isr(){
    PIT_TFLG2 = 1;
    if(sync.mode == SYNC_MODE_SLAVE){
        resetTimeSync();
        requestFastSync();
    }else{
        uint8_t data_out[RF_PACKET_SIZE];
        data_out[0] = 'r';
        nrf24_broadcast(data_out);        
        enableFastSync();
    }
}


void sync_dma_init(uint8_t source) {
    SIM_SCGC7 |= SIM_SCGC7_DMA;
    SIM_SCGC6 |= SIM_SCGC6_DMAMUX;

    DMA_TCD5_SADDR = &PIT_CVAL2;
    DMA_TCD5_SOFF = 0;
    DMA_TCD5_SLAST = 0;
    if(sync.mode == SYNC_MODE_MASTER)
    	DMA_TCD5_DADDR = &sync.nGT;
    else
    	DMA_TCD5_DADDR = &sync.nLT;
    DMA_TCD5_DOFF = 0;
    DMA_TCD5_DLASTSGA = 0;
    DMA_TCD5_ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2);
    DMA_TCD5_NBYTES_MLNO = 4;
    DMA_TCD5_CITER_ELINKNO = 0x0001;
    DMA_TCD5_BITER_ELINKNO = 0x0001;
    DMA_TCD5_CSR = DMA_TCD_CSR_INTMAJOR;

    DMAMUX0_CHCFG5 = DMAMUX_DISABLE;
    DMAMUX0_CHCFG5 = source | DMAMUX_ENABLE;

    DMA_SERQ = 5;

    NVIC_ENABLE_IRQ(IRQ_DMA_CH5);
}

void dma_ch5_isr(void){
    DMA_CINT = 5;
    if(sync.sent == 1){
        radio_sent = 0;
        sync.sent = 0;
        sync.GTm = MAX_US - sync.nGT;
    }else if(radio_sent == 1){
        radio_sent = 0;
        // TODO: Look into radio busy using this interrupt
    }else{
        receivedRF = 1; // TODO: DUNNO?
    }
}

void pit3_isr(){
    PIT_TFLG3 = 1;
    uint8_t data_out[RF_PACKET_SIZE];
    if(sync.fastSyncEnabled == 1){
        if(sync.fastSyncCounter++ >= FAST_SYNC_COUNT)
            disableFastSync();
    }
    sync.sent = 1;
    data_out[0] = 's';
    data_out[1] = (sync.i >> 8) & 0xFF;
    data_out[2] = sync.i & 0xFF;
    data_out[3] = (sync.GTm >> 24) & 0xFF;
    data_out[4] = (sync.GTm >> 16) & 0xFF;
    data_out[5] = (sync.GTm >> 8) & 0xFF;
    data_out[6] = sync.GTm & 0xFF;
    nrf24_broadcast(data_out);        
    sync.i++;
}


#define CLOCK_P ((F_BUS / 1000000)-1)    // 1 Mhz = 1 us

void sync_clock_init(void){
    SIM_SCGC6 |= SIM_SCGC6_PIT;
    PIT_MCR = 0x00;

    PIT_LDVAL2 = MAX_US;
    NVIC_ENABLE_IRQ(IRQ_PIT_CH2);
    PIT_TCTRL2 = TIE;
    PIT_TCTRL2 |= CHN;
    PIT_TCTRL2 |= TEN;

    PIT_LDVAL1 = CLOCK_P;
    PIT_TCTRL1 |= TEN;

    if(sync.mode == SYNC_MODE_MASTER){
	    PIT_LDVAL3 = sync.SCP_normal;
	    NVIC_ENABLE_IRQ(IRQ_PIT_CH3);
	    PIT_TCTRL3 = TIE;
	    PIT_TCTRL3 |= TEN;
	}
}

void enableFastSync(){
    sync.fastSyncEnabled = 1;
    sync.fastSyncCounter = 0;
    PIT_LDVAL3 = sync.SCP_fast;
}

void disableFastSync(){
    PIT_LDVAL3 = sync.SCP_normal;
    sync.fastSyncEnabled = 0;
}


void requestFastSync(){
    xprintf("Requesting fast sync\r\n");
    uint8_t data_out[RF_PACKET_SIZE];
    data_out[0] = 'f';
    nrf24_broadcast(data_out);        
}

void resetTimeSync(){
    uint8_t count;
    for(count = 0; count < TST_SIZE; count++){
        sync.LT[count] = 0;
        sync.DIFF[count] = 0;
        sync.GT[count] = 0;
        sync.n = 0;
    }
    sync_last = sync.i;
}


void calcTimeSync(uint8_t * data){
    uint16_t n;
    int offset, GTx;
    double skew;
    int offset_var, skew_var2;
    long long skew_var;
    uint64_t lt_var;
    GTx = 0;

    sync.i = (data[1]<<8) + data[2]; // i starts from 1

    /*if(sync.i != sync_last + 1){
        resetTimeSync();
        requestFastSync();
    }else{*/
        sync_last = sync.i;

        sync.im = (sync.i - 1) % TST_SIZE;

        sync.GT[sync.im] = (data[3]<<24) + (data[4]<<16) + (data[5]<<8) + data[6];
        

        sync.DIFF[sync.im] = (int)(sync.GT[sync.im] - sync.LT[sync.im]);
        // TODO: Check missing packet (i) and implement fast sync request
        
        if(sync.n >= MIN_TST_ENTRIES){
            offset_var = skew_var = skew_var2 = lt_var = 0;
          
            for(n = 0; n < sync.n; n++){
                lt_var += sync.LT[n];
                offset_var += sync.DIFF[n];
            }
            sync._LT = (lt_var / sync.n);
             

            offset = (int)(offset_var / sync.n);
            for(n = 0; n < sync.n; n++){
                skew_var += ((long long)((int)sync.LT[n] - (int)sync._LT) * (long long)((int)sync.LT[n] - (int)sync._LT));
                skew_var2 += ((int)sync.LT[n] - (int)sync._LT) * (sync.DIFF[n] - offset);
            }
            skew = ((double)skew_var2 / skew_var);
            GTx = ((int)sync.LT[sync.im] + offset + (int)(skew * (double)((int)sync.LT[sync.im] - sync._LT)));
            if(GTx > (sync.GT[sync.im] + MAX_AVG_DIFF) || GTx < (sync.GT[sync.im] - MAX_AVG_DIFF)){
                sync.VALID[sync.im] = 0;
                in_sync = 0;
            }else{
                sync.VALID[sync.im] = 1;
                sync.SKEW = skew;
                sync.OFFSET = offset;
                in_sync++;
            }
        }else{
            sync.VALID[sync.im] = 0;
            in_sync = 0;
        }

        xprintf("%d, ", sync.GT[sync.im]-calculateGT(sync.LT[sync.im]));
        if(sync.i%8 == 0)
            xprintf("\r\n");

        //xprintf("I: %3d, Valid: %1d, GT(%10d), GTx(%10d) \t diff: %d\r\n\r\n", sync.i, sync.VALID[sync.im], sync.GT[sync.im], calculateGT(sync.LT[sync.im]), sync.GT[sync.im]-calculateGT(sync.LT[sync.im]));
        sync.LT[sync.i % TST_SIZE] = MAX_US - sync.nLT;
        if(sync.n < TST_SIZE)
            sync.n++;
    //}

}

uint32_t calculateGT(uint32_t timeS){
    if(sync.mode == SYNC_MODE_MASTER)
        return timeS;
    else
        return (uint32_t)((int)timeS + sync.OFFSET + (int)(sync.SKEW * (double)((int)timeS - sync._LT)));
}
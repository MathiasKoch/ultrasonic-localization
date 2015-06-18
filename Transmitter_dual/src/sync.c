#include "sync.h"

void sync_init(uint8_t mode, uint8_t source){
	sync.mode = mode;
	sync.i = 1;
	sync.SCP_normal =  ((F_BUS / 0.5)-1);
    sync.SCP_fast = ((F_BUS / 2) - 1);

	sync_dma_init(source);
	sync_clock_init();

}

void pit2_isr(){
    PIT_TFLG1 = 1;
    if(sync.mode == SYNC_MODE_SLAVE){
        resetTimeSync();
        requestFastSync();
    }else{
        uint8_t data_out[RF_PACKET_SIZE];
        data_out[0] = 'r';
        nrf24_tx_address(broadCastAddress);
        nrf24_send(data_out);        
        while(nrf24_isSending());
        nrf24_powerUpRx();

        enableFastSync();
    }
    return;
}


void sync_dma_init(uint8_t source) {
    SIM_SCGC7 |= SIM_SCGC7_DMA;
    SIM_SCGC6 |= SIM_SCGC6_DMAMUX;

    DMA_CR = 0;

    DMA_TCD1_SADDR = &PIT_CVAL2;
    DMA_TCD1_SOFF = 0;
    DMA_TCD1_SLAST = 0;
    if(sync.mode == SYNC_MODE_MASTER)
    	DMA_TCD1_DADDR = &sync.nGT;
    else
    	DMA_TCD1_DADDR = &sync.nLT;
    DMA_TCD1_DOFF = 0;
    DMA_TCD1_DLASTSGA = 0;
    DMA_TCD1_ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2);
    DMA_TCD1_NBYTES_MLNO = 4;
    DMA_TCD1_CITER_ELINKNO = 0x0001;
    DMA_TCD1_BITER_ELINKNO = 0x0001;
    DMA_TCD1_CSR = DMA_TCD_CSR_INTMAJOR;

    DMAMUX0_CHCFG1 = DMAMUX_DISABLE;
    DMAMUX0_CHCFG1 = source | DMAMUX_ENABLE;

    DMA_SERQ = 1;

    NVIC_ENABLE_IRQ(IRQ_DMA_CH1);
}

void dma_ch1_isr(void){
    DMA_CINT = 1;
    if(sync.sent == 1){
        sync.sent = 0;
        // TODO: Look into radio busy using this interrupt
        sync.GTm = MAX_US - sync.nGT;
    }else{
        receivedRF = 1;
    }
}

void pit3_isr(){
	uint8_t data_out[RF_PACKET_SIZE];
    PIT_TFLG3 = 1;

    if(sync.fastSyncEnabled == 1){
        if(sync.fastSyncCounter++ > FAST_SYNC_COUNT)
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
    nrf24_tx_address(broadCastAddress);
    nrf24_send(data_out);        
    while(nrf24_isSending());
    nrf24_powerUpRx();
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
    uint8_t data_out[RF_PACKET_SIZE];
    data_out[0] = 'f';
    nrf24_tx_address(broadCastAddress);
    nrf24_send(data_out);        
    while(nrf24_isSending());
    nrf24_powerUpRx();
}

void resetTimeSync(){
    uint8_t count;
    for(count = 0; count < TST_SIZE; count++){
        sync.LT[count] = 0;
        sync.DIFF[count] = 0;
        sync.GT[count] = 0;
        sync.n = 0;
    }
}

void calcTimeSync(uint8_t * data){
    uint16_t n;
    uint32_t offset;
    uint32_t skew;
    uint64_t offset_var = 0;
    uint64_t skew_var = 0;
    uint64_t skew_var2 = 0;
    uint32_t GTx = 0;
    uint64_t lt_var = 0;

    sync.i = (data[1]<<8) + data[2]; // i starts from 1

    sync.im = (sync.i - 1) % TST_SIZE;
    sync.GT[sync.im] = (data[3]<<24) + (data[4]<<16) + (data[5]<<8) + data[6];
    sync.LT[sync.i%TST_SIZE] = MAX_US - sync.nLT;

    
    if(sync.i <= TST_SIZE)
        sync.n = sync.i;

    sync.DIFF[sync.im] = (sync.GT[sync.im] - sync.LT[sync.im]);
    // TODO: Check missing packet (i) and implement fast sync request
    if(sync.n > MIN_TST_ENTRIES){
        offset_var = 0;
        skew_var = 0;
        skew_var2 = 0;
        lt_var = 0;
        for(n = 0; n < sync.n; n++){
            lt_var += sync.LT[n];
            offset_var += sync.DIFF[n];
        }
        sync._LT = (uint32_t)(lt_var / sync.n);
        offset = (uint32_t)(offset_var / sync.n);
        for(n = 0; n < sync.n; n++){
            skew_var += (sync.LT[n] - sync._LT) * (sync.LT[n] - sync._LT);
            skew_var2 += (sync.LT[n] - sync._LT) * (sync.DIFF[n] - sync.OFFSET);
        }
        skew = (uint32_t)(skew_var2 / skew_var);
        GTx = (sync.LT[sync.im] + offset + skew * (sync.LT[sync.im] - sync._LT)) + SYNC_OFFSET;
        if(GTx > (sync.GT[sync.im] + MAX_AVG_DIFF) || GTx < (sync.GT[sync.im] - MAX_AVG_DIFF))
            sync.VALID[sync.im] = 0;
        else{
            sync.VALID[sync.im] = 1;
            sync.SKEW = skew;
            sync.OFFSET = offset;
        }
    }else{
        sync.VALID[sync.im] = 0;
    }
    //xprintf("Valid: %d, GT(%10ld), GTx(%10ld) \t diff: %d\r\n", sync.VALID[sync.im], sync.GT[sync.im], GTx, sync.GT[sync.im]-GTx);
}

uint32_t calculateGT(uint32_t timeS){
    return (timeS + sync.OFFSET + sync.SKEW * (timeS - sync._LT)) - SYNC_OFFSET;
}
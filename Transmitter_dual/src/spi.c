#include "spi.h"
#include "dac.h"
#include "adc.h"
#include "xprintf.h"


unsigned int spi_is_running(void){
  return SPI0_SR & SPI_SR_TXRXS;
}

void spi_dma_init_tx(){
  SIM_SCGC7 |= SIM_SCGC7_DMA;
  SIM_SCGC6 |= SIM_SCGC6_DMAMUX;


  DMA_TCD0_SADDR = dac_signal;  /* Set the Source Address */
    /* Destination address */
  DMA_TCD0_DADDR = &SPI0_PUSHR;
  DMA_TCD0_SOFF = 0x04;

  DMA_SERQ = 0x00;
    /* Source and Destination Modulo off, source and destination size 2 = 32 bits */
  DMA_TCD0_ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2);  
    /* Transfer 4 bytes per transaction */
  DMA_TCD0_NBYTES_MLNO = 0x04;
    /* No adjust needed */
  DMA_TCD0_SLAST = -sizeof(dac_signal);
    /* Destination offset disabled */
  DMA_TCD0_DOFF = 0x00;
  DMA_TCD0_CITER_ELINKNO = DAC_BUF_SIZE;
  DMA_TCD0_BITER_ELINKNO = DAC_BUF_SIZE;
    /* No adjustment to destination address */
  DMA_TCD0_DLASTSGA = 0x00;

  DMA_TCD0_CSR = DMA_TCD_CSR_DREQ;

  DMA_TCD0_CSR |= DMA_TCD_CSR_INTMAJOR;

  NVIC_ENABLE_IRQ(IRQ_DMA_CH0);

  DMAMUX0_CHCFG0 = DMAMUX_DISABLE;
  DMAMUX0_CHCFG0 = DMAMUX_SOURCE_ALWAYS0 |  DMAMUX_ENABLE;

}

void dma_ch0_isr(void){
  if(_user == PIT_USER_DAC){
    pit_run(0);
  }
  DMA_CINT = 0;
}

void spi_setup_master(void){
  /* configure inout ports */
  /* chapters 10, 11, 12 */


  /* enable module clocking */
  if ((SIM_SCGC6 & SIM_SCGC6_SPI0) == 0) 
    SIM_SCGC6 |= SIM_SCGC6_SPI0;


  PORTC_PCR4 = PORT_PCR_MUX(2) | PORT_PCR_SRE | PORT_PCR_DSE; // CSn nRF24

  PORTD_PCR6 = PORT_PCR_MUX(2) | PORT_PCR_SRE | PORT_PCR_DSE; // CSn DAC

  PORTC_PCR3 |= PORT_PCR_MUX(1); // CE output
  GPIOC_PDDR |= _BV(3); 

  /* PTC5, SPI0_SCK, ALT2 */
  PORTC_PCR5 = PORT_PCR_MUX(2) | PORT_PCR_SRE; // nRF24
  PORTD_PCR1 = PORT_PCR_MUX(2) | PORT_PCR_SRE; // DAC

  /* PTC6, SPI0_SOUT, ALT2, internall pullup */
  PORTC_PCR6 = PORT_PCR_MUX(2) | PORT_PCR_SRE | PORT_PCR_DSE; // nRF24
  PORTD_PCR2 = PORT_PCR_MUX(2) | PORT_PCR_SRE | PORT_PCR_DSE; // DAC
  /* PTC7, SPI0_SIN, ALT2 */
  PORTC_PCR7 = PORT_PCR_MUX(2) | PORT_PCR_PE; // nRF24
  
  SPI0_MCR = SPI_MCR_HALT;

  SPI0_MCR |= SPI_MCR_MSTR | SPI_MCR_DIS_TXF | SPI_MCR_DIS_RXF | SPI_MCR_PCSIS(0x01) | SPI_MCR_PCSIS(0x08);

  SPI0_CTAR1 = SPI_CTAR_FMSZ(7) | SPI_CTAR_PBR(0) | SPI_CTAR_BR(4) | SPI_CTAR_ASC(3); // 1.5 MHz clk
  SPI0_CTAR0 = SPI_CTAR_FMSZ(0xF) | SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_ASC(0) | SPI_CTAR_CPHA; // 12 MHz clk

  SPI0_RSER = 0;

  /* enable DSPI clocks */
  SPI0_MCR &= ~SPI_MCR_HALT;
}

uint8_t spi_write_uint8(uint8_t x, uint8_t eoq){
  /* SPI0_PUSHR */
  /* CONT: PCS to inactive between transfers */
  /* CTAR: TAR0 selected */
  /* EOQ: last data */
  /* CTCNT: do not clear counter */
  /* PCS: select cs0 (not applicable if hand controlled) */
  /* TXDATA: data */
  if(eoq == 1)
    SPI0_PUSHR = (x | SPI_PUSHR_PCS(0x01) | SPI_PUSHR_CTAS(1));
  else
    SPI0_PUSHR = (x | SPI_PUSHR_PCS(0x01) | SPI_PUSHR_CONT | SPI_PUSHR_CTAS(1));
  while (((SPI0_SR >> 4) & 7) == 0);
  return SPI0_POPR;
}

void spi_write_uint16(uint16_t x){
    SPI0_PUSHR = (x | SPI_PUSHR_PCS(0x08) | SPI_PUSHR_CTAS(0));
    while (((SPI0_SR >> 4) & 7) == 0);
    SPI0_POPR;
}


void spi_ce(uint8_t state){
    if(state)
        GPIOC_PDOR |= _BV(3);
    else
        GPIOC_PDOR &= ~_BV(3);
}
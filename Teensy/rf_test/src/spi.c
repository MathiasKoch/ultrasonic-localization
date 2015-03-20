#include <stdint.h>
#include <avr/io.h>
#include "mk20dx128.h"


void spi_halt(void){
  SPI0_MCR |= SPI_MCR_HALT;
}

void spi_start(void){
  SPI0_MCR &= ~SPI_MCR_HALT;
}

unsigned int spi_is_running(void){
  return SPI0_SR & SPI_SR_TXRXS;
}

void spi_setup_master(void){
  /* configure inout ports */
  /* chapters 10, 11, 12 */

  /* enable module clocking */
  if ((SIM_SCGC6 & SIM_SCGC6_SPI0) == 0) 
    SIM_SCGC6 |= SIM_SCGC6_SPI0;

  /* PORTC, slave select pin */

  PORTC_PCR4 = PORT_PCR_MUX(1) | PORT_PCR_DSE | PORT_PCR_SRE;

  GPIOC_PDDR |= _BV(4);
  GPIOC_PDOR |= _BV(4);

  /* PTC5, SPI0_SCK, ALT2 */
  PORTC_PCR5 = PORT_PCR_MUX(2);

  /* PTC6, SPI0_SOUT, ALT2, internall pullup */
  PORTC_PCR6 = PORT_PCR_MUX(2) | PORT_PCR_PE;
  /* PTC7, SPI0_SIN, ALT2 */
  PORTC_PCR7 = PORT_PCR_MUX(2);

  /* SPI0_MCR, module configuration register */
  /* MSTR: enable master */
  /* CONT_SCKE: continuous SCK disabled */
  /* DCONF: SPI mode */
  /* FRZ: freeze disabled */
  /* MTFE: modified SPI transfer format disabled */
  /* ROOE: ignore incoming data on overflow */
  /* PCSIS: active low */
  /* DOZE: disabled */
  /* MDIS: DSPI clocks disabled */
  /* DIS_TXF: transmit fifo disabled */
  /* DIS_RXF: receive fifo disabled */

  /* disable MDIS first to disable FIFOs */
  SPI0_MCR = SPI_MCR_MDIS;

  SPI0_MCR = SPI_MCR_MSTR | SPI_MCR_MDIS | SPI_MCR_DIS_TXF | SPI_MCR_DIS_RXF | SPI_MCR_HALT;
  while (spi_is_running());

  /* SPI0_CTAR0, clock and transfer attributes register */
  /* DBR: double baud rate, 50 / 50 */
  /* FMSZ: frame size set to 7 + 1 */
  /* CPOL: inactive when sck low */
  /* CPHA: data sampled on leading edge */
  /* LSBFE: MSB first */
  /* PCSSCK: PCS activation prescaler set to 1 */
  /* PASC: PCS deactivation prescaler set to 1 */
  /* PDT: delay after transfer prescaler set to 1 */
  /* PBR: baud rate prescaler set to 1 */
  /* CSSCK: PCS to SCK delay scaler set to 2 */
  /* ASC: after delay scaler set to 0 */
  /* DT: delay after transfer scaler set to 0 */
  /* BR: baud rate scaler set to 128 (ie. 375kHz) */
  SPI0_CTAR0 = SPI_CTAR_FMSZ(7) | SPI_CTAR_BR(8);

  /* SPI0_RSER, DMA and interrupt request select and enable register */
  /* all disabled */
  SPI0_RSER = 0;

  /* enable DSPI clocks */
  SPI0_MCR &= ~(SPI_MCR_MDIS | SPI_MCR_HALT);
}

uint8_t spi_write_uint8(uint8_t x)
{
  /* tx fifo disabled */

  /* SPI0_PUSHR */
  /* CONT: PCS to inactive between transfers */
  /* CTAR: TAR0 selected */
  /* EOQ: last data */
  /* CTCNT: do not clear counter */
  /* PCS: select cs0 (not applicable if hand controlled) */
  /* TXDATA: data */

  SPI0_PUSHR = x;
  while (((SPI0_SR >> 4) & 7) == 0) ;
  return SPI0_POPR;
}

void spi_csn(uint8_t state)
{
    if(state)
        GPIOC_PDOR |= _BV(3);
    else
        GPIOC_PDOR &= ~_BV(3);
}

void spi_ce(uint8_t state)
{
    if(state)
        GPIOC_PDOR |= _BV(4);
    else
        GPIOC_PDOR &= ~_BV(4);
}

uint8_t spi_read_uint8(void)
{
  /* rx fifo disabled */
  spi_write_uint8(0xff);
  while (((SPI0_SR >> 4) & 7) == 0) ;
  return SPI0_POPR;
}
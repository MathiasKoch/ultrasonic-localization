/*
* ----------------------------------------------------------------------------
* “THE COFFEEWARE LICENSE” (Revision 1):
* <ihsan@kehribar.me> wrote this file. As long as you retain this notice you
* can do whatever you want with this stuff. If we meet some day, and you think
* this stuff is worth it, you can buy me a coffee in return.
* -----------------------------------------------------------------------------
* This library is based on this library: 
*   https://github.com/aaronds/arduino-nrf24l01
* Which is based on this library: 
*   http://www.tinkerer.eu/AVRLib/nRF24L01
* -----------------------------------------------------------------------------
*/
#include <avr/io.h>
#include "nrf24.h"
#include "spi.h"
#include "xprintf.h"


uint8_t payload_len;


/* init the hardware pins */
void nrf24_init(){
    xprintf("Initializing RF module\t\t - \t");
    spi_setup_master();
    spi_ce(LOW);
    xprintf("Done\r\n");
}

/* configure the module */
void nrf24_config(uint8_t channel, uint8_t pay_length){
    /* Use static payload length ... */
    payload_len = pay_length;

    // Set RF channel
    nrf24_configRegister(RF_CH,channel);

    // Set length of incoming payload 
    nrf24_configRegister(RX_PW_P0, 0x00); // Auto-ACK pipe ...
    nrf24_configRegister(RX_PW_P1, payload_len); // Data payload pipe
    nrf24_configRegister(RX_PW_P2, payload_len); // Broadcast pipe
    nrf24_configRegister(RX_PW_P3, 0x00); // Pipe not used 
    nrf24_configRegister(RX_PW_P4, 0x00); // Pipe not used 
    nrf24_configRegister(RX_PW_P5, 0x00); // Pipe not used 

    // 1 Mbps, TX gain: 0dbm
    nrf24_configRegister(RF_SETUP, (0<<RF_DR)|((0x03)<<RF_PWR));

    // CRC enable, 1 byte CRC length
    nrf24_configRegister(CONFIG,nrf24_CONFIG);

    // Auto Acknowledgment
    nrf24_configRegister(EN_AA,(1<<ENAA_P0)|(1<<ENAA_P1)|(1<<ENAA_P2)|(0<<ENAA_P3)|(0<<ENAA_P4)|(0<<ENAA_P5));

    // Enable RX addresses
    nrf24_configRegister(EN_RXADDR,(1<<ERX_P0)|(1<<ERX_P1)|(1<<ERX_P2)|(0<<ERX_P3)|(0<<ERX_P4)|(0<<ERX_P5));

    // Auto retransmit delay: 250 us and Up to 5 retransmit trials
    nrf24_configRegister(SETUP_RETR,(0x00<<ARD)|(0x05<<ARC));

    // Dynamic length configurations: No dynamic length
    nrf24_configRegister(DYNPD,(0<<DPL_P0)|(0<<DPL_P1)|(0<<DPL_P2)|(0<<DPL_P3)|(0<<DPL_P4)|(0<<DPL_P5));

    // Start listening
    nrf24_powerUpRx();
}

/* Set the RX address */
void nrf24_rx_address(uint8_t * adr){
    spi_ce(LOW);
    nrf24_writeRegister(RX_ADDR_P1,adr,nrf24_ADDR_LEN);
    spi_ce(HIGH);
}

void nrf24_rx_broadcast_address(uint8_t * adr){
    spi_ce(LOW);
    nrf24_writeRegister(RX_ADDR_P2,adr,nrf24_ADDR_LEN);
    spi_ce(HIGH);
}

/* Returns the payload length */
uint8_t nrf24_payload_length(){
    return payload_len;
}

/* Set the TX address */
void nrf24_tx_address(uint8_t* adr){
    /* RX_ADDR_P0 must be set to the sending addr for auto ack to work. */
    nrf24_writeRegister(RX_ADDR_P0,adr,nrf24_ADDR_LEN);
    nrf24_writeRegister(TX_ADDR,adr,nrf24_ADDR_LEN);
}

/* Checks if data is available for reading */
/* Returns 1 if data is ready ... */
uint8_t nrf24_dataReady(){
    if(nrf24_getStatus() & (1 << RX_DR))
        return 1;
    return !nrf24_rxFifoEmpty();
}

/* Checks if receive FIFO is empty or not */
uint8_t nrf24_rxFifoEmpty(){
    uint8_t fifoStatus;
    nrf24_readRegister(FIFO_STATUS,&fifoStatus,1);
    return (fifoStatus & (1 << RX_EMPTY));
}

/* Returns the length of data waiting in the RX fifo */
uint8_t nrf24_payloadLength(){
    spi_write_uint8(R_RX_PL_WID, 0);
    return spi_write_uint8(0x00, 1);
}

/* Reads payload bytes into data array */
void nrf24_getData(uint8_t* data){
    spi_write_uint8( R_RX_PAYLOAD, 0);
    nrf24_transferSync(data,data,payload_len);
    nrf24_configRegister(STATUS,(1<<RX_DR));   
}

/* Returns the number of retransmissions occured for the last message */
uint8_t nrf24_retransmissionCount(void){
    uint8_t rv;
    nrf24_readRegister(OBSERVE_TX,&rv,1);
    return (rv & 0x0F);
}

// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
void nrf24_send(uint8_t* value){    
    spi_ce(LOW);
    nrf24_powerUpTx();
    /* Do we really need to flush TX fifo each time ? */
    #if 1
        spi_write_uint8(FLUSH_TX, 1);
    #endif 
    spi_write_uint8(W_TX_PAYLOAD, 0);
    nrf24_transmitSync(value,payload_len);
    spi_ce(HIGH);
}

uint8_t nrf24_isSending(){
    return !((nrf24_getStatus() & ((1 << TX_DS)  | (1 << MAX_RT))));
}

uint8_t nrf24_getStatus(){
    return spi_write_uint8(NOP, 1);
}

uint8_t nrf24_lastMessageStatus(){
    uint8_t rv = nrf24_getStatus();
    if((rv & ((1 << MAX_RT))))
        return NRF24_MESSAGE_LOST;
    else if((rv & ((1 << TX_DS))))
        return NRF24_TRANSMISSON_OK;
    else
        return 0xFF;
}

void nrf24_powerUpRx(){     
    spi_ce(LOW);    
    spi_write_uint8(FLUSH_RX, 1);
    spi_write_uint8(FLUSH_TX, 1);
    nrf24_configRegister(STATUS,(1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT)); 
    nrf24_configRegister(CONFIG,nrf24_CONFIG|((1<<PWR_UP)|(1<<PRIM_RX)));    
    spi_ce(HIGH);
}

void nrf24_powerUpTx(){
    nrf24_configRegister(STATUS,(1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT)); 
    nrf24_configRegister(CONFIG,nrf24_CONFIG|((1<<PWR_UP)|(0<<PRIM_RX)));
}

void nrf24_powerDown(){
    spi_ce(LOW);
    nrf24_configRegister(CONFIG,nrf24_CONFIG);
}

/* send and receive multiple bytes over SPI */
void nrf24_transferSync(uint8_t* dataout, uint8_t* datain, uint8_t len){
    uint8_t i;
    for(i=0;i<len;i++){
        if(i < len-1)
            datain[i] = spi_write_uint8(dataout[i], 0);
        else
            datain[i] = spi_write_uint8(dataout[i], 1);
    }
}

/* send multiple bytes over SPI */
void nrf24_transmitSync(uint8_t* dataout, uint8_t len){
    uint8_t i;
    for(i=0;i<len;i++){
        if(i < len-1)
            spi_write_uint8(dataout[i], 0);
        else
            spi_write_uint8(dataout[i], 1);
    }
}

/* Clocks only one byte into the given nrf24 register */
void nrf24_configRegister(uint8_t reg, uint8_t value){
    spi_write_uint8(W_REGISTER | (REGISTER_MASK & reg), 0);
    spi_write_uint8(value, 1);
}

/* Read single register from nrf24 */
void nrf24_readRegister(uint8_t reg, uint8_t* value, uint8_t len){
    spi_write_uint8(R_REGISTER | (REGISTER_MASK & reg), 0);
    nrf24_transferSync(value,value,len);
}

/* Write to a single register of nrf24 */
void nrf24_writeRegister(uint8_t reg, uint8_t* value, uint8_t len){
    spi_write_uint8(W_REGISTER | (REGISTER_MASK & reg), 0);
    nrf24_transmitSync(value,len);
}

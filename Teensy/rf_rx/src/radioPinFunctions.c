/*
* ----------------------------------------------------------------------------
* “THE COFFEEWARE LICENSE” (Revision 1):
* <ihsan@kehribar.me> wrote this file. As long as you retain this notice you
* can do whatever you want with this stuff. If we meet some day, and you think
* this stuff is worth it, you can buy me a coffee in return.
* -----------------------------------------------------------------------------
* Please define your platform spesific functions in this file ...
* -----------------------------------------------------------------------------
*/

#include <avr/io.h>

/* ------------------------------------------------------------------------- */
void nrf24_setupPins()
{
    SIM_SCGC5 |= SIM_SCGC5_PORTC;
    PORTC_PCR3 |= PORT_PCR_MUX(0x01); // CE output
    GPIOC_PDDR |= _BV(3);
    PORTC_PCR4 |= PORT_PCR_MUX(0x01); // CSN output
    GPIOC_PDDR |= _BV(4);
    PORTC_PCR5 |= PORT_PCR_MUX(0x01); // SCK output
    GPIOC_PDDR |= _BV(5);
    PORTC_PCR7 |= PORT_PCR_MUX(0x01); // MOSI output
    GPIOC_PDDR |= _BV(7);
    PORTC_PCR6 |= PORT_PCR_MUX(0x01); // MISO input
    GPIOC_PDDR &= ~_BV(6);
}
/* ------------------------------------------------------------------------- */
void nrf24_ce_digitalWrite(uint8_t state)
{
    if(state)
    {
        GPIOC_PDOR |= _BV(3);
    }
    else
    {
        GPIOC_PDOR &= ~_BV(3);
    }
}
/* ------------------------------------------------------------------------- */
void nrf24_csn_digitalWrite(uint8_t state)
{
    if(state)
    {
        GPIOC_PDOR |= _BV(4);
    }
    else
    {
        GPIOC_PDOR &= ~_BV(4);
    }
}
/* ------------------------------------------------------------------------- */
void nrf24_sck_digitalWrite(uint8_t state)
{
    if(state)
    {
        GPIOC_PDOR |= _BV(5);
    }
    else
    {
        GPIOC_PDOR &= ~_BV(5);
    }
}
/* ------------------------------------------------------------------------- */
void nrf24_mosi_digitalWrite(uint8_t state)
{
    if(state)
    {
        GPIOC_PDOR |= _BV(7);
    }
    else
    {
        GPIOC_PDOR &= ~_BV(7);
    }
}
/* ------------------------------------------------------------------------- */
uint8_t nrf24_miso_digitalRead()
{
    return (GPIOC_PDIR & _BV(6));
}
/* ------------------------------------------------------------------------- */

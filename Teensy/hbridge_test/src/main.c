#include <avr/io.h>
#define ARM_MATH_CM4
#include <arm_math.h>
//#include <common.h>
#include <util/delay.h>
#include <usb_serial.h>
#include <stdio.h>

#include "xprintf.h"

#define PDB_CH0C1_TOS 0x0100
#define PDB_CH0C1_EN 0x01

#define PWM 6
#define HBRO_E 8
#define MUX_1 17
#define MUX_2 16
#define MUX_TRANSMIT 1
#define MUX_RECEIVE 2

void change_mux(int type){
    if(type == MUX_TRANSMIT){
        digitalWrite(MUX_1, HIGH);
        digitalWrite(MUX_2, LOW);
    }else if(type == MUX_RECEIVE){

    }
}

#define PDB_CONFIG (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN \
    | PDB_SC_CONT | PDB_SC_PRESCALER(0) | PDB_SC_MULT(0) | PDB_SC_PDBIE)

#define PDB_PERIOD ((F_BUS / 2 / 40000)-1)

void pdbInit() {
    // Enable PDB clock
    SIM_SCGC6 |= SIM_SCGC6_PDB;
    // Timer period
    PDB0_MOD = PDB_PERIOD;
    // Interrupt delay
    PDB0_IDLY = 0;
    // Enable pre-trigger
    PDB0_CH0C1 = PDB_CH0C1_TOS | PDB_CH0C1_EN;
    // PDB0_CH0DLY0 = 0;
    PDB0_SC = PDB_CONFIG | PDB_SC_LDOK;
    // Software trigger (reset and restart counter)
    PDB0_SC |= PDB_SC_SWTRIG;

    NVIC_ENABLE_IRQ(IRQ_PDB);
}


void pdb_isr(void){
    digitalWrite(PWM, !digitalRead(PWM));
    PDB0_SC &= ~0x40u;  // clear interrupt mask
    return;
}


int main()
{

    /* init the xprintf library */

SIM_CLKDIV1 |= SIM_CLKDIV1_OUTDIV2(1);

    xdev_out(usb_serial_putchar);
    pinMode(HBRO_E, OUTPUT);
    pinMode(PWM, OUTPUT);
    pinMode(MUX_2, OUTPUT);
    pinMode(MUX_1, OUTPUT);

    pdbInit();
    change_mux(MUX_TRANSMIT);

    analogWriteResolution(10);
   // analogWriteFrequency(HBRO_E, 25000);
    digitalWrite(PWM, HIGH);

    xprintf("\r\n> Device setup to transmit\r\n");
    digitalWrite(HBRO_E, HIGH);

    while(1)
    {    
    
    }
}
/* ------------------------------------------------------------------------- */
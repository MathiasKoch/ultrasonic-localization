#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "neopixel.h"


DMAMEM int displayMemory[16*6];
int drawingMemory[16*6];

int main(){
    //SIM_CLKDIV1 |= SIM_CLKDIV1_OUTDIV2(1);

    neo_init(16, displayMemory, drawingMemory);
    neo_show();
    int i = 0;
    while(1){    
        int x = 15;
        for(i = 0; i < 16; i++){
            neo_setPixel(i, 0x0000FF);
            if(i>0)
                neo_setPixel(i-1, 0x000000);
            if(i==0)
                neo_setPixel(15, 0x000000);
            neo_setPixel(x, 0xFF0000);
            if(x==15)
                neo_setPixel(0, 0x000000);
            if(x<15)
                neo_setPixel(x+1, 0x000000);

            neo_show();
              x--;
            if(x==0)
                x = 15;
            delay(50);
        }
    }
}
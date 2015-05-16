#include "WProgram.h"

int main(void)
{
	//float phase = 0.0;
	//float twopi = 3.14159 * 2;
    pinMode(A14, OUTPUT);
    analogWriteResolution(12);
    while (1) {
        float val = 0.26;
		analogWrite(A14, (int)(val*2048.0));
		//phase = phase + 0.02;
		//if (phase >= twopi) phase = 0;
		//_delay_us(500);
    }
}
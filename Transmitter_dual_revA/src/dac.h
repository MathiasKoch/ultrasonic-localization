#ifndef DAC
#define DAC

#define LOW 0
#define HIGH 1

#define SWITCH_1 0
#define SWITCH_2 1
#define SWITCH_3 2
#define SWITCH_4 3
#define SWITCH_ALL 4

void dac_init();
void switch_init();
void switch_set(uint8_t sw, uint8_t enable);


#endif
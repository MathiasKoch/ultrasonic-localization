#ifndef GPIO
#define GPIO

#include <avr/io.h>
#include "calibrate.h"

volatile uint8_t dip1, dip2, dip3;

void gpio_init();
void portd_isr(void);

#endif
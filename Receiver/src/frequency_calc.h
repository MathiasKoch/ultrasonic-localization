#ifndef FREQUENCY_CALC
#define FREQUENCY_CALC

#include <arm_math.h>

void frequency_calc(q15_t * cmplxInp, q15_t * realOut, uint32_t blockSize);

#endif /* FREQUENCY_CALC */
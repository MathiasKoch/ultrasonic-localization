#ifndef FREQUENCY_CALC
#define FREQUENCY_CALC

#include <arm_math.h>

void atan2_fp(q15_t * cmplxInp, q15_t * realOut, uint32_t blockSize);
uint32_t isqrt32 (uint32_t n);

#endif /* FREQUENCY_CALC */
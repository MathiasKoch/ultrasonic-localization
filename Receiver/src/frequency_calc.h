#ifndef FREQUENCY_CALC
#define FREQUENCY_CALC

#include <arm_math.h>

void atan2_fp(q15_t * cmplxInp, q15_t * realOut, uint32_t blockSize);
void unwrap_fp(q15_t * pSrc, q15_t * pDst, uint32_t blockSize);
uint32_t change_detect(q15_t * pSrc, uint32_t blockSize);

#endif /* FREQUENCY_CALC */
#define ARM_MATH_CM4
#include <arm_math.h>
#include "frequency_calc.h"

void frequency_calc(q15_t * cmplxInp, q15_t * realOut, uint32_t blockSize)
{
	// Q1.15 format
	uint32_t blkCnt;     			/* loop counter */
	q15_t coeff_1 = 0;				// <--- Fix coeffs!
	q15_t coeff_1b = 0;
	q15_t coeff_1c = 0;

#ifndef ARM_MATH_CM0

	q31_t inA1, inA2;
  	q31_t inB1, inB2;
  	q31_t inA2_abs, inB2_abs;
  	q31_t r, r3;
  	q15_t outA, outB;

  	/*loop Unrolling */
	blkCnt = blockSize >> 2u;

	/* First part of the processing with loop unrolling.  Compute 4 outputs at a time.    
	** a second loop below computes the remaining 1 to 3 samples. */
	while(blkCnt > 0u)
	{
		inA1 = *__SIMD32(cmplxInp)++;
		inA2 = *__SIMD32(cmplxInp)++;
		inB1 = *__SIMD32(cmplxInp)++;
		inB2 = *__SIMD32(cmplxInp)++;


		inA2_abs = inA2 < 0? __QSUB16(0,inA2): inA2;
		inB2_abs = inB2 < 0? __QSUB16(0,inB2): inB2;

		if(inA2 == 0)
			outA = inA1 >= 0? 0: 180;				// <--- FP this
		else if(inA1 >= 0)
			r = ( __QSUB16(inA1,inA2_abs) << 15 ) / __QADD16(inA1,inA2_abs);
		else
			r = ( __QADD16(inA1,inA2_abs) << 15 ) / __QSUB16(inA1,inA2_abs);

		r3 =  ((((r * r) >> 15) * r) >> 15) * coeff_1c;

		outA = (q15_t) __SSAT(__QADD16(coeff_1b * r,r3)>>15, 16);
		if(inA1 < 0)
			outA = __QADD16(coeff_1*3, outA);
		else
			outA = __QADD16(coeff_1, outA);
		if(inA2 < 0)
			outA = __QSUB16(0,outA);


		if(inB2 == 0)
			outB = inB1 >= 0? 0: 180;				// <--- FP this
		else if(inB1 >= 0)
			r = ( ( (q31_t) __QSUB16(inB1,inB2_abs) ) << 15 ) / ( (q31_t) __QADD16(inB1,inB2_abs) );
		else
			r = ( ( (q31_t) __QADD16(inB1,inB2_abs) ) << 15 ) / ( (q31_t) __QSUB16(inB1,inB2_abs) );

		r3 =  ((((r * r) >> 15) * r) >> 15) * coeff_1c;

		outB = (q15_t) __SSAT(__QADD16(coeff_1b * r,r3)>>15, 16);
		if(inB1 < 0)
			outB = __QADD16(coeff_1*3, outB);
		else
			outB = __QADD16(coeff_1, outB);

		if(inB2 < 0)
			outB = __QSUB16(0,outB);


		

		*__SIMD32(realOut)++ = outA;
		*__SIMD32(realOut)++ = outB;

		/* Decrement the loop counter */
		blkCnt--;
	}

	/* If the blockSize is not a multiple of 4, compute any remaining output samples here.    
	** No loop unrolling is used. */
	blkCnt = blockSize % 0x4u;

	while(blkCnt > 0u)
	{
		inA1 = *__SIMD32(cmplxInp)++;
		inA2 = *__SIMD32(cmplxInp)++;

		inA2_abs = inA2 < 0? __QSUB16(0,inA2): inA2;

		if(inA2 == 0)
			outA = inA1 >= 0? 0: 180;				// <--- FP this
		else if(inA1 >= 0)
			r = ( __QSUB16(inA1,inA2_abs) << 15 ) / __QADD16(inA1,inA2_abs);
		else
			r = ( __QADD16(inA1,inA2_abs) << 15 ) / __QSUB16(inA1,inA2_abs);

		r3 =  ((((r * r) >> 15) * r) >> 15) * coeff_1c;

		outA = (q15_t) __SSAT(__QADD16(coeff_1b * r,r3)>>15, 16);
		if(inA1 < 0)
			outA = __QADD16(coeff_1*3, outA);
		else
			outA = __QADD16(coeff_1, outA);
		if(inA2 < 0)
			outA = __QSUB16(0,outA);

		*__SIMD32(realOut)++ = outA;

		/* Decrement the loop counter */
		blkCnt--;
	}




#else

  /* Run the below code for Cortex-M0 ... NOT APPLICABLE NOW!*/

#endif /* #ifndef ARM_MATH_CM0 */

	/*int32_t coeff_1 = 45;
	int32_t coeff_1b = -56;	// 56.24;
	int32_t coeff_1c = 11;	// 11.25
	int16_t coeff_2 = 135;

	int16_t angle = 0;

	int32_t r;
	int32_t r3;

	int16_t y_abs_fp = y_fp;
	if (y_abs_fp < 0)
		y_abs_fp = -y_abs_fp;

	if (y_fp == 0)
	{
		if (x_fp >= 0)
		{
			angle = 0;
		}
		else
		{
			angle = 180;
		}
	}
	else if (x_fp >= 0)
	{
		r = (((int32_t)(x_fp - y_abs_fp)) << MULTIPLY_FP_RESOLUTION_BITS) / ((int32_t)(x_fp + y_abs_fp));

		r3 = r * r;
		r3 =  r3 >> MULTIPLY_FP_RESOLUTION_BITS;
		r3 *= r;
		r3 =  r3 >> MULTIPLY_FP_RESOLUTION_BITS;
		r3 *= coeff_1c;
		angle = (int16_t) (  	coeff_1 + ((coeff_1b * r + r3) >> MULTIPLY_FP_RESOLUTION_BITS)   );
	}
	else
	{
		r = (((int32_t)(x_fp + y_abs_fp)) << MULTIPLY_FP_RESOLUTION_BITS) /
((int32_t)(y_abs_fp - x_fp));
		r3 = r * r;
		r3 =  r3 >> MULTIPLY_FP_RESOLUTION_BITS;
		r3 *= r;
		r3 =  r3 >> MULTIPLY_FP_RESOLUTION_BITS;
		r3 *= coeff_1c;
		angle = coeff_2 + ((int16_t)	(((coeff_1b * r + r3) >>
MULTIPLY_FP_RESOLUTION_BITS))	);
	}

	if (y_fp < 0)
		return (-angle);     // negate if in quad III or IV
	else
		return (angle);*/
}
#include <arm_math.h>
#include "frequency_calc.h"
#include "xprintf.h"

#define FIX_PI 201
#define SHIFT 15

void atan2_fp(q15_t * cmplxInp, q15_t * realOut, uint32_t blockSize)
{
	// Q1.15 format
	uint32_t blkCnt;     			/* loop counter */
	q15_t coeff_1 = 1608;		//  50		
	q15_t coeff_1b = -63;	// -63
	q15_t coeff_1c = 13;	//  13
	q15_t coeff_2 = 1608*3;	//  150

	q15_t inA1, inA2;
  	q15_t inB1, inB2;
  	q15_t inA2_abs, inB2_abs;
  	q31_t r, r3, r32;
  	q15_t outA, outB;

  	/*loop Unrolling */
	blkCnt = blockSize >> 2u;

	/* First part of the processing with loop unrolling.  Compute 4 outputs at a time.    
	** a second loop below computes the remaining 1 to 3 samples. */
	while(blkCnt > 0u)
	{
		r = 0;
		r32 = 0;
		inA1 = *cmplxInp++;
		inA2 = *cmplxInp++;
		inB1 = *cmplxInp++;
		inB2 = *cmplxInp++;
		outA = 0;
		outB = 0;

		inA2_abs = inA2 >= 0? inA2 : __QSUB16(0,inA2);
		inB2_abs = inB2 >= 0? inB2 : __QSUB16(0,inB2);



		/*if(inA2 == 0)
			outA = inA1 >= 0? 0: FIX_PI;				
		else*/ if(inA1 >= 0){
			r = (((q31_t) (inA1 - inA2_abs))<<15) / ((q31_t) inA1 + inA2_abs);
		}else{
			r = (((q31_t) (inA1 + inA2_abs))<<15) / ((q31_t) inA2_abs - inA1);
		}


		
		r3 =  ((((r * r) >> SHIFT) * r) >> SHIFT) * coeff_1c;
		r32 = (coeff_1b * r);
		outA = (q15_t) __SSAT((r32 + r3)>>10,16); 
		
		if(inA1 >= 0){
			outA = coeff_1 + outA;
		}else{
			outA = coeff_2 + outA;
		}


		if(inA2 < 0)
			outA = -outA;

	
		r = 0;
		r32 = 0;
		/*if(inB2 == 0)
			outB = inB1 >= 0? 0: FIX_PI;			
		else*/ if(inB1 >= 0){
			r = (((q31_t) (inB1 - inB2_abs))<<15) / ((q31_t) inB1 + inB2_abs);
		}else{
			r = (((q31_t) (inB1 + inB2_abs))<<15) / ((q31_t) inB2_abs - inB1); // Q17.15
		}

		r3 =  ((((r * r) >> SHIFT) * r) >> SHIFT) * coeff_1c; // Q11.21
		r32 = (coeff_1b * r); // Q11.21
		outB = (q15_t) __SSAT((r32 + r3)>>10,16); // Q5.11
		
		
		if(inB1 >= 0){
			outB = coeff_1 + outB;
		}else{
			outB = coeff_2 + outB;
		}



		if(inB2 < 0)
			outB = -outB;
		
		*realOut++ = outA;
		*realOut++ = outB;

		/* Decrement the loop counter */
		blkCnt--;
	}

	/* If the blockSize is not a multiple of 4, compute any remaining output samples here.    
	** No loop unrolling is used. */
	/*blkCnt = blockSize % 0x4u;

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

		blkCnt--;
	}*/




}

#define PI2 201
#define THRESH 150

void unwrap_fp(q15_t * pSrc, q15_t * pDst, uint32_t blockSize){

	// Q1.15 format
	uint32_t blkCnt;     			/* loop counter */

	q15_t inA1, inA2, out;

	uint16_t i = 0;

  	/*loop Unrolling */
	blkCnt = blockSize;

	/* First part of the processing with loop unrolling.  Compute 4 outputs at a time.    
	** a second loop below computes the remaining 1 to 3 samples. */
	*pDst++ = *pSrc;

	while(blkCnt > 0u)
	{
		inA1 = *pSrc++;
		inA2 = *pSrc;

		out = inA2;

		if(inA2 > (inA1 + THRESH))
			out -= (PI2 * ++i);
		else if(inA2 < (inA1 - THRESH))
			out += (PI2 * ++i);
		else
			out += (PI2 * i);
		*pDst++ = out;
		blkCnt--;
	}
}

#define SEPERATION 16
#define BURN 25
#define INDEX_CORRECTION 3

uint32_t change_detect(q15_t * pSrc, uint32_t blockSize){
	uint32_t blkCnt;     			/* loop counter */
	uint16_t index = 0;
	q15_t inA1, inA2, diff, temp;

  	/*loop Unrolling */
	blkCnt = (SEPERATION + BURN);
	diff = 0;

	/* First part of the processing with loop unrolling.  Compute 4 outputs at a time.    
	** a second loop below computes the remaining 1 to 3 samples. */
	pSrc+=(SEPERATION + BURN);

	while(blkCnt < (blockSize-BURN)){

		inA1 = *pSrc;
		inA2 = *(pSrc++ - SEPERATION);
		temp = inA1 - inA2;
		if(temp > diff){
			index = blkCnt;
			diff = temp;
		}
		blkCnt++;
	}
	return index-INDEX_CORRECTION;
}
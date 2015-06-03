/*------------------------------------------------------------------------*/
/* Universal string handler for user console interface  (C)ChaN, 2011     */
/*------------------------------------------------------------------------*/

#ifndef _STRFUNC
#define _STRFUNC

#define _USE_XFUNC_OUT	1	/* 1: Use output functions */
#define	_CR_CRLF		0	/* 1: Convert \n ==> \r\n in the output char */

#define _USE_XFUNC_IN	0	/* 1: Use input function */
#define	_LINE_ECHO		0	/* 1: Echo back input chars in xgets function */


#if _USE_XFUNC_OUT
#define xdev_out(func) xfunc_out = (void(*)(unsigned char))(func)
extern void (*xfunc_out)(unsigned char);
void xputc (char c);
void xputs (const char* str);
void xfputs (void (*func)(unsigned char), const char* str);
void xprintf (const char* fmt, ...);
void xsprintf (char* buff, const char* fmt, ...);
void xfprintf (void (*func)(unsigned char), const char*	fmt, ...);
void put_dump (const void* buff, unsigned long addr, int len, int width);
#define DW_CHAR		sizeof(char)
#define DW_SHORT	sizeof(short)
#define DW_LONG		sizeof(long)
#endif

#if _USE_XFUNC_IN
#define xdev_in(func) xfunc_in = (unsigned char(*)(void))(func)
extern unsigned char (*xfunc_in)(void);
int xgets (char* buff, int len);
int xfgets (unsigned char (*func)(void), char* buff, int len);
int xatoi (char** str, long* res);
#endif



// ADCx_SC1n
#define COCO 7
#define AIEN 6
#define DIFF 5
#define ADCH4 4
#define ADCH3 3
#define ADCH2 2
#define ADCH1 1
#define ADCH0 0

// ADCx_CFG1
#define ADLPC 7
#define ADIV1 6
#define ADIV0 5
#define ADLSMP 4
#define MODE1 3
#define MODE0 2
#define ADICLK1 1
#define ADICLK0 0

// ADCx_CFG2
#define MUXSEL 4
#define ADACKEN 3
#define ADHSC 2
#define ADLSTS1 1
#define ADLSTS0	0

// ADCx_SC2
#define ADACT 7
#define ADTRG 6
#define ACFE 5
#define ACFGT 4
#define ACREN 3
#define DMAEN 2
#define REFSEL1 1
#define REFSEL0 0

// ADCx_SC3
#define CAL 7
#define CALF 6
#define ADCO 3
#define AVGE 2
#define AVGS1 1
#define AVGS0 0


#endif

#include <arm_math.h>
#include <math.h>
#include "frequency_calc.h"
#include "xprintf.h"

double dot_product_atan2(double * v, double * u, int n){
    double result_real = 0.0;
    double result_imag = 0.0;
    uint16_t i;
    for (i = 0; i < n; i+=2){
        result_imag += v[i]*u[i];
        result_real += v[i+1]*u[i+1];
    }
    result_real*=-2;
    result_imag*=2;
    return atan2(result_imag/n, result_real/n);
}

uint32_t isqrt32 (uint32_t n){  
    uint32_t root = 0;  
    uint32_t remaind = n;  
    uint32_t place = 0x40000000;
    while(place > remaind)  
        place >>= 2;  
    while(place){  
        if(remaind >= root + place){  
            remaind -= root - place;  
            root += (place << 1);  
        }  
        root >>= 1;  
        place >>= 2;  
    }  
    return root;  
}  
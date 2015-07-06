#include <arm_math.h>
#include <math.h>
#include "frequency_calc.h"
#include "xprintf.h"

double dot_product_atan2(double * v, double * u, int n){
    double result_real = 0.0;
    double result_imag = 0.0;
    uint16_t i;
    for (i = 0; i < (n*2); i+=2){
        result_imag += v[i]*u[i/2];
        result_real += v[i+1]*u[i/2];
    }
    result_real *= -2;
    result_imag *= 2;
    return atan2(result_imag/n, result_real/n);
}
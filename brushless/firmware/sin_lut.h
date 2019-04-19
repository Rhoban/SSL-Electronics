#ifndef _SIN_LUT_H
#define _SIN_LUT_H

#include <stdint.h>

#define SIN_INPUT_RESOLUTION 16384
#define SIN_OUPUT_MAX 8192

/**
 * Discrete sin
 * Input : x in [0 to SIN_OUPUT_MAX] 
 * output : y in [-SIN_OUPUT_MAX to SIN_OUPUT_MAX]
 */
int16_t discrete_sin(int16_t x);

/**
 * Get the cosinus from angle in turn
 * 
 * turn goes from -1.0 to 1.0
 * return value goes from 
 */
float sin_t(float turn);

/**
 * get the sinus from angle in turn
 * 
 * turn goes from -1.0 to 1.0
 * return value goes from 
 */
float cos_t(float turn);

int16_t discrete_cos(int16_t x);
int16_t discrete_sin(int16_t x);

#endif

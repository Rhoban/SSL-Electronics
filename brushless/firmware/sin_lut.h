#ifndef _SIN_LUT_H
#define _SIN_LUT_H

#include <stdint.h>

#define SIN_INPUT_RESOLUTION 8192
#define SIN_OUPUT_MAX 8192

/**
 * Get the sine from the LUT
 * 
 * x goes from 0 to 8191 (2^13-1)
 * return value goes from 
 */
uint16_t sin_lut(uint16_t x);

/**
 * Discrete sin
 * Input : x in [0 to 8191] 
 * ouptu : y in [-8191 to 8191]
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

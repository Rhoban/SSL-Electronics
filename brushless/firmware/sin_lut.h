#ifndef _SIN_LUT_H
#define _SIN_LUT_H

#include <stdint.h>

/**
 * Get the sine from the LUT
 * 
 * x goes from 0 to 8191
 * return value goes from 
 */
uint16_t sin_lut(uint16_t x);

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

#endif

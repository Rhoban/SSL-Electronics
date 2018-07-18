#ifndef _SIN_LUT_H
#define _SIN_LUT_H

#include <stdint.h>

/**
 * Get the sine from the LUT
 * 
 * x goes from 0 to 8095
 * return value goes from 
 */
uint16_t sin_lut(uint16_t x);

#endif
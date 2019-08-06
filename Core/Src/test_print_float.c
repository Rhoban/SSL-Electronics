/*
 * Copyright (C) 2019  Adrien Boussicault <adrien.boussicault@labri.fr>
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "debug.h"

#define CONFIGURE_TO_PRINT_FLOAT
#include "print_real.h"

#include <math.h>

void print_test(float f){
  PRINTD("-------------");
  PRINTD("%g", f);
  #define NMAX FLOAT_MAX_PRINT_SPACE
  uint32_t nb; 
  uint8_t F[FLOAT_MAX_PRINT_SPACE];
  for(int j=0; j<FLOAT_MAX_PRINT_SPACE; j++){
    F[j] = 'z';
  }
  floating_point_print(f, 10,  F, &nb, NMAX);
  PRINTD("%s", (const char*)F);
}

int main(){
#if 0 
  //float f = 0.0000000000001234567; //423728125;
  //float f = -0.3423728125;
  //float f = 33.3423728125;
  //float f = NAN;
  //float f = INFINITY;
  //float f = -INFINITY;
  //float f = -0.012345;
#endif
  #define N 48
  float f[N] = {
    0,
    0.5,
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    8,
    9,
    10,
    2.0,
    2.23f,
    0.1234567890123e-45,
    0.1234567890123e-44,
    .0000000000001234567890123,
    .000000000001234567890123,
    .00000000001234567890123,
    .0000000001234567890123,
    .000000001234567890123,
    .00000001234567890123,
    .0000001234567890123,
    .000001234567890123,
    .00001234567890123,
    .0001234567890123,
    .001234567890123,
    .01234567890123,
    .1234567890123,
    1.234567890123,
    12.34567890123,
    123.4567890123,
    1234.567890123,
    12345.67890123,
    123456.7890123,
    1234567.890123,
    12345678.90123,
    123456789.0123,
    1234567890.123,
    12345678901.23,
    123456789012.3,
    1234567890123.,
    1.234567890123e38,
    1.234567890123e39,
    NAN,
    INFINITY,
    -INFINITY
  };
  for( int i=0; i<N; i++){
    print_test(f[i]);
    print_test(-f[i]);
  }

  return 0;
}

////
  //int32_t exponent = (
  //  (
  //    BITWISE_CAST(f, uint32_t) >> FLOAT_MANTISSA_SIZE 
  //  ) & (
  //    ~( 1u<<FLOAT_EXPONENT_SIZE )
  //  ) 
  //);
  //int32_t mantissa = (
  //  BITWISE_CAST(f, uint32_t) & (
  //    ( (~0u) << (FLOAT_EXPONENT_SIZE + FLOAT_SIGN_SIZE) ) >> 
  //    (FLOAT_EXPONENT_SIZE + FLOAT_SIGN_SIZE)
  //  )
  //);
  // uint32_t f,p,e;
  // p = FLOAT_MANTISSA_SIZE;
  // if(exponent==0){
  //  e = 1-FLOAT_BIAS;
  //  f = mantissa;
  // }else{
  //  e = exponent-FLOAT_BIAS;
  //  f = mantissa | (1u<<FLOAT_MANTISSA_SIZE);
  // }

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

#define PRINTD(format, ...) printf("D: " format " - %s : %d\n", ##__VA_ARGS__, __FILE__, __LINE__ )

#define CONFIGURE_TO_PRINT_DOUBLE
#include "print_real.h"

#include <math.h>

void print_test(double f){
  PRINTD("-------------");
  PRINTD("%g", f);
  uint32_t nb; 
  uint8_t F[FLOAT_MAX_PRINT_SPACE];
  for(int j=0; j<FLOAT_MAX_PRINT_SPACE; j++){
    F[j] = 'z';
  }
  floating_point_print(f, 10,  F, &nb, FLOAT_MAX_PRINT_SPACE);
  PRINTD("%s", (const char*)F);
}

int main(){
  #define N 48
  double f[N] = {
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
    0.1234567890123456789e-45,
    0.1234567890123456789e-44,
    .0000000000001234567890123456789,
    .000000000001234567890123456789,
    .00000000001234567890123456789,
    .0000000001234567890123456789,
    .000000001234567890123456789,
    .00000001234567890123456789,
    .0000001234567890123456789,
    .000001234567890123456789,
    .00001234567890123456789,
    .0001234567890123456789,
    .001234567890123456789,
    .01234567890123456789,
    .1234567890123456789,
    1.234567890123456789,
    12.34567890123456789,
    123.4567890123456789,
    1234.567890123456789,
    12345.67890123456789,
    123456.7890123456789,
    1234567.890123456789,
    12345678.90123456789,
    123456789.0123456789,
    1234567890.123456789,
    12345678901.23456789,
    123456789012.3456789,
    1234567890123.456789,
    1.234567890123456789e38,
    1.234567890123456789e39,
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

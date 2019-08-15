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

// Do not insert prgama once preprocessor for this header. 

#include <stdio.h>
#include <stdint.h>
#include <math.h>

// TO USE THAT FILE, include it in yout .C and before define the followinf
// DEFINE
#ifdef CONFIGURE_TO_PRINT_FLOAT
  #define FLOAT_SIGN_SIZE 1
  #define FLOAT_EXPONENT_SIZE 8
  #define FLOAT_MANTISSA_SIZE 23
  #define FLOAT_BIAS 127
  #define FLOAT_MAX_PRECISION 7
  #define SIZE_MAX_EXPONENT_PRINT 4 // 1 ('e') + 1 sign exponent + 2 (exponent) 
  #define TYPE_FLOAT float
#endif

#ifdef CONFIGURE_TO_PRINT_DOUBLE
  #define FLOAT_SIGN_SIZE 1
  #define FLOAT_EXPONENT_SIZE 11
  #define FLOAT_MANTISSA_SIZE 52
  #define FLOAT_BIAS 1023
  #define FLOAT_MAX_PRECISION 15
  #define SIZE_MAX_EXPONENT_PRINT 5 // 1 ('e') + 1 sign exponent + 3 (exponent) 
  #define TYPE_FLOAT double
#endif

#define FLOAT_MAX_PRINT_SPACE (FLOAT_MAX_PRECISION+SIZE_MAX_EXPONENT_PRINT+3)
  // en scientifique : PRECISON + EXPONNENT + 1 (sign) + 1('.') + 1(\0)  

#define SIZE_INTEGER_IN_SCIENTIFIC 6 // It's convenient to read
#define SIZE_RATIONAL_IN_SCIENTIFIC SIZE_MAX_EXPONENT_PRINT // To print the maximal precision allowed.

// See How to Article 
// Print Floating-Point Numbers Accurately, Guy L. teele Jr. & Jon L White
// 90 Conf. Prog. Lang. Design and Implementation. 1990.
inline static void fixed_point_fraction(
  TYPE_FLOAT f, // the input of a strictly positive real.
  //uint32_t p,
  const uint32_t B, // The radix of the output
  uint32_t* N, // The resulting number of digit of the output 
  uint8_t* F, // The output in the 'radix_out' base
  uint32_t Nmax // THe maximum size of 
){
  TYPE_FLOAT R;
  uint32_t k;
  uint8_t U=0;

  k = 0;
  R = f;

  while(1){
    k++;
    R *= B;
    U = R;
    R -= U;
    if( k==Nmax  ) break;
    F[k-1] = '0'+U;
  }
  F[k-1] = R < 1.0/2 ? '0'+U : '0'+U+1;
  *N = k; 
}

inline static void floating_point_print(
  TYPE_FLOAT v, uint32_t B, uint8_t* buf, uint32_t* N, uint32_t Nmax
){
  if( isinf(v) ){
      if(v<0){
        buf[0] = '-';
      }else{
        buf[0] = '+';
      }
      buf[1] = 'i';
      buf[2] = 'n';
      buf[3] = 'f';
      buf[4] = '\0';
      *N = 5;
      return;
  }
  if( isnan(v) ){
      buf[0] = 'n';
      buf[1] = 'a';
      buf[2] = 'n';
      buf[3] = '\0';
      *N = 4;
      return;
  }
  if( v == 0.0 ){
    buf[0] = '0';
    buf[1] = '\0';
    *N = 2;
    return;
  }

  *N = 0;
  if(v<0){
    buf[(*N)++] = '-';
    v = -v;
  }
  
  // We compute x the exponent for the scientific mode 
  // v = v' * B**x < b**p * B**x = term 
  int32_t x = 0;
  TYPE_FLOAT vp = v;
  TYPE_FLOAT term = B;
  while( v >= term ){
    x++;
    term *= B;
    vp /= B;
  }
  term = 1.0;
  while( term > v ){
    x--;
    term /= B;
    vp *= B;
  }
  // We disable scientfic when exponent is small
  int exp; 
  TYPE_FLOAT significand = 2*frexpf( v, &exp );
  uint32_t precision = significand == 1.0 ? FLOAT_MAX_PRECISION : FLOAT_MAX_PRECISION-1 ;
  if( x < SIZE_INTEGER_IN_SCIENTIFIC && x > 0 ){
    while( x!= 0){
      precision--;
      x--;
      vp *= B;
    }
  }else if( x >= -SIZE_RATIONAL_IN_SCIENTIFIC && x < 0 ){
    while( x!= 0){
      precision++;
      x++;
      vp /= B;
    }
  }
  // We compute the Exponential part of the number and store the result
  // at the end.
  uint32_t size_exponnent = 0;
  if(x!=0){
    uint8_t s;
    if( x>0 ){
      s = '+';
    }else{
      s = '-';
      x = -x;
    } 
    while(x > 0){
      buf[ Nmax - ++size_exponnent ] = '0'+ (x % B);
      x /= B;
    }
    buf[ Nmax - ++size_exponnent ] = s;
    buf[ Nmax - ++size_exponnent ] = 'e';
  }

  int32_t integral_part_vp = vp; 
  TYPE_FLOAT fractional_part_vp = vp - integral_part_vp; 

  if( integral_part_vp == 0 ){
    buf[(*N)++] = '0';
  }
  
  // We compute the integer part of the number 
  uint32_t size_integral_part_vp=0;
  while(integral_part_vp > 0){
    buf[(*N)++] = '0'+(integral_part_vp % B);
    integral_part_vp /= B;
    size_integral_part_vp++;
  }
  for(uint32_t j = 0; j<size_integral_part_vp/2; j++ ){
    uint8_t t = buf[ (*N)-1-j ];
    buf[ (*N)-1-j ] = buf[ (*N)-size_integral_part_vp+j ];
    buf[ (*N)-size_integral_part_vp+j ] = t;
  }
  buf[(*N)++] = '.';

  int nb = Nmax - *N - size_exponnent -1; // -1 => \0
  // We compute the fractional part of the number
  uint32_t Np;
  fixed_point_fraction(
    fractional_part_vp, //p, 
    B, &Np, buf+*N, 
    nb>precision ? precision : nb
  );
  *N += Np;

  // We place the exponential part at the correct place
  for(uint32_t i=0; i<size_exponnent; i++){
    buf[(*N)++] = buf[Nmax-size_exponnent+i];
  }
  buf[(*N)++] = '\0';
} 

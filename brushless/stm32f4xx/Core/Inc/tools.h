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

#pragma once

#include <math.h>

// give the next id ( (id+1)%size ) when
// size is a power of 2. 
#define MOD(value, size) ((value)&(size-1))
#define NEXT(id, size) MOD(id+1,size)
// give the prev id ( (id-1)%size ) when
// size is a power of 2. 
#define PREV(id, size) ( (id+(size-1))&(size-1))
#define RMASK(nb) ( ~( ((~0u) >> nb) << nb ) )
#define LMASK(nb) ( ~( ((~0u) << nb) >> nb ) )

inline float mod_float(float theta, float quotient){
  float res = fmod(theta, quotient);
  return res>=0 ? res : res+quotient;
}

inline float mod_2_pi(float theta){
  return mod_float(theta, (2*M_PI));
}

#define BORN_MAX(x, max) x = ((x)<=(max))? (x) : (max)
#define BORN_MIN(x, min) x = ((x)>=(min))? (x) : (min)

#define BORN(x, min, max) \
  do { \
    BORN_MIN(x,min); \
    BORN_MAX(x,max); \
  } while(0u)

// Compute he root square of a float.
__attribute__((always_inline))
inline float _VSQRTF(float f){
  float res;
  asm volatile("vsqrt.f32 %0, %1" : "=&t" (res) : "t" (f));
  return res;
}


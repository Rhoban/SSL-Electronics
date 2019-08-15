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

#ifdef DEBUG

#include "debug.h"
#include <terminal.h>
#include <assertion.h>
#include <tools.h>
#include <time.h>

#define NB_SAMPLE 32
_Static_assert( IS_POW_2(NB_SAMPLE), "Have to be a 2^N" );
static uint32_t delta[NB_SAMPLE] = {0};
static uint32_t pos = 0;
static uint32_t last_time_delta = 0;
static int32_t sum_delta = 0;

static inline uint32_t debug_period_us(){
  return sum_delta/NB_SAMPLE;
}

static inline float debug_freq(){
  return 1000.0/debug_period_us();
}
static volatile uint32_t last_freq_update = 0;

void print_freq(uint32_t milis, int fd){
  uint32_t time = time_get_us();
  if(time-last_freq_update > 1000*milis){
    last_freq_update = time;
    DPRINTF(fd, "%.3f KHz", debug_freq());
  }
  sum_delta -= delta[pos];
  sum_delta += ( delta[pos] = (time - last_time_delta) );
  pos = NEXT(pos, NB_SAMPLE);
  last_time_delta = time;
}

#endif

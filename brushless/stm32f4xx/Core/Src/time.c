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

#include "time.h"

void init_countdown(countdown_t * countdown, uint32_t period_us){
  countdown->period_us = period_us;  
  countdown->counter = 0;
}

void reset_countdown(countdown_t * countdown){
  countdown->counter = countdown->period_us;
  countdown->time = time_get_us();
}

static inline void _tick_countdown(countdown_t * countdown){
  uint32_t time = time_get_us();
  uint32_t delta = time -  countdown->time;
  countdown->time = time;
  if( countdown->counter > delta ){
    countdown->counter -= delta;
  }else{
    countdown->counter = 0;
  }
}

void tick_countdown(countdown_t * countdown){
  _tick_countdown(countdown);
}

bool update_countdown(countdown_t * countdown){
  _tick_countdown(countdown);
  if(countdown->counter){
    return false;
  }else{
    countdown->counter = countdown->period_us;
    return true;
  }
}

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

void init_cooldown(cooldown_t * cooldown, uint32_t period_us){
  cooldown->period_us = period_us;  
  cooldown->counter = 0;
}

void reset_cooldown(cooldown_t * cooldown){
  cooldown->counter = cooldown->period_us;
  cooldown->time = time_get_us();
}

static inline void _tick_cooldown(cooldown_t * cooldown){
  uint32_t time = time_get_us();
  uint32_t delta = time -  cooldown->time;
  cooldown->time = time;
  if( cooldown->counter > delta ){
    cooldown->counter -= delta;
  }else{
    cooldown->counter = 0;
  }
}

void tick_cooldown(cooldown_t * cooldown){
  _tick_cooldown(cooldown);
}

bool update_cooldown(cooldown_t * cooldown){
  _tick_cooldown(cooldown);
  if(cooldown->counter){
    return false;
  }else{
    cooldown->counter = cooldown->period_us;
    return true;
  }
}

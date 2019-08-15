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

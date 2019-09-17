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

#include <log.h>
#include "debug.h"
#include <queue.h>

static uint32_t cpt = 0;

define_and_declare_static_queue(log_sample_t, logs, 256 );

volatile log_sample_t* current_log_sample;

void init_log_sample(volatile log_sample_t* sample){
    sample->cpt = cpt;
    sample->to_adapt = 0;
}

static bool log_is_not_full = true;
static bool log_is_enable = false;

volatile uint32_t time_ref;

void log_init(){
  current_log_sample = logs_get_next_to_insert(&logs);
  init_log_sample(current_log_sample);
  log_is_not_full = true;
  time_ref = time_get_us();
}

void log_start(){
  cpt = 0;
  logs_clear(&logs);
  init_log_sample(current_log_sample);
  log_is_not_full = true;
  log_is_enable = true;
}

void log_next_sample(){
  if( ! log_is_enable ) return;
  cpt++;
  if( log_is_not_full ){
    log_is_not_full = logs_insert(&logs);
    if( !log_is_not_full ){
      PRINTJ("Log queue is full. We stop the log service.");
    }
  }
  current_log_sample = logs_get_next_to_insert(&logs);
  init_log_sample(current_log_sample);
}

void log_title(){
  dprintf(
    DIRECT_JTAG_FILE_DESCRIPTOR,
    "cpt, "
    "to_adapt\n"
  );
  delay_us(1000);
}

TERMINAL_COMMAND(start_log,"start log"){
  log_title();
  log_start();
}

void log_stop(){
  log_is_enable = false;
}

TERMINAL_COMMAND(stop_log,"stop log"){
  log_stop();
}

void process_sample(volatile log_sample_t* sample){
  dprintf(
    DIRECT_JTAG_FILE_DESCRIPTOR,
    "%ld, "
    "%f\n" // TO ADAPT
    ,
    sample->cpt,
    sample->to_adapt
  );
}

void log_tick(){
  for( uint32_t i=0; i<(logs_size(&logs)+1)/2; i++){
    process_sample(logs_first(&logs));    
    logs_drop(&logs, 1);    
  }
}

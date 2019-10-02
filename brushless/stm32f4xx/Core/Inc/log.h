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

#include <stdint.h>

#define SIM_VELOCITY 10.0

//
// This tool is used to log some data.
//
// To use this tools, first, you need to 
// activate the log by uncomenting the line
//
// #define ACTIVATE_LOG
//
// in Core/Src/debug.h
// 
// then, you need to adapt the code marked as 
// `TO_ADAPT` or marked as `to_adapt`.
//
// Finally, in your code, you can store the data in the current sample
// by writting :
// 
// current_log_sample->to_adapt = value_to_log;
//
// and then, you log the sample by writting :
//
// log_next_sample();
//
// To log, connect the board with jtag : 
// ./print_jtag > log
//
// Then conect the board with USB tty :
// cu -l /dev/ttyACM0
//
// then type the command : 
//
// start_log
// 
// stop the log at any time :
// 
// stop_log 

typedef struct {
  volatile uint32_t cpt;
  volatile float raw_angle;
  volatile float angle;
  volatile float pred_angle;
  volatile float angle_consign;
  volatile float speed_consign;
  volatile float current_consign;
  volatile float speed;
} log_sample_t;

extern volatile log_sample_t* current_log_sample;
extern volatile uint32_t time_ref;

void log_init();
void log_start();
void log_tick();
void log_next_sample();

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

#include <foc.h>
#include <observer.h>
#include <pid.h>
#include <hardware.h>
#include <motor.h>
#include <frequence_definitions.h>
#include <terminal.h>
#include <assertion.h>
#include "debug.h"

typedef enum {
  OPEN_LOOP_CONTROL,
  ANGULAR_CONTROL,
  SPEED_CONTROL
} foc_mode_t;

typedef struct {
  foc_mode_t mode;

  volatile float angular_consign;
  // PI for the angular position
  Pi_constants angular_pi_constants;
  Pi angular_pi;

  volatile float speed_consign;
  // PI for the speed
  Pi_constants speed_pi_constants;
  Pi speed_pi;

} foc_t;

foc_t foc;

void foc_set_angular_consign(float angular_consign){
  foc.angular_consign = angular_consign;
  motor_change_to_foc_mode();
  foc.mode = ANGULAR_CONTROL;
}

void foc_set_speed_consign(float speed_consign){
  foc.speed_consign = speed_consign;
  motor_change_to_foc_mode();
  observer_update_level( foc.speed_consign );
  foc.mode = SPEED_CONTROL;
}

TERMINAL_COMMAND(sc,"speed consign tr/s"){
  if( argc != 1 ){
    terminal_println_float(foc.speed_consign);
    return;
  }else{
    float speed_csg = 2*M_PI*atof(argv[0]);
    foc_set_speed_consign(speed_csg);
  }
}

TERMINAL_COMMAND(ac,"angular consign tr"){
  if( argc != 1 ){
    terminal_println_float(foc.angular_consign);
    return;
  }else{
    float speed_csg = 2*M_PI*atof(argv[0]);
    foc_set_angular_consign(speed_csg);
  }
}

TERMINAL_COMMAND( ksp, "" ){
  if( argc == 0 ){
    terminal_println_float(foc.speed_pi_constants.p);
  }else if( argc == 1 ){
    float kp = atof(argv[0]);
    foc.speed_pi_constants.p = kp;
  }
}

TERMINAL_COMMAND( ksi, "" ){
  if( argc == 0 ){
    terminal_println_float(foc.speed_pi_constants.i);
  }else if( argc == 1 ){
    float ki = atof(argv[0]);
    foc.speed_pi_constants.i = ki;
  }
}

TERMINAL_COMMAND( kap, "" ){
  if( argc == 0 ){
    terminal_println_float(foc.angular_pi_constants.p);
  }else if( argc == 1 ){
    float kp = atof(argv[0]);
    foc.angular_pi_constants.p = kp;
  }
}

TERMINAL_COMMAND( kai, "ki" ){
  if( argc == 0 ){
    terminal_println_float(foc.angular_pi_constants.i);
  }else if( argc == 1 ){
    float ki = atof(argv[0]);
    foc.angular_pi_constants.i = ki;
  }
}

void print_mode(){
  switch( foc.mode ){
    case OPEN_LOOP_CONTROL:
      terminal_println("OPEN_LOOP_CONTROL");
      break;
    case ANGULAR_CONTROL:
      terminal_println("ANGULAR_CONTROL");
      break;
    case SPEED_CONTROL:
      terminal_println("SPEED_CONTROL");
      break;
    default:
      ASSERT(false);
  } 
}

void print_constants(){
  switch( foc.mode ){
    case OPEN_LOOP_CONTROL:
      break;
    case ANGULAR_CONTROL:
      terminal_print("kp angle : ");
      terminal_println_float(foc.angular_pi_constants.p);
      terminal_print("ki angle : ");
      terminal_println_float(foc.angular_pi_constants.i);
    case SPEED_CONTROL:
      terminal_print("kp speed : ");
      terminal_println_float(foc.speed_pi_constants.p);
      terminal_print("ki speed : ");
      terminal_println_float(foc.speed_pi_constants.i);
      break;
    default:
      ASSERT(false);
  } 
}

void print_consigns(){
  switch( foc.mode ){
    case OPEN_LOOP_CONTROL:
      break;
    case ANGULAR_CONTROL:
      terminal_print("angle consign : ");
      terminal_println_float(foc.angular_consign);
    case SPEED_CONTROL:
      terminal_print("speed consign : ");
      terminal_println_float(foc.speed_consign);
      break;
    default:
      ASSERT(false);
  } 
}

void print_errors(){
  switch( foc.mode ){
    case OPEN_LOOP_CONTROL:
      break;
    case ANGULAR_CONTROL:
      terminal_print("angle err : ");
      terminal_println_float(foc.angular_pi.e);
      terminal_print("angle p err : ");
      terminal_println_float(foc.angular_pi.p);
      terminal_print("angle i err : ");
      terminal_println_float(foc.angular_pi.i);
    case SPEED_CONTROL:
      terminal_print("speed err : ");
      terminal_println_float(foc.speed_pi.e);
      terminal_print("speed p err : ");
      terminal_println_float(foc.speed_pi.p);
      terminal_print("speed i err : ");
      terminal_println_float(foc.speed_pi.i);
      break;
    default:
      ASSERT(false);
  } 
}

TERMINAL_COMMAND(foc, ""){
  print_mode();
  print_consigns();
  print_errors();
  print_constants();
}

TERMINAL_COMMAND(reset_foc, ""){
  reset_pi(&(foc.speed_pi));
  reset_pi(&(foc.angular_pi));
}
void foc_init(){
  init_pi_k(
    &(foc.speed_pi_constants), KP_SPEED, KI_SPEED,
    CURRENT_LIMIT, INTEGRAL_CURRENT_LIMIT
  );
  init_pi_k(
    &(foc.angular_pi_constants), KP_ANGLE, KI_ANGLE,
    SPEED_LIMIT, INTEGRAL_SPEED_LIMIT
  );
  reset_pi(&(foc.speed_pi));
  reset_pi(&(foc.angular_pi));
  foc.mode = OPEN_LOOP_CONTROL;
  foc.speed_consign = 0;
  foc.angular_consign = 0;
}

static inline void _security_check(){
  // TODO
  switch( foc.mode ){
    case OPEN_LOOP_CONTROL:
      break;
    case ANGULAR_CONTROL:
    case SPEED_CONTROL:
      break;
    default:
      ASSERT(false);
      break;
  }
}

float foc_update_and_get_control(){
  float control;
  float angle, angle_error;
  float speed, speed_error;
  switch( foc.mode ){
    case OPEN_LOOP_CONTROL:
      control = 0;
      break;
    case ANGULAR_CONTROL:
      angle = observer_get_angle();
      angle_error = foc.angular_consign - angle;
      update_pi(&(foc.angular_pi_constants), &(foc.angular_pi), angle_error, (1.0/ENCODER_FREQ));
      foc.speed_consign = get_output(&(foc.angular_pi));
      observer_update_level( foc.speed_consign );
    case SPEED_CONTROL:
      speed = observer_get_velocity();
      speed_error = foc.speed_consign - speed;
      update_pi(&(foc.speed_pi_constants), &(foc.speed_pi), speed_error, (1.0/ENCODER_FREQ));
      control = get_output(&(foc.speed_pi));
      break;
    default:
      control = 0;
      ASSERT(false);
      break;
  }
  _security_check();
  return control;
}

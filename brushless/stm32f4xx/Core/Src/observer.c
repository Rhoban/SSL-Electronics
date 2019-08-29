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

#include <observer.h>
#include <tools.h>
#include <assertion.h>
#include <frequence_definitions.h>
#include <terminal.h>
#include <errors.h>
#include <errors.h>
#include "debug.h"

#define HYSTORIC_SIZE 64
_Static_assert(IS_POW_2(HYSTORIC_SIZE), "Should be a 2^n.");

typedef struct {
  int oversample_counter;

  float angle[HYSTORIC_SIZE];
  float velocity;
  uint32_t position;

  volatile uint32_t updated_tick;
  volatile uint32_t current_tick;
  volatile uint32_t norm_command_tick;
  volatile uint32_t pwm_duty_cycle_tick;
  volatile uint32_t encoder_tick;
  volatile uint32_t encoder_time;
  volatile uint32_t encoder_reading_time;

  uint32_t level;
} observer_t; 

static observer_t observer;

void observer_pwm_tick(){
  (observer.current_tick)++;
}
void observer_norm_command_tick(){
  observer.norm_command_tick = observer.current_tick;
}
void observer_pwm_duty_cycletick(){
  observer.pwm_duty_cycle_tick = observer.current_tick;
}
void observer_encoder_tick(){
  observer.encoder_tick = observer.current_tick;
  observer.encoder_time = time_get_us();
}

void observer_start_encoder_reading_tick(){
  observer.encoder_reading_time = time_get_us();
}

void observer_estimate(float * speed, float* angle){
  (*speed) = observer.velocity;
  (*angle) = observer.angle[observer.position] + (
      ((observer.current_tick - observer.updated_tick) & 0xFFFF) *
      observer.velocity
  ) / PWM_DUTY_CYCLE_PERIOD;
}

void observer_init(){
  for( uint32_t i=0; i<HYSTORIC_SIZE; i++ ){
    observer.angle[i] = 0;
  }
  
  observer.velocity = 0;
  observer.level = 0;
  observer.position = HYSTORIC_SIZE-1;
  observer.oversample_counter = OVERSAMPLING_NUMBER-1;
}

static inline void update_velocity(uint32_t level){
  observer.velocity = (
    observer.angle[observer.position] -
    observer.angle[(observer.position-level-1) & (HYSTORIC_SIZE-1)]
  )*OVERSAMPLING_FREQ*(1.0/(level+1));
}

void observer_update(float angle){
  if( ++observer.oversample_counter == OVERSAMPLING_NUMBER ){
    observer.oversample_counter = 0;
  }

  // We make some oversampling
  if(observer.oversample_counter) return;

  observer.position = NEXT(observer.position, HYSTORIC_SIZE);
  
  observer.angle[observer.position] = angle;

  //
  // We update the level according to the velocity :
  // we want the noise is negligible in front of the angle increase.
  //
  // So we want that velocity / update_frequence >> angle_noise 
  // so, 
  // velocity / update_frequence > angle_noise * negligible_factor
  //
  // with 
  //
  // update_frequence = OVERSAMPLING_FREQ/(level+1)
  // 
  // so, we have 
  //
  // velocity *(level+1)/OVERSAMPLING_FREQ > angle_noise * negligible_factor
  //
  // and finally
  // 
  // velocity > OVERSAMPLING_FREQ * angle_noise * negligible_factor / (level+1)
  // or
  // level > ( OVERSAMPLING_FREQ * angle_noise * negligible_factor / velocity )-1
  // 
  #define NEGLIGABLE 100
  #define ANGLE_NOISE (2*M_PI*MAXIMAL_AMPLITUDE_ERROR_AT_50_TR_S/(360*1000.0))
  #define LEVEL_FACTOR (ANGLE_NOISE*NEGLIGABLE*OVERSAMPLING_FREQ)
  #if 0
  float level = fabs(LEVEL_FACTOR/observer.velocity);
  // We change the level according to an hysteresis to avoid osciling 
  // phenomena.
  if( fabs( level - (observer.level+.5) ) >= .5+.2 ){
    observer.level = level;
  }
  if( observer.level >= HYSTORIC_SIZE-1 ) observer.level = HYSTORIC_SIZE-2;
  update_velocity(observer.level);
  #else
  update_velocity(0);
  #endif

  observer.updated_tick = observer.current_tick;
}

TERMINAL_COMMAND( level, "veloctity level"){
  terminal_println_int( observer.level );
}

float observer_get_angle(){
  return observer.angle[observer.position];
}

float observer_get_velocity(){
  return observer.velocity;
}

TERMINAL_COMMAND(angle, "angle in tr (0: tr, 1:rad, 2:deg)")
{
  float angle = observer_get_angle();
  if(argc == 0){
    terminal_println_float( angle/(2*M_PI) );
  }else if(argc==1){
    if( atoi( argv[0] ) == 0 ){
      terminal_println_float( angle/(2*M_PI) );
    }else if( atoi( argv[0] ) == 1 ){
      terminal_println_float( angle );
    }else{
      terminal_println_float( angle*360.0/(2*M_PI) );
    }
  }else{
    terminal_println( "usage: angle [0/1/2]" );
    terminal_println( "   0/1/2 : tr/rad/deg");
  }
}

TERMINAL_COMMAND(speed, "speed in tr/s (0: tr/s, 1:rad/s, 2:deg/s)")
{
  float velocity = observer_get_velocity();
  if(argc == 0){
    terminal_println_float( velocity/(2*M_PI) );
  }else if(argc==1){
    if( atoi( argv[0] ) == 0 ){
      terminal_println_float( velocity/(2*M_PI) );
    }else if( atoi( argv[0] ) == 1 ){
      terminal_println_float( velocity );
    }else{
      terminal_println_float( velocity*360.0/(2*M_PI) );
    }
  }else{
    terminal_println( "usage: velocity [0-1-2]" );
    terminal_println( "   0/1/2 : tr/s-rad/s-deg/s");
  }
}

TERMINAL_COMMAND(delay, "DELAY"){
  PRINTT("EXPECTED DELAY : %d", ENC_SPI_DELAY_US);
  PRINTT("MEASURED DELAY : %ld", observer.encoder_reading_time - observer.encoder_time);
}

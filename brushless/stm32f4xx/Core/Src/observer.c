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

// #define HISTORIC_SIZE 64
#define HISTORIC_SIZE 128
// #define HISTORIC_SIZE 256
_Static_assert(IS_POW_2(HISTORIC_SIZE), "Should be a 2^n.");

#include <filter.h>

typedef struct {
  int oversample_counter;

  float angle[HISTORIC_SIZE];
  float mesure_time_systick[HISTORIC_SIZE];
  float velocity;
  uint32_t position;

  volatile uint32_t updated_tick;
  volatile uint32_t encoder_tick;
  volatile uint32_t angle_measure_tick;

  uint32_t level;
} observer_t; 

static observer_t observer;

void observer_encoder_tick(){
  observer.encoder_tick = SysTick->VAL;
}

void observer_reset(){
  for( uint32_t i=0; i<HISTORIC_SIZE; i++ ){
    observer.angle[i] = 0;
  }
  
  observer.velocity = 0;
  observer.level = 0;
  
  observer.position = HISTORIC_SIZE-1;
  observer.oversample_counter = OVERSAMPLING_NUMBER-1;
}
void observer_init(){
  observer_reset();
}

#include <recursive_macro.h>

#if 1
#define ADAPTATIVE_FACTOR(i, _) 1.0/(i+1),
static const float adaptative_frequence_factors[ HISTORIC_SIZE-1 ] = {
  EVAL(REPEAT(DEC(DEC(HISTORIC_SIZE)), ADAPTATIVE_FACTOR, ~))
  1.0/(HISTORIC_SIZE-2+1)
};
static inline void update_velocity(uint32_t level){
  ASSERT( level < HISTORIC_SIZE-1 );
  observer.velocity = (
    observer.angle[observer.position] -
    observer.angle[(observer.position-level-1) & (HISTORIC_SIZE-1)]
  )*OVERSAMPLING_FREQ*adaptative_frequence_factors[level];
}
#else
static inline void update_velocity(uint32_t level){
  float adaptative_frequence_factor;
  //
  // This is an optimized implementation of the code :
  // adaptative_frequence_factor=1.0/(level+1);
  //
  #define CASE(i, _) case i: adaptative_frequence_factor = 1.0/(i+1); break;
  switch(level){
    EVAL(REPEAT(DEC(HISTORIC_SIZE), CASE, ~))
    default:
       adaptative_frequence_factor = 1; 
  }
  observer.velocity = (
    observer.angle[observer.position] -
    observer.angle[(observer.position-level-1) & (HISTORIC_SIZE-1)]
  )*OVERSAMPLING_FREQ*adaptative_frequence_factor;
}
#endif

static inline void _observer_update_level( float velocity ){
  //
  // When we want to choose a level where the noise is negligible in front of 
  // the angle increase.
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
  #define ADAPTATIVE_FREQUENCE_FACTOR (1.0/LEVEL_FACTOR)
  float inverse_level = fabs(observer.velocity*ADAPTATIVE_FREQUENCE_FACTOR);
  // We change the level according to an hysteresis to avoid osciling 
  // phenomena.
  //
  // The hysteresis condition is :
  // fabs( level - (observer.level+.5) ) >= .5+.2
  // This is the optimized form of that condition : 
  if( fabs( 1.0 - inverse_level*(observer.level+.5) ) >= (.5+.2)*inverse_level ){
    observer.level = 1/inverse_level;
  }
  if( observer.level >= HISTORIC_SIZE-1 ) observer.level = HISTORIC_SIZE-2;
}

TERMINAL_PARAMETER_FLOAT(dt, "time us", 0.0);

void get_estimated_angle_for_next_PWM_update(float* angle, float *speed){
  (*speed) = observer.velocity;
// Sysclk
// counter
//   ^                                       /
//   |     LOAD   |\    |\    |\   /|\--->|\-   |\    |
//   | msk, et1 --|>\   | \---|>\-- | \   |.\   | \   | 
//   |      et3 - | .\  | /\  | .\  |  \  |. \  |  \  | 
//   |  et0 et0 --|-->\-|-  \ | . \ |   \ |.  \ |   \ |
//   |      et2 - | . .\|    \| .  \|    \|.  -\|--->\|
//   |              . .         .          .         .
// <--------------------------------------------------- time in
//                 et0 msk     et0-ENC  et0-2*ENC  et0-3ENC   sysclk
//                 <-->         .          .         .
//                  diff        .       PWM_PERIOD   .
//                  . .         .          <>        .
//                  ^^^         ^^         ^^        ^^        ^^
//                  |||         ||         ||        ||        ||
//              ------------------------------------------------------
//                  et msk      et         et        et        et
//                   fcat        fact       fcat      fcat      fcat
//                  or                                           or
//                      <----------------------------->
//                          delay(oversample_counter)
//
// mts : mesure_time_systick
// et : encoder tick
// fcat : first center aligned tick were the new command is applied
// or : Overampling reset : 
//
// eti = mod( et0-oversample_counter*ENV , LOAD )
// et0 - oversample_counter*ENV = k*LOAD + eti
// et0 = k*LOAD + eti + ovserample_counter*ENC
// mod( msk - et0, LOAD) = mod(
//    msk - eti - ovserample_counter*ENC, LOAD
// )
// diff = mod( msk - eti - OVERSAMPLE*ENC, LOAD ) 
//
// delay = (oversample_counter+1)*ENC + PWM_PERIOD - diff 
  int32_t diff = (
    ( 
      (int32_t) (
        observer.encoder_tick + 
        observer.oversample_counter * SYCLK_TO_ENCODER_PERIOD
      )
    ) - (
      (int32_t) observer.mesure_time_systick[observer.position]
    )
  );
  diff = diff % CLK_SYSCLK_PERIOD;
  diff = diff < 0 ? diff + CLK_SYSCLK_PERIOD: diff;
  float delay = (
    PWM_PERIOD + observer.oversample_counter * SYCLK_TO_ENCODER_PERIOD 
  ) * (1.0/CLK_SYSCLK) + (FILTER_DELAY_US/1000000.0);
  (*angle) = observer.angle[observer.position]
   //+ observer.velocity*dt*(1.0/1000000);
   + observer.velocity * delay;
}

void observer_update_level( float velocity ){
  _observer_update_level( observer.velocity );
}
void observer_update(float angle, uint32_t mesure_time_systick){
  observer.oversample_counter += 1;
  if( observer.oversample_counter == OVERSAMPLING_NUMBER ){
    observer.oversample_counter = 0;
  }

  // We make some oversampling
  if(observer.oversample_counter) return;

  observer.position = NEXT(observer.position, HISTORIC_SIZE);
  observer.angle[observer.position] = angle;
  observer.mesure_time_systick[observer.position] = mesure_time_systick;

  //
  // TODO 
  // The correct level have to be computed with the velocity control command.
  // So we need to move the following line in the function that set the control 
  // command.
  _observer_update_level( observer.velocity );
  update_velocity( observer.level );
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


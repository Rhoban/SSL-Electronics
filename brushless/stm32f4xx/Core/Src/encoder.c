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

#include <encoder.h>
#include <as5047d.h>
#include <terminal.h>
#include <time.h>
#include <stdint.h>
#include <errors.h>
#include "debug.h"
#include <frequence_definitions.h>
#include <filter.h>
#include <observer.h>
#include <motor.h>
//
// Clock computation
//
// At 10 Mhz (BaudRate = 10Mb/s, tclk=100ns), 1 packet of 16 bits is 
// received each : 
// 
//    T_2bits = TL + TclkH + 15 * Tclk + TH + Tcsn = 350 + 50 + 15*100 + 50 + 350 = 2300 ns
//
//    Time to a complete comunication :
//    T_com = T_2bits*3 = 6900 ns < 7us
//
// At our spi clock : SYCLK/PRESCALER = 84MHz/16 = 5.35MHz 
// (Baudrate 5.35Mb/s, tclk=190.476 ns)
//    TL = Tcsn = 1000 ns (what is implemented)
//    TclkH = TH = Tclk/2 = 95.238 ns
//     
//    T_2bits = 1000 + 95.238 + 15*190.476 + 95.238 + 1000 = 5048 ns
//    T_com = T_2bits*3 = 15142 ns < 16 us
//   
// The clock prescaler for a clock of 84 MHz is 16 : 84Mhz/16 = 5.25MHz
// (BaudRate = 5.25b/s)
//
static as5047d_t device;

typedef struct {
  butterworth_3_data_t butterworth_filter;
  
  uint16_t last_raw_angle;  
  volatile uint16_t raw_angle;
  uint32_t data_sysclk_count;
  int32_t absolute_angle;

  volatile as5047d_error_t raw_angle_error;  

  as5047d_diagnostic_t diagnostic;
  as5047d_error_t diagnostic_error;

  bool is_ready;

  float angle;
  float velocity;
} encoder_t;

typedef enum {
  NOT_INIT,
  READING_ANGLE,
  MAKE_DIAGNOSTIC
} encoder_state_t;
volatile static encoder_state_t state = NOT_INIT;

#define RESOLUTION_ONE_TURN 16384
_Static_assert( IS_POW_2(RESOLUTION_ONE_TURN), "" );

static encoder_t encoder;
volatile static uint32_t computation_is_done = 0;

void encoder_init(  
  SPI_HandleTypeDef* hspi, GPIO_TypeDef*  gpio_port_cs, uint16_t gpio_pin_cs
){
  encoder.is_ready = false;

  encoder.absolute_angle = 0;  
  encoder.last_raw_angle = 0;  
  encoder.raw_angle = 0;
  encoder.raw_angle_error = AS5047D_OK;  
  clear_diagnostic( &encoder.diagnostic );
  encoder.diagnostic_error = AS5047D_OK;
  encoder.angle = 0.0;
  encoder.velocity = 0.0;

  // To have good result, the buterworth filter should work on small 
  // values.
  // So we give a limit_value to the filter and the filter perform
  // the following computation : 
  //
  // real_value = base + small_value
  // filtered = filter( small_value )
  // small_value = fmod( filtered, limit_value )
  // base += filtered - small_value;
  // 
  // This limit_value should be smaller as possible to maintain
  // a good precision.
  //
  // To determine that value, we use the maximal velocity
  // TODO : To improve and to prove ! 
  const float frame_limit = (
    (3.0*MAXIMAL_MOTOR_VELOCITY*2*M_PI)/ENCODER_FREQ
  );
  init_butterworth_3_pulsation_1256_rad_s(
    &(encoder.butterworth_filter), frame_limit
  );
  
  as5047d_init( &device, hspi, gpio_port_cs, gpio_pin_cs );

}

void encoder_start(){
  computation_is_done = 0;
  state = READING_ANGLE;
 
  delay_us( 2.7 * FREQ_TO_US( ENCODER_FREQ ) );
  encoder.absolute_angle = encoder.raw_angle;
  encoder.last_raw_angle = encoder.raw_angle;

  encoder.is_ready = true;
}
void encoder_stop(){
  state = NOT_INIT;
}

extern TIM_HandleTypeDef htim5;

void start_read_encoder_position(){
#if 0
  if( !as5047d_start_reading_angle(&device) ){
    raise_warning(WARNING_ENCODER_BUSY, ENCODER_DO_NOT_START);
  }
#else
  if( !as5047d_fast_reading_angle(&device) ){
    raise_warning(WARNING_ENCODER_BUSY, ENCODER_DO_NOT_START);
  }
#endif
}

volatile static bool diagnostic_request = false;


//
// This function should be as fast as possible, because its
// code is executed with the priority of a SPI interruption.
//
void as5047d_call_back_when_finished(as5047d_t* as5047d){
  if( as5047d == &device ){
    switch( state ){
      case READING_ANGLE:
        if( computation_is_done == 0 ){
          computation_is_done++;
          encoder.data_sysclk_count = device.data_sysclk_count;
          encoder.raw_angle = as5047d_data_to_angle(&device);
          encoder.raw_angle_error = device.error;

          // We reset timer 5 to raise an interuption and to execute in a lower priority
          // the rest of the calculus.  
          __HAL_TIM_SetCounter(&htim5, 0u);
        }else{
          raise_warning(WARNING_ENCODER_LAG, LAG_IN_ANGLE_COMPUTATION);
        }
        device.is_ready = true;
        if( diagnostic_request ){
          state = MAKE_DIAGNOSTIC;
          if( !as5047d_start_reading_diagnostic(&device) ){
            raise_warning(WARNING_ENCODER_BUSY, ENCODER_DO_NOT_START);
          }
        }
        break;
      case MAKE_DIAGNOSTIC:
        encoder.diagnostic_error = device.error;
        if(!device.error){
          as5047d_data_to_diagnostic(&device, &(encoder.diagnostic)); 
        }
        state = READING_ANGLE; 
        diagnostic_request = false;
        device.is_ready = true;
        break;
      case NOT_INIT:
        break;
      default:
        ASSERT(false);
    }
  }
}

static inline uint16_t predict_encoder_angle(
  uint16_t last_encoder_angle, float velocity
){ 
  return last_encoder_angle + (
    (uint16_t) (
      velocity*(RESOLUTION_ONE_TURN/(2*M_PI*ENCODER_FREQ))
    )
  );
}

inline int32_t encoder_compute_delta(uint16_t a, uint16_t b)
{
    int32_t delta = ((int32_t) b) - ((int32_t) a);

    if (delta > 0x1fff) {
        delta -= 0x4000;
    }
    if (delta < -0x1fff) {
        delta += 0x4000;
    }

    return delta;
}

//
// This function is automatically called with an TIM5 interruption. This TIM5
// interuption is raised by chanel1. This channel is configured in an ouput 
// compare mode.
// This interuption is manually raised by reseting the counter of TIM5i by 
// using the callback function `as5047d_call_back_when_finished()`.
// 
// We don't execute this function inside the as5047d callback because the 
// execution is made under a SPI priority. We want to perform this computation
// calcul with a lower priority.
//
// So, DON'T CALL THIS FUNCTION BY YOURSELF.
// 
void encoder_compute_angle(){
  if( (computation_is_done > 1) ){
      return;
  }
  computation_is_done++;
  if( state == NOT_INIT ){
    computation_is_done = 0;
    return;
  }
  if(encoder.raw_angle_error){
    if(
        encoder.raw_angle_error & 
        (AS5047D_ERROR | AS5047D_SPI_ERROR | AS5047D_SPI_CRASH)
    ){
      if(encoder.raw_angle_error & AS5047D_SPI_ERROR){
        raise_error(ERROR_ENCODER_SPI_TRANSMITRECEIVE, encoder.raw_angle_error);
      }
      if(device.error & AS5047D_SPI_CRASH){
        raise_error(ERROR_ENCODER_SPI_CRASH, encoder.raw_angle_error);
      }
      if( device.error & ~( AS5047D_SPI_ERROR | AS5047D_SPI_CRASH ) ){
        raise_error(ERROR_ENCODER, encoder.raw_angle_error);
      }
    }else{
      raise_warning(WARNING_ENCODER_ERROR_ON_AS5047D, encoder.raw_angle_error);
    }
    // We reconstruct a dynamic angle.
    encoder.raw_angle = predict_encoder_angle(
      encoder.last_raw_angle, observer_get_velocity()
    );
  }
  int32_t delta = encoder_compute_delta(
    encoder.raw_angle, encoder.last_raw_angle
  );
  #define TOLERANCE_FACTOR 1.5
  #define MAXIMAL_DELTA_tr ((1.0*MAXIMAL_MOTOR_VELOCITY)/ENCODER_FREQ)
  #define MAXIMAL_DELTA ((int32_t) (TOLERANCE_FACTOR * MAXIMAL_DELTA_tr * RESOLUTION_ONE_TURN))
  if( abs(delta) >  MAXIMAL_DELTA ){
    if( encoder.is_ready ){
      raise_warning(WARNING_ENCODER_UNEXPECTED_VALUE, ANGLE_INCREASE_IS_TOO_BIG);
      encoder.raw_angle = predict_encoder_angle(
        encoder.last_raw_angle, observer_get_velocity()
      );
      delta = encoder_compute_delta(
        encoder.raw_angle, encoder.last_raw_angle
      );
    }
  }
  encoder.absolute_angle += delta;
 
  //#define DESACTIVE_FILTER 1 
  #ifndef DESACTIVE_FILTER
    update_butterworth_3_pulsation_1256_rad_s(
      encoder.absolute_angle * (2*M_PI/RESOLUTION_ONE_TURN),
      &(encoder.butterworth_filter)
    );
    encoder.angle = get_filtered_data(&(encoder.butterworth_filter));
  #else
    encoder.angle = encoder.absolute_angle * (2*M_PI/RESOLUTION_ONE_TURN);
  #endif

  encoder.last_raw_angle = encoder.raw_angle;

  observer_update( encoder.angle, encoder.data_sysclk_count );
  
  computation_is_done = 0;
}

void encoder_set_origin(){
  encoder.absolute_angle = 0;
  encoder.velocity = 0;
  reset_filter(&(encoder.butterworth_filter));
}

TERMINAL_COMMAND(set_origin, "Define the origine"){
  encoder_set_origin();
}

void encoder_error_spi_call_back(){
  as5047d_error_spi_call_back(&device);
}

void encoder_spi_call_back(){
  as5047d_spi_call_back(&device);
}

void encoder_tick(){
}

TERMINAL_COMMAND(enc, "Read encoder")
{
  terminal_println_int( encoder.raw_angle );
}


TERMINAL_COMMAND(diagnostic, "encoder diagnostic")
{
  diagnostic_request = true;
  
  while(diagnostic_request);

  if(!encoder.diagnostic_error){
    terminal_print( "magnetic field strength :" );
    if(encoder.diagnostic.mfs_too_low){
      terminal_println(" too low");
    }
    if(encoder.diagnostic.mfs_too_high){
      terminal_println(" too high");
    }
    if( ( ! encoder.diagnostic.mfs_too_high ) && ( ! encoder.diagnostic.mfs_too_low ) ){
      terminal_println(" OK");
    }
    terminal_print( "cordi : " );
    if(encoder.diagnostic.cordi_overflow){
      terminal_println("OVERFLOW (the measured angle is not reliable)");
    }else{
      terminal_println("OK");
    }
    terminal_print( "offset comensation : " );
    if(encoder.diagnostic.offset_compensation_is_ready){
      terminal_println("OK");
    }else{
      terminal_println("NOT READY");
    }
    terminal_print( "automatic gain control : " );
    terminal_println_int( encoder.diagnostic.automatic_gain_control );
  }else{
    terminal_print("Reading error : ");
    terminal_println_int(encoder.diagnostic_error);
  }
}

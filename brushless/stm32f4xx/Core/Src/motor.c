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

#include <motor.h>
#include <foc.h>
#include <tools.h>
#include <sin_table.h>
#include <observer.h>
#include <encoder.h>
#include <frequence_definitions.h>
#include <stdint.h>
#include <stdbool.h>
#include <assertion.h>
#include "stm32f4xx_hal.h"
#include <hardware.h>
#include <time.h>
#include <terminal.h>
#include <arm_math.h>
#include "debug.h"

#define MAX_VOLTAGE MOTOR_VOLTAGE
#define MAX_VOLTAGE_SQUARE (MOTOR_VOLTAGE*MOTOR_VOLTAGE)
#define INV_MAX_VOLTAGE (1.0/MOTOR_VOLTAGE)
#define MAX_STM32_PWM (100-MAX_PWM_PERCENTAGE+*PWM_PERIOD)/100
#define MAX_PWM_AMPLITUDE ( \
  ( \
    (100 - (DRIVER_MIN_PWM_PERCENTAGE + MAX_PWM_PERCENTAGE)) * PWM_PERIOD \
  )/100 \
)
#define MIN_PWM ( \
  ( \
    DRIVER_MIN_PWM_PERCENTAGE * PWM_PERIOD \
  )/100 \
)

typedef enum {
  TARE,
  FIXED_DQ_VOLTAGE,
  CLOSED_LOOP
} motor_mode_t;

inline const char* motor_mode_string( motor_mode_t m ){
  switch(m){
    case TARE:
      return "TARE";
    case FIXED_DQ_VOLTAGE:
      return "FIXED_DQ_VOLTAGE";
    case CLOSED_LOOP:
      return "CLOSED_LOOP";
    default:
      return "?";
  }
}

typedef struct {
  bool is_ready;

  volatile float direct_voltage;
  volatile float quadrature_voltage;

  float phase_voltage_u;
  float phase_voltage_v;
  float phase_voltage_w;

  volatile uint32_t phase_pwm_u;
  volatile uint32_t phase_pwm_v;
  volatile uint32_t phase_pwm_w;

  motor_mode_t mode;

  bool free_spining;

  bool reset_origin;
} motor_t;

static motor_t motor;

extern TIM_HandleTypeDef htim1;

void motor_stop_device(){
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
}

void motor_start_device(){
  if( HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ){
    raise_error(ERROR_MOTOR, MOTOR_PWM_INITIALISATION_1);
  }
  if( HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ){
    raise_error(ERROR_MOTOR, MOTOR_PWM_INITIALISATION_2);
  }
  if( HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK ){
    raise_error(ERROR_MOTOR, MOTOR_PWM_INITIALISATION_3);
  }
}

void motor_init_device(){
}

static inline void dq_voltage_to_phase_voltage(float angle){
  #ifndef REVERSE_PHASE
    float theta_park = NB_POSITIVE_MAGNETS * angle;
  #else
    float theta_park = - NB_POSITIVE_MAGNETS * angle;
  #endif

  float t1 = mod_2_pi(theta_park);
  float t2 = mod_2_pi(theta_park - (2*M_PI/3));
  float t3 = mod_2_pi(theta_park + (2*M_PI/3));

#if 0
  float c1 = cos_table(t1);
  float s1 = sin_table(t1);
  float c2 = cos_table(t2);
  float s2 = sin_table(t2);
  float c3 = cos_table(t3);
  float s3 = sin_table(t3);
#else
  float c1 = arm_cos_f32(t1);
  float s1 = arm_sin_f32(t1);
  float c2 = arm_cos_f32(t2);
  float s2 = arm_sin_f32(t2);
  float c3 = arm_cos_f32(t3);
  float s3 = arm_sin_f32(t3);
#endif

  motor.phase_voltage_u = c1*motor.direct_voltage - s1*motor.quadrature_voltage;
  motor.phase_voltage_v = c2*motor.direct_voltage - s2*motor.quadrature_voltage;
  motor.phase_voltage_w = c3*motor.direct_voltage - s3*motor.quadrature_voltage;
}


void motor_set_direct_quadrature_voltage(
  float direct_voltage, float quadrature_voltage
){
  // max( pwm_u ) = alpha * max( u-v )
  //
  // We need now to evaluate the maximum value
  // of the difference between two phases voltages when theta is going from 
  // 0 to 2 pi.
  // 
  // u - v = a*(c1-c2) - b*(s1-s2);
  //
  // Let J = max( a*(c1-c2) - b*(s1-s2) )
  //
  // max occurs
  // when deriv( a*(c1-c2) - b*(s1-s2) ) = -a*(s1-s2) - b*(c1-c2) = 0
  // so
  // a * J = (c1-c2) * ( a^2 + b^2 )     and     c1-c2 = a * J /(a^2+b^2)
  // b * J = (s1-s2) * ( -a^2 -b^2 )     and     s1-s2 = - b * J /(a^2+b^2)
  //
  // (c1-c2)^2 + (s1-s2)^2 = 2 - 2 (c1*c2+s1*s2) = 
  // 2 - 2*cos( theta - (theta +- 2pi/3) ) = 2 - 2 cos(2pi/3) = 3
  //
  // 3 = (a^2 + b^2) * J^2 / (a^2 + b^2)^2
  // 
  // J = sqrt(3) * sqrt( a^2 + b^2 ) 
  //
  // So the maximal difference voltage is equal to 
  // max( u - v ) = sqrt(3) * sqrt( a^2 + b^2 )
  float norm2 = direct_voltage*direct_voltage + quadrature_voltage*quadrature_voltage;
  if( norm2 <=  (MAX_VOLTAGE_SQUARE/3.0) ){
    motor.direct_voltage = direct_voltage;
    motor.quadrature_voltage = quadrature_voltage;
  }else{
    #define SQRT_3 1.7320508075688772
    #define EPSILON 0.00001
    _Static_assert(fabs(SQRT_3- sqrt(3.0)) < 0.000001,"");
    float inverse_norm = 1.0/(sqrt(3.0) * _VSQRTF(norm2) + EPSILON);
    motor.direct_voltage = direct_voltage * MAX_VOLTAGE * inverse_norm;
    motor.quadrature_voltage = quadrature_voltage * MAX_VOLTAGE * inverse_norm;
  }
}


void phase_voltage_to_pwm_voltage(){
  float min = motor.phase_voltage_u;
  if(motor.phase_voltage_v < min) min = motor.phase_voltage_v;
  if(motor.phase_voltage_w < min) min = motor.phase_voltage_w;
  motor.phase_pwm_u = (uint32_t)(
    (motor.phase_voltage_u - min)*(INV_MAX_VOLTAGE * MAX_PWM_AMPLITUDE)
  ) + MIN_PWM;
  motor.phase_pwm_v = (uint32_t)(
    (motor.phase_voltage_v - min)*(INV_MAX_VOLTAGE * MAX_PWM_AMPLITUDE)
  ) + MIN_PWM;
  motor.phase_pwm_w = (uint32_t)(
    (motor.phase_voltage_w - min)*(INV_MAX_VOLTAGE * MAX_PWM_AMPLITUDE)
  ) + MIN_PWM;
}

static inline void set_pwm_to_zero(){
  motor.phase_pwm_u = 0;
  motor.phase_pwm_v = 0;
  motor.phase_pwm_w = 0;
}


static inline void set_pwm_to_free_spinig(){
  set_pwm_to_zero();
  motor_apply_pwm();
}

static inline void activate_driver(){
  motor.free_spining = false;
  set_pwm_to_free_spinig();
  HAL_GPIO_WritePin(UEN_GPIO_Port, UEN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(VEN_GPIO_Port, VEN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(WEN_GPIO_Port, WEN_Pin, GPIO_PIN_SET);
}

static inline void deactivate_driver(){
  HAL_GPIO_WritePin(UEN_GPIO_Port, UEN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(VEN_GPIO_Port, VEN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(WEN_GPIO_Port, WEN_Pin, GPIO_PIN_RESET);
  set_pwm_to_free_spinig();
  motor.free_spining = true;
}

void motor_init(){
  motor.is_ready = false;

  motor.free_spining = true;

  set_pwm_to_zero();

  motor.direct_voltage = 0;
  motor.quadrature_voltage = 0;

  motor.mode = FIXED_DQ_VOLTAGE;

  motor_init_device();

  motor.is_ready = true;
  motor.reset_origin = false;

  foc_init();
}

void motor_start(){
  motor_start_device();
  deactivate_driver();
  motor_apply_pwm();
}

void motor_enable(bool enable){
  if( enable ){
    activate_driver();
  }else{
    deactivate_driver();
  }
}

void motor_emergency(){
  raise_error(ERROR_EMERGENCY, MOTOR_EMERGENCY);
  deactivate_driver();
}

bool motor_is_enable(){
  return !motor.free_spining;
}

void motor_prepare_pwm(){
  float angle = 0;
  switch( motor.mode ){
    case CLOSED_LOOP:
      //foc_get_control(&motor.direct_voltage, &motor.quadrature_voltage);
    case FIXED_DQ_VOLTAGE:
      angle = observer_get_angle();
      break;
    case TARE:
      angle = 0;
      break;
    default:
      deactivate_driver();
      ASSERT(false);
  } 
  dq_voltage_to_phase_voltage(angle);
  if( motor.free_spining ){
    set_pwm_to_zero();
  }else{
    phase_voltage_to_pwm_voltage();
  }
  motor.is_ready = true;
}

void motor_apply_pwm(){
  // ASSERT(motor.is_ready);
  if(motor.is_ready){
    htim1.Instance->CCR1 = motor.phase_pwm_u;
    htim1.Instance->CCR2 = motor.phase_pwm_v;
    htim1.Instance->CCR3 = motor.phase_pwm_w;
    motor.is_ready = false;
  }
  if( motor.mode == TARE && motor.reset_origin){
    encoder_set_origin();
    observer_reset();
  }
}



void tare(){
  motor_mode_t save_mode = motor.mode;
  
  motor.reset_origin = false;
  motor.mode = TARE;
  motor.quadrature_voltage = 0;
  motor.direct_voltage = MAX_VOLTAGE_FOR_TARING_PROCESS;

  DELAY_MS(1000);
  motor.reset_origin = true;
  DELAY_MS(1000);
  
  motor.reset_origin = false;

  
  motor.direct_voltage = 0;
  motor.quadrature_voltage = 0;

  motor.mode = save_mode;
}

TERMINAL_COMMAND(dqv, "Set direct and quadratic voltage" ){
  motor.mode = FIXED_DQ_VOLTAGE;
  if(argc == 2){
    float d = atof( argv[0] );
    float q = atof( argv[1] );
    BORN(d, 0, MAX_VOLTAGE);
    BORN(q, 0, MAX_VOLTAGE);
    motor_set_direct_quadrature_voltage(d, q);
  }else{
    terminal_print("dqv <direct voltage> <quadratic voltage> (max voltage:");
    terminal_print_int(MAX_VOLTAGE);
    terminal_println(")");
  }
}

TERMINAL_COMMAND(motor_mode, "the mode of the motor" ){
  terminal_print( motor_mode_string( motor.mode ) );
  terminal_println("");
}

TERMINAL_COMMAND(motor_consign, "motor consign"){
  terminal_print("direct v. : ");
  terminal_println_float(motor.direct_voltage);
  terminal_print("quadrature v. : ");
  terminal_println_float(motor.quadrature_voltage);
}

TERMINAL_COMMAND(pwm, "phase pwm"){
  terminal_print("u : ");
  terminal_println_int(motor.phase_pwm_u);
  terminal_print("v : ");
  terminal_println_float(motor.phase_pwm_v);
  terminal_print("w. :");
  terminal_println_float(motor.phase_pwm_w);
}

TERMINAL_COMMAND(tare, "define the position 0" ){
  tare();
}

TERMINAL_COMMAND(motor_info, ""){
  terminal_print("enable : ");
  terminal_println_int(motor_is_enable());
  terminal_print("Motor Voltage : ");
  terminal_println_int( MOTOR_VOLTAGE );
  terminal_print("Max Voltage : ");
  terminal_println_int( MAX_VOLTAGE );
}



TERMINAL_COMMAND(motor_enable, "set motor" ){
  bool enable = false;
  if(argc==0){
    enable = true;
  }else if(argc==1){
    enable = atoi(argv[0]);
  }else{
    terminal_print("enable_motor [<0|1>]");
  }
  motor_enable(enable);
}

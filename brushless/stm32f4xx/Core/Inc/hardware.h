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

#define NB_POSITIVE_MAGNETS 8
#define MOTOR_VOLTAGE 24
#define REVERSE_PHASE
#define ADDITIONAL_DELAY_FROM_ENCODER_TO_COMMAND -110
#define KEV_rad_per_V_S 19.8 // Electromagnetic constant
#define FRICTION_CONSTANT_rad_s 25.0 // Electromagnetic constant
#define MOTOR_LQ 0.0 //0.000463  // Quadrature Inductance : inductance of the virtual coil that is 
              // perpendicular to the virtual magnet of the stator.
#define MOTOR_R 4.6 // 2.6  // resistance of the coil
#define MOTOR_KEM (1.0/KEV_rad_per_V_S)   // Electromagnetic constant of the motor

// 
// To allow current measure and good rising edges, we need to have a minimal 
// duty cycle in the pwm of the motor driver.
//
#define DRIVER_MIN_PWM_PERCENTAGE 6
_Static_assert( DRIVER_MIN_PWM_PERCENTAGE >= 0, "" );
_Static_assert( 100 - DRIVER_MIN_PWM_PERCENTAGE >= 0, "" );
_Static_assert( DRIVER_MIN_PWM_PERCENTAGE >= 6, "Minimal value to have stable current (measured with an oscilloscope" ); // TODO Make this value independent of the frequence !

//
// Pwm will be bounded in 
// [SECURITY_MIN_PWM_PERCENTAGE, 100-SECURITY_MAX_PWM_PERCENTAGE] 
// percent.
//
#define SECURITY_MIN_PWM_PERCENTAGE 20
#define SECURITY_MAX_PWM_PERCENTAGE 20
_Static_assert( SECURITY_MAX_PWM_PERCENTAGE >= 0, "" );
_Static_assert( SECURITY_MAX_PWM_PERCENTAGE <= 100, "" );
_Static_assert( SECURITY_MIN_PWM_PERCENTAGE >= 0, "" );
_Static_assert( SECURITY_MIN_PWM_PERCENTAGE <= 100, "" );
// To not burn the drivers, a pwm have to be stricly lesser than 100% !
_Static_assert( SECURITY_MAX_PWM_PERCENTAGE >= 1, "have to be equal to 0 !!" );
_Static_assert( 100 > SECURITY_MIN_PWM_PERCENTAGE + SECURITY_MAX_PWM_PERCENTAGE, "" );
_Static_assert(
  SECURITY_MIN_PWM_PERCENTAGE >= DRIVER_MIN_PWM_PERCENTAGE,
  "The pwm security is not sufficient to allow some current measures." 
);
_Static_assert(
  SECURITY_MAX_PWM_PERCENTAGE >= DRIVER_MIN_PWM_PERCENTAGE,
  "The pwm security is not sufficient to allow som current measures." 
);


#define MAX_VOLTAGE_FOR_TARING_PROCESS (MAX_VOLTAGE/8.0) 
#define MAX_CURRENT 1.7

#define SPEED_LIMIT (30*2*M_PI)
#define INTEGRAL_SPEED_LIMIT SPEED_LIMIT
#define KP_ANGLE 12.0
#define KI_ANGLE 1.0

//#define U_CURRENT_IS_POSITIVE
#define U_CURRENT_IS_NEGATIVE
#define V_CURRENT_IS_POSITIVE
//#define V_CURRENT_IS_NEGATIVE
//#define W_CURRENT_IS_POSITIVE
#define W_CURRENT_IS_NEGATIVE

#if defined(U_CURRENT_IS_NEGATIVE) && defined(U_CURRENT_IS_POSITIVE)
  _Static_assert(false, "");
#endif
#if !( defined(U_CURRENT_IS_NEGATIVE) || defined(U_CURRENT_IS_POSITIVE) )
  _Static_assert(false, "");
#endif
#if defined(V_CURRENT_IS_NEGATIVE) && defined(V_CURRENT_IS_POSITIVE)
  _Static_assert(false, "");
#endif
#if !( defined(V_CURRENT_IS_NEGATIVE) || defined(V_CURRENT_IS_POSITIVE) )
  _Static_assert(false, "");
#endif
#if defined(W_CURRENT_IS_NEGATIVE) && defined(W_CURRENT_IS_POSITIVE)
  _Static_assert(false, "");
#endif
#if !( defined(U_CURRENT_IS_NEGATIVE) || defined(U_CURRENT_IS_POSITIVE) )
  _Static_assert(false, "");
#endif


// PID 
#define KP_SPEED 0.1
#define KI_SPEED 0.1
#define CURRENT_LIMIT MAX_CURRENT 
#define INTEGRAL_CURRENT_LIMIT MAX_CURRENT 

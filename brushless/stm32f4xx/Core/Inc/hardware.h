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
#define MOTOR_LQ 0  // Quadrature Inductance : inductance of the virtual coil that is 
              // perpendicular to the virtual magnet of the stator.
#define MOTOR_R 0   // resistance of the coil
#define MOTOR_KEM 0   // Electromagnetic constant of the motor


// 
// To allow current measure and good rising edges, we need to have a minimal 
// duty cycle in the pwm of the motor driver.
//
#define DRIVER_MIN_PWM_PERCENTAGE 6
_Static_assert( DRIVER_MIN_PWM_PERCENTAGE >= 0, "" );
_Static_assert( 100 - DRIVER_MIN_PWM_PERCENTAGE >= 0, "" );

//
// Pwm will be bounded in 
// [SECURITY_MIN_PWM_PERCENTAGE, 100-SECURITY_MAX_PWM_PERCENTAGE] 
// percent.
//
#define SECURITY_MIN_PWM_PERCENTAGE 6
#define SECURITY_MAX_PWM_PERCENTAGE 6
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


#define MAX_VOLTAGE_FOR_TARING_PROCESS (MAX_VOLTAGE/12.0) 

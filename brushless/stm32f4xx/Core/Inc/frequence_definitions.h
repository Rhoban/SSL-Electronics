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

#include <time.h>
#include <tools.h>

#define MAXIMAL_AUDIO_HUMAN_FREQUENCY 22000
#define MAXIMAL_AUDIO_DOG_FREQUENCY 45000
#define MAXIMAL_AUDIO_CAT_FREQUENCY 64000

#define CAT_FRIENDLY
#define DOG_FRIENDLY
#define HUMAN_FRIENDLY

#if defined(CAT_FRIENDLY)
  #define PWM_FREQ 140000
#elif defined(DOG_FRIENDLY)
  #define PWM_FREQ 96000
#elif defined(HUMAN_FRIENDLY)
  #define PWM_FREQ 60000
#endif
_Static_assert( CLK_SYSCLK % PWM_FREQ == 0, "" );

#define PWM_PERIOD  (CLK_SYSCLK/PWM_FREQ)
#define NB_OF_BITS_FOR_PWM_PRESCALER 16
_Static_assert( PWM_PERIOD <= RMASK(NB_OF_BITS_FOR_PWM_PRESCALER), "" );

//
//                 1/PWM_FREQ
//                   <----->
//                   |     |     |     |     |     |
//                   |_    |     |_    |     |_    |
// compare -> ......_| |_..|...._| |_..|...._| |_..|....
//                _| |   |_|  _| |   |_|  _| |   |_|
//              _|   |     |_|   |     |_|   |     |_
//                   |     |     |     |     |     |
//                  _|_    |    _|_    |    _|_    |
// center-      ___| | |___|___| | |___|___| | |___|_
// aligned           |     |     |     |     |     |
//                   |     |     |     |     |     |
//                   <----------->                   
//             1/(CENTER_ALIGNED_PWM_FREQ)                 
//

#define CENTER_ALIGNED_PERIOD 2
_Static_assert( CENTER_ALIGNED_PERIOD == 2, "Because it is the center aligned mode." );

#define CENTER_ALIGNED_PWM_FREQ (PWM_FREQ/CENTER_ALIGNED_PERIOD)
_Static_assert( PWM_FREQ % CENTER_ALIGNED_PERIOD == 0, "" );
_Static_assert( CLK_SYSCLK % CENTER_ALIGNED_PWM_FREQ == 0, "" );

#define DUTY_CYCLE_PRECISION 600
_Static_assert( DUTY_CYCLE_PRECISION <= PWM_PERIOD, "" );

_Static_assert(
  MAXIMAL_AUDIO_HUMAN_FREQUENCY < CENTER_ALIGNED_PWM_FREQ,
  "Robot should not be listen by humans."
);  
#ifdef DOG_FRIENDLY
_Static_assert(
  MAXIMAL_AUDIO_DOG_FREQUENCY < CENTER_ALIGNED_PWM_FREQ,
  "Robot should not be listen by dogs."
);
#endif
#ifdef CAT_FRIENDLY
  _Static_assert(
    MAXIMAL_AUDIO_CAT_FREQUENCY < CENTER_ALIGNED_PWM_FREQ,
    "Robot should not be listen by cats."
  );
#endif  

#define IR2104_RISE_TIME 170  // ns // See the datasheet of I42104
#define IR2104_FALL_TIME 90  // ns // See the datasheet of I42104
#define IR2104_SWITCHING_TIME (IR2104_RISE_TIME+IR2104_FALL_TIME)
#define MOTOR_DRIVER_SWITCHING_TIME IR2104_SWITCHING_TIME // ns // See

#if defined(CAT_FRIENDLY)
  #define NEGLIGIBLE_FACTOR 54  // 100 would be better to say it is negligible :(
#elif defined(DOG_FRIENDLY)
  #define NEGLIGIBLE_FACTOR 80  // Idem :(
#elif defined(HUMAN_FRIENDLY)
  #define NEGLIGIBLE_FACTOR 128 // >100 :(
#endif

#define FREQ_ONE_NS 1000000000
_Static_assert(
  CENTER_ALIGNED_PWM_FREQ * MOTOR_DRIVER_SWITCHING_TIME * NEGLIGIBLE_FACTOR < FREQ_ONE_NS,
  "The switching time of the motor dirver is not negligible."
);

#define AS5047D_FREQ_MAX 11000
#define AS5047D_FREQ_MIN 9000
#define ENCODER_FREQ_MAX AS5047D_FREQ_MAX
#define ENCODER_FREQ_MIN AS5047D_FREQ_MIN
#define MAXIMAL_AS5047D_ERROR_AT_50_TR_S 38  // in milli-degree (1/1000 degree)
#define MAXIMAL_AS5047D_ERROR 20  // in milli-degree (1/1000 degree)


#if defined(CAT_FRIENDLY)
  #define ENCODER_FREQ 10000
#elif defined(DOG_FRIENDLY)
  #define ENCODER_FREQ 9600
#elif defined(HUMAN_FRIENDLY)
  #define ENCODER_FREQ 10000
#endif
_Static_assert( ENCODER_FREQ_MIN < ENCODER_FREQ, "" );
_Static_assert( ENCODER_FREQ_MAX > ENCODER_FREQ, "" );

#define ENCODER_PERIOD (CENTER_ALIGNED_PWM_FREQ/ENCODER_FREQ)
_Static_assert(  CENTER_ALIGNED_PWM_FREQ % ENCODER_FREQ == 0, "" ); 

#define NYQUIST_FACTOR 5
_Static_assert( NYQUIST_FACTOR>=2, "Nyquist factor have to be greater than 2.");
_Static_assert( ENCODER_FREQ % NYQUIST_FACTOR == 0, "");
#define OVERSAMPLING_NUMBER 4
#define MAXIMAL_NB_POSITIVE_MAGNETS 8
#define MINIMAL_NB_POSITIVE_MAGNETS 7

#define OVERSAMPLING_FREQ (ENCODER_FREQ/OVERSAMPLING_NUMBER)
_Static_assert( ENCODER_FREQ % OVERSAMPLING_NUMBER == 0, "");

#define MAXON_MAXIMLAL_VELOCITY 50 // 50 tr/s
#define MAXIMAL_MOTOR_VELOCITY MAXON_MAXIMLAL_VELOCITY

#define MAXIMAL_PHASE_MOTOR_FREQ (MAXIMAL_MOTOR_VELOCITY*MAXIMAL_NB_POSITIVE_MAGNETS)
_Static_assert( 
  MAXIMAL_PHASE_MOTOR_FREQ * NYQUIST_FACTOR * OVERSAMPLING_NUMBER <= ENCODER_FREQ,
  ""
);

#define MAXIMAL_AMPLITUDE_ERROR (2*MAXIMAL_AS5047D_ERROR)
      // in milli-degree (1/1000 degree)
#define MAXIMAL_AMPLITUDE_ERROR_AT_50_TR_S (2*MAXIMAL_AS5047D_ERROR_AT_50_TR_S)
      // in milli-degree (1/1000 degree)

#define table_sin_8
#ifdef table_sin_8
  #include <sin_table.h>
  #define SINUS_TABLE_SIZE SIN_RESOLUTION
  #define NB_FOLDING_SINUS 8 // by using sin(2*t) = 1 - 2 sin^2(pi/4 - t)
#else
  _Static_assert(false, "TODO: Downgrad sin_table.h to us 4 folding.")
  #define SINUS_TABLE_SIZE 4096
  #define NB_FOLDING_SINUS 4
#endif
#define SINUS_RESOLUTION (NB_FOLDING_SINUS*SINUS_TABLE_SIZE)

// We want to place a magnet at the encoder position precision
#define MINIMAL_PARK_RESOLUTION (SINUS_RESOLUTION*MINIMAL_NB_POSITIVE_MAGNETS)
_Static_assert(
  (360*1000)/MINIMAL_PARK_RESOLUTION < MAXIMAL_AMPLITUDE_ERROR,
  "With the sinus table resolution, it is not possible the place a magnet at"
  "the given encoder position. You should increase size of sinus table."
);


#if defined(CAT_FRIENDLY)
  #define PWM_DUTY_CYCLE_PERIOD 5 
#elif defined(DOG_FRIENDLY)
  #define PWM_DUTY_CYCLE_PERIOD 5 
#elif defined(HUMAN_FRIENDLY)
  #define PWM_DUTY_CYCLE_PERIOD 3 
#endif

#define PWM_DUTY_CYCLE_FREQ (CENTER_ALIGNED_PWM_FREQ/PWM_DUTY_CYCLE_PERIOD)
_Static_assert(CENTER_ALIGNED_PWM_FREQ % PWM_DUTY_CYCLE_PERIOD == 0, "");

#define NORM_COMMAND_FREQ (ENCODER_FREQ/NYQUIST_FACTOR)
_Static_assert(ENCODER_FREQ % NYQUIST_FACTOR == 0, "");

#define NORM_PERIOD (PWM_DUTY_CYCLE_FREQ/NORM_COMMAND_FREQ)
_Static_assert(PWM_DUTY_CYCLE_FREQ % NORM_COMMAND_FREQ == 0, "");

// We want that the sinus table precision is smaller that the delta angle 
// of the application at high speed.
_Static_assert(
  // 1/MINIMAL_PARK_RESOLUTION < MAXIMAL_MOTOR_VELOCITY / PWM_DUTY_CYCLE_FREQ 
  PWM_DUTY_CYCLE_FREQ
  <
  MINIMAL_PARK_RESOLUTION*MAXIMAL_MOTOR_VELOCITY
  ,
  "Your Sinus table is not big enought to update correctly the angle position"
  "of the park transform"
);

// We want that the sinus table precision is smaller that the delta angle 
// of the application at low speed.
#define EXPECTED_MINIMAL_MOTOR_VELOCITY 0.13
#define IN_FLOAT 1.0
_Static_assert(
  // 1/MINIMAL_PARK_RESOLUTION < EXPECTED_MINIMAL_MOTOR_VELOCITY / PWM_DUTY_CYCLE_FREQ 
  PWM_DUTY_CYCLE_FREQ*IN_FLOAT
  <
  MINIMAL_PARK_RESOLUTION*EXPECTED_MINIMAL_MOTOR_VELOCITY
  ,
  "Your Sinus table is not enought big to update correctly the angle position"
  "of the park transform."
);

// TODO :
//
//  1) Verifier que la table des sinus a une précision suffisament 
//           grande vis à vis de la résolution de l'angle calculé par l'observeur
//  2) Vérifier que l'erreur du calcul en sinus (par la table) est 
//     plus grossier que la précision commandable via PWM_PERIOD
//  3) 

// Frequnce relation summary :
// ---------------------------
// 
//      CLK_SYSCLK
//          |
//          | / PWM_PERIOD 
//          =
//       PWM_FREQ
//          |
//          | / CENTER_ALIGNED_PERIOD (=2)
//          =
// CENTER_ALIGNED_PWM_FREQ
//          |__________________________________________________
//          |                        |                         |
//          | /ENCODER_PERIOD        | / OBS_PERIOD            | / PWM_DUTY_CYCLE_PERIOD
//          =                        =                         =
//      ENCODER_FREQ       ANGLE_OBSERVER_FREQ . . . = PWM_DUTY_CYCLE_FREQ
//          |              =ARG_COMMAND_FREQ         (DUTY CYCLE use ARG_COMMAND + lin. inter. NORM_COMMAND)       .
//          |_________________________                        . 
//          |                         |                       . / NORM_PERIOD
//          | / OVERSAMPLING_NUMBER   | / NYQUIST_FACTOR      .
//          =                         =                       .
//      ANGLE_FREQ              NORM_COMMAND_FREQ = . . . . . .
//          |
//          | / NYQUIST_FACTOR
//          >
// MAXIMAL_PHASE_MOTOR_FREQ
//          |
//          | / MAXIMAL_NB_POSITIVE_MAGNETS
//          =
// MAXIMAL_MOTOR_VELOCITY
// 

#define SYCLK_TO_ENCODER_PERIOD (CENTER_ALIGNED_PERIOD*PWM_PERIOD*ENCODER_PERIOD)
#define SYCLK_TO_PWM_DUTY_CYCLE_PERIOD (CENTER_ALIGNED_PERIOD*PWM_PERIOD*PWM_DUTY_CYCLE_PERIOD)

//
// Convert a period to a time.
// The period T should represent be the ration betweeen CLK_SYSCLK and 
// a the CLK_SYSCLK/T .
// 
#define PERIOD_TO_NS(period) ( (period*100000)/(CLK_SYSCLK/10000) )
#define PERIOD_TO_US(period) ( (period*1000)/(CLK_SYSCLK/1000) )

#define FREQ_TO_US(freq) (1000000/freq)

#if 0
  #define ENC_SPI_DELAY_US 10
  #define ENC_SPI_DELAY (CLK_SYSCLK*ENC_SPI_DELAY_US/1000000)
  //#define ENC_SPI_DELAY (PWM_PERIOD * CENTER_ALIGNED_PERIOD /4)
#elif 0
  #define MIN_ENC_SPI_DELAY_US 15
  #define ENC_SPI_DELAY ( \
      (PWM_PERIOD/2) + (PWM_PERIOD*CENTER_ALIGNED_PERIOD) * ( \
      1 + ( (MIN_ENC_SPI_DELAY_US*CENTER_ALIGNED_PWM_FREQ)/1000000) \
    ) \
  )
  #define  ENC_SPI_DELAY_US PERIOD_TO_US(ENC_SPI_DELAY)
#else
  #define ENC_SPI_DELAY (PWM_PERIOD/2)
//  #define ENC_SPI_DELAY (3*PWM_PERIOD/2)
  #define ENC_SPI_DELAY_US PERIOD_TO_US(ENC_SPI_DELAY)
#endif

#include <as5047d.h>

//
// Time process for as5047d
//
#define ENCODER_SPI_BAUDRATE SPI_BAUDRATEPRESCALER_32

// We manualy tested that a SPI baudrate strictly lesser than 32 will produce
// instabilities on the SPI comunication.
// This is due to the presence of some interruptions with an higher priority 
// preemption (more urgency).
// If you need to increase this baudrate, you have to change the priority of
// these interruptions. In our brushless driver, this is not possible because
// higher priority interruptions are used to update accuratly the PWM of the
// phases.
_Static_assert(
  ENCODER_SPI_BAUDRATE >= 32, "Baud rate HAVE to be greater or equals to 32."
);
#define AS5047D_PRESCALAR (2*ENCODER_SPI_BAUDRATE) // TODO : Check with stm32 doc
#define AS5047D_CLK (CLK_SYSCLK/AS5047D_PRESCALAR)
#define AS5047D_PERIOD_CLK_ns  (1000000000/AS5047D_CLK + 1)

#define AS5047D_TIME_ns AS5047D_MINIMAL_TIME_COMUNICATION_ns(AS5047D_PERIOD_CLK_ns)
#define AS5047D_TIME_FAILURE_ns AS5047D_MINIMAL_TIME_COMUNICATION_WITH_FAILURE_ns(AS5047D_PERIOD_CLK_ns)
#define AS5047D_UPDATE_FREQ (1000000000/AS5047D_TIME_ns)
#define AS5047D_UPDATE_FAILURE_FREQ (1000000000/AS5047D_TIME_FAILURE_ns)

#define AS5047D_UPDATE_SECURITY_FACTOR 2 // To save time for computing the result
_Static_assert(
  AS5047D_UPDATE_FREQ > AS5047D_UPDATE_SECURITY_FACTOR*ENCODER_FREQ,
  "Encoder frequence is to big for AS5047D."
);
_Static_assert(
  AS5047D_UPDATE_FAILURE_FREQ > AS5047D_UPDATE_SECURITY_FACTOR*ENCODER_FREQ,
  "Encoder frequence is to big for AS5047D."
);

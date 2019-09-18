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

#define RADC_KHOMS 6 // See datasheet of stm32f401 : DS9716 Rev 11, Avril 2019, page 104 
#define CADC_PICO_FARAD 7 // // See datasheet of stm32f401 : DS9716 Rev 11, Avril 2019, page 105 

#define RAIN_HOMS 100 // It is the output resistance, in the worst case of the 
  // AO of INA2401.
  // see. http://e2e.ti.com/support/amplifiers/f/14/t/763177?INA240-Output-Impedance
  // and http://e2e.ti.com/support/amplifiers/f/14/t/699660?tisearch=e2e-sitesearch&keymatch=faq:true
#define RC_SAMPLING_TIME_NS ((RADC_KHOMS+ RAIN_HOMS/1000)*CADC_PICO_FARAD)

#define NB_CYCLE_SAMPLING_TIME 3
#define PCLCK2_FREQ 84000000
#define ADC_CLOCK_PRESCALAR 4
#define ADC_FREQ (PCLCK2_FREQ/ADC_CLOCK_PRESCALAR)

#define SAMPLING_FREQ (ADC_FREQ/NB_CYCLE_SAMPLING_TIME)
#define SAMPLING_TIME_NS (1000000/(SAMPLING_FREQ/1000))
#define MINIMAL_SAMPLING_TIME_FACTOR 3.3805
  // The loading time of a RC circuit can be computed from 
  // V = 1 - exp(-t/RC) 
  // If we want that V/Vlimit > f then
  // 1-exp(-t/RC) > f
  // -ln( 1-f ) * RC < t
  // So we need to wait RC time to let the time of the ADC to reach 63% 
  // of the wanted value.
  // So, if we want to reach 99% of the measured value, we need to wait : 
  // -log(1-0.99) * RC = 4.6 * RC  seconds
  // if we want to reach 96% of the measured value, we need to wait 
  // -log(1-0.96) * RC = 3.21 * RC  seconds
_Static_assert( SAMPLING_TIME_NS>RC_SAMPLING_TIME_NS*MINIMAL_SAMPLING_TIME_FACTOR, "" );

//
// We want a precision of 1/100.
// TODO : Have a better argument for this precision.
#define CURRENT_FACTOR_PRECISION 100

#define NB_BIT_PRECISION 8
_Static_assert(
  (1u<<NB_BIT_PRECISION) > CURRENT_FACTOR_PRECISION,
  "Not enought precision for the current measure."
);
_Static_assert( 6<=NB_BIT_PRECISION, "See stm32F4 datasheet"  );
_Static_assert( NB_BIT_PRECISION<=12, "See stm32F4 datasheet"  );
_Static_assert( NB_BIT_PRECISION%2==0, "See stm32F4 datasheet"  );

#define ADC_CLOCK_CYCLE (3+NB_BIT_PRECISION)

#define CURRENT_DECODING_FREQ (ADC_FREQ/(NB_CYCLE_SAMPLING_TIME+ADC_CLOCK_CYCLE))
#define CURRENT_SAMPLING_FREQ (ADC_FREQ/(NB_CYCLE_SAMPLING_TIME))

#include <frequence_definitions.h>

#define delay_ns 460
#define delay_period 39
#define CURRENT_MEDURE_HALF_PERIOD ((DUTY_CYCLE_CURRENT_MESURE_IN_MOTOR_PWM*PWM_PERIOD)/100)

#define DUTY_CYCLE_CURRENT_MESURE_IN_MOTOR_PWM ( \
  (100 * CENTER_ALIGNED_PWM_FREQ) / SAMPLING_FREQ \
)

#define CURRENT_MEDURE_HALF_PERIOD ((DUTY_CYCLE_CURRENT_MESURE_IN_MOTOR_PWM*PWM_PERIOD)/100)

#include <hardware.h>

_Static_assert(
  DUTY_CYCLE_CURRENT_MESURE_IN_MOTOR_PWM < DRIVER_MIN_PWM_PERCENTAGE,
  "PWM driver should should have a minimal value to allow current measure."
);

#define INA240A1_GAIN 20
#define CURRENT_AO_GAIN INA240A1_GAIN

#define STM32_ADC_VOLTAGE_MAX 3.3

typedef enum {
  ADC_LAG=1,
  ADC_IS_NOT_ENABLED=2,
  ADC_FAIL_TO_START=3,
  ADC_FAIL_TO_STOP=4,
  ADC_PWM=5
} current_error_t;

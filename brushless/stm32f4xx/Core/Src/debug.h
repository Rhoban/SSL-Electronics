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

#include "terminal.h"
#include <serial.h>

#include <stdio.h>
#include <time.h>
#include <tools.h>
#include <assertion.h>
#include <main.h>
#include <errors.h>

#define DPRINTF(fd, format, ...) dprintf(fd, "D " format "\n", ##__VA_ARGS__ )
//#define DPRINTF(fd, format, ...) dprintf(fd, "D: " format " - %s : %d\n", ##__VA_ARGS__, __FILE__, __LINE__ )

// Printing debug on JTAG
#define PRINTJ(format, ...) DPRINTF( JTAG_FD, format, ##__VA_ARGS__ )

// Printing debug on terminal
#define PRINTT(format, ...) DPRINTF( TERMINAL_FD, format, ##__VA_ARGS__ )

// Define a static variable `variable_name` and update it by computing the 
// frequence in Hz of all "calls of that MACRO".
#define FREQ(variable_name, nb_sample) \
  static float variable_name; \
  { \
    _Static_assert( IS_POW_2(nb_sample), "Have to be a 2^N" ); \
    static uint32_t variable_name ## _freq_delta[nb_sample] = {0}; \
    static uint32_t variable_name ## _freq_pos = 0; \
    static uint32_t variable_name ## _freq_last_time_delta = 0; \
    static int32_t variable_name ## _freq_sum_delta = 0; \
    uint32_t variable_name ## _freq_time = time_get_us(); \
    variable_name ## _freq_sum_delta -= variable_name ## _freq_delta[variable_name ## _freq_pos]; \
    variable_name ## _freq_sum_delta += ( \
      variable_name ## _freq_delta[variable_name ## _freq_pos] = ( \
        variable_name ## _freq_time - variable_name ## _freq_last_time_delta \
      ) \
    ); \
    variable_name ## _freq_pos = NEXT(variable_name ## _freq_pos, nb_sample); \
    variable_name ## _freq_last_time_delta = variable_name ## _freq_time; \
    variable_name = (1000000.0*nb_sample)/variable_name ## _freq_sum_delta; \
  }

#define AVERAGE(variable_name, variable_to_read, nb_sample) \
  static float variable_name; \
  { \
    _Static_assert( IS_POW_2(nb_sample), "Have to be a 2^N" ); \
    static float variable_name ## _average_sample[nb_sample] = {0}; \
    static uint32_t variable_name ## _average_pos = 0; \
    static float variable_name ## _average_sum = 0; \
    variable_name ## _average_sum -= variable_name ## _average_sample[variable_name ## _average_pos]; \
    variable_name ## _average_sum += ( \
      variable_name ## _average_sample[variable_name ## _average_pos] = ( \
        variable_to_read \
      ) \
    ); \
    variable_name ## _average_pos = NEXT(variable_name ## _average_pos, nb_sample); \
    variable_name = variable_name ## _average_sum/nb_sample; \
  }


static inline void DELAY_MS(uint32_t milli) {
  uint32_t deadline = time_get_us()+milli*1000;
  while (time_get_us() < deadline ) {
    asm("nop");
  }
}


#define DPRINT_PERIODIC(fd, period, format, ... ) {\
    static volatile uint32_t _time_ ## __COUNTER__ = 0; \
    uint32_t _time_2_ ## __COUNTER__ = time_get_us(); \
    if( _time_2_ ## __COUNTER__ - _time_ ## __COUNTER__ >= 1000*period){ \
      DPRINTF(fd, format, ##__VA_ARGS__ ); \
      _time_ ## __COUNTER__ = time_get_us(); \
    }\
  }

#define PRINTJ_PERIODIC(period, format, ... ) \
  DPRINT_PERIODIC(JTAG_FD, period, format, ##__VA_ARGS__ )

#define PRINTT_PERIODIC(period, format, ... ) \
  DPRINT_PERIODIC(TERMINAL_FD, period, format, ##__VA_ARGS__ )

// Print the frequence of the calls of 
// print_freq.
// The parameter milis is the printing period 
// of the result in a file descriptor.
// fd is the "file descriptor".
// Usual file descriptors are TERMINAL_FD and HTAG_FD .
void print_freq(uint32_t milis, int fd);


#define EXECUTE_ONETIME(condition) \
    static volatile uint32_t _assertion_ ## __COUNTER__ = 0; \
    if( \
      (_assertion_ ## __COUNTER__ == 0) && (condition) && \
      (_assertion_ ## __COUNTER__ ++ == 0) \
    )


#define DASSERTION(fd, condition, format, ...) \
  { \
    EXECUTE_ONETIME(!(condition)) \
    { \
      DPRINTF(fd, format, ##__VA_ARGS__); \
    } \
  }

#define ASSERTIONJ(condition, format, ...) DASSERTION(JTAG_FD, condition, format, ##__VA_ARGS__)
#define ASSERTIONT(condition, format, ...) DASSERTION(TERMINAL_FD, condition, format, ##__VA_ARGS__)

#define DWATCH(fd, condition, cooldown_ms, format, ...) \
  { \
    static uint32_t _watch_last_time_ ## __COUNTER__ = 0; \
    uint32_t _watch_time ## __COUNTER__ = time_get_us(); \
    if( (condition) && ((_watch_time ## __COUNTER__ - _watch_last_time_ ## __COUNTER__) >= 1000*cooldown_ms) ){ \
      _watch_last_time_ ## __COUNTER__ = _watch_time ## __COUNTER__; \
      DPRINTF(fd, format, ##__VA_ARGS__); \
    } \
  }

#define WATCHJ(condition, cooldown_ms, format, ...) DWATCH(JTAG_FD, condition, cooldown_ms, format, ##__VA_ARGS__)
#define WATCHT(condition, cooldown_ms, format, ...) DWATCH(TERMINAL_FD, condition, cooldown_ms, format, ##__VA_ARGS__)

#define LED HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin)

#define LED_ON HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET)
#define LED_OFF HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET)

#define COOLDOWN(period) COUNTDOWN(period)

#ifdef DEBUG
  #define RAISE_ERROR raise_error_file( __FILE__ ); raise_error( ERROR_LINE, __LINE__ )
  #define RAISE_WARNING raise_warning_file( __FILE__ ); raise_warning( ERROR_LINE, __LINE__ )
#else
  #define RAISE_ERROR ((void)0U)
#endif

#define HELP1(v) h ## v = v;
#define HELP2(v) TERMINAL_PARAMETER_INT( h ## v , "", 0 )

#ifdef DEBUG
  #define LED_ON_WHEN_ENCODER_TICK
  #define LED_ON_WHEN_COMPUTING_COMMANDS
#endif

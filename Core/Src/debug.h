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

#define DPRINTF(fd, format, ...) dprintf(fd, "D " format "\n", ##__VA_ARGS__ )
//#define DPRINTF(fd, format, ...) dprintf(fd, "D: " format " - %s : %d\n", ##__VA_ARGS__, __FILE__, __LINE__ )

// Printing debug on JTAG
#define PRINTJ(format, ...) DPRINTF( JTAG_FD, format, ##__VA_ARGS__ )

// Printing debug on terminal
#define PRINTT(format, ...) DPRINTF( TERMINAL_FD, format, ##__VA_ARGS__ )

#define FREQT(t) print_freq(t, TERMINAL_FD)
#define FREQJ(t) print_freq(t, JTAG_FD)


static inline void DELAY_MS(uint32_t milli) {
  uint32_t deadline = time_get_us()+milli*1000;
  while (time_get_us() < deadline ) {
    asm("nop");
  }
}


#define DPRINT_PERIODIC(fd, period, name, format, ... ) {\
    static volatile uint32_t _time_ ##name = 0; \
    uint32_t _time_2_ ##name = time_get_us(); \
    if( _time_2_ ##name - _time_ ##name >= 1000*period){ \
      DPRINTF(fd, format, ##__VA_ARGS__ ); \
      _time_ ##name = time_get_us(); \
    }\
  }

#define PRINTJ_PERIODIC(period, name, format, ... ) \
  DPRINT_PERIODIC(JTAG_FD, period, name, format, ##__VA_ARGS__ )

#define PRINTT_PERIODIC(period, name, format, ... ) \
  DPRINT_PERIODIC(TERMINAL_FD, period, name, format, ##__VA_ARGS__ )

// Print the frequence of the calls of 
// print_freq.
// The parameter milis is the printing period 
// of the result in a file descriptor.
// fd is the "file descriptor".
// Usual file descriptors are TERMINAL_FD and HTAG_FD .
void print_freq(uint32_t milis, int fd);


#define EXECUTE_ONETIME(name, condition) \
    static volatile uint32_t _assertion_ ##name = 0; \
    if( (_assertion_ ##name == 0) && (condition) && (_assertion_ ##name ++ == 0) )


#define DASSERTION(fd, name, condition, format, ...) \
  { \
    EXECUTE_ONETIME(name, !(condition)) \
    { \
      DPRINTF(fd, format, ##__VA_ARGS__); \
    } \
  }

#define ASSERTIONJ(name, condition, format, ...) DASSERTION(JTAG_FD, name, condition, format, ##__VA_ARGS__)
#define ASSERTIONT(name, condition, format, ...) DASSERTION(TERMINAL_FD, name, condition, format, ##__VA_ARGS__)

#define DWATCH(fd, name, condition, cooldown_ms, format, ...) \
  { \
    static uint32_t _watch_last_time_ ##name = 0; \
    uint32_t _watch_time ##name = time_get_us(); \
    if( (condition) && ((_watch_time ##name - _watch_last_time_ ##name) >= 1000*cooldown_ms) ){ \
      _watch_last_time_ ##name = _watch_time ##name; \
      DPRINTF(fd, format, ##__VA_ARGS__); \
    } \
  }

#define WATCHJ(name, condition, cooldown_ms, format, ...) DWATCH(JTAG_FD, name, condition, cooldown_ms, format, ##__VA_ARGS__)
#define WATCHT(name, condition, cooldown_ms, format, ...) DWATCH(TERMINAL_FD, name, condition, cooldown_ms, format, ##__VA_ARGS__)



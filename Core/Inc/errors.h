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

#include <stdint.h>
#include <stdbool.h>

typedef enum {
  ERROR_DEBUG=1,
  ERROR_SYSTEM_INITIALISATION=2,
  ERROR_USB_INITIALISATION=3,
  ERROR_ENCODER=4,
  ERROR_ENCODER_NB_TRANSMITRECEIVE=5,
  ERROR_ENCODER_SPI_TRANSMITRECEIVE=6,
  ERROR_ENCODER_SPI_CRASH=7,
  ERROR_TIMER_INIT_AT_LINE=8,
  ERROR_STM32_HAL_LIBRARY=9
} error_code_t;

inline const char* error_to_string( error_code_t e ){
  switch( e ){
    case ERROR_DEBUG :
      return "ERROR_DEBUG";
    case ERROR_SYSTEM_INITIALISATION:
      return "ERROR_SYSTEM_INITIALISATION";
    case ERROR_USB_INITIALISATION:
      return "ERROR_USB_INITIALISATION";
    case ERROR_ENCODER:
      return "ERROR_ENCODER";
    case ERROR_ENCODER_NB_TRANSMITRECEIVE:
      return "ERROR_ENCODER_NB_TRANSMITRECEIVE";
    case ERROR_ENCODER_SPI_TRANSMITRECEIVE:
      return "ERROR_ENCODER_SPI_TRANSMITRECEIVE";
    case ERROR_ENCODER_SPI_CRASH:
      return "ERROR_ENCODER_SPI_CRASH";
    case ERROR_TIMER_INIT_AT_LINE:
      return "ERROR_TIMER_INIT_AT_LINE";
    case ERROR_STM32_HAL_LIBRARY:
      return "ERROR_STM32_HAL_LIBRARY";
    default : 
      return "?";
  }
}

typedef struct {
  error_code_t code;
  uint32_t value;
} error_t;

bool has_errors();
bool has_filtered_errors();
uint32_t nb_errors();
uint32_t nb_filtered_errors();
bool error_queue_is_full();
bool filtered_error_queue_is_full();
void append_error(error_t e);
void raise_error(error_code_t code, uint32_t value);
void process_all_errors(void (*fct)(error_t e, void* data), void* data);
void process_all_filtered_errors(void (*fct)(error_t e, void* data), void* data);
void clear_errors();
void clear_filtered_errors();
void print_error(const error_t * e, bool verbose);


typedef enum {
  WARNING_LAG = 1,
  WARNING_ENCODER_BUSY = 2,
  WARNING_ENCODER_ERROR_ON_AS5047D = 3
} warning_code_t;

inline const char* warning_to_string( warning_code_t w ){
  switch( w ){
    case WARNING_LAG :
      return "WARNING_LAG"; 
    case WARNING_ENCODER_BUSY :
      return "WARNING_ENCODER_BUSY"; 
    case WARNING_ENCODER_ERROR_ON_AS5047D :
      return "WARNING_ENCODER_ERROR_ON_AS5047D";
    default : 
      return "?";
  }
}

typedef struct {
  warning_code_t code;
  uint32_t value;
} warning_t;

bool has_warnings();
bool has_filtered_warnings();
uint32_t nb_warnings();
uint32_t nb_filtered_warnings();
bool warning_queue_is_full();
bool filtered_warning_queue_is_full();
void append_warning(warning_t e);
void raise_warning(warning_code_t code, uint32_t value);
void process_all_warnings(void (*fct)(warning_t e, void* data), void* data);
void process_all_filtered_warnings(void (*fct)(warning_t e, void* data), void* data);
void clear_warnings();
void clear_filtered_warnings();
void print_warning(const warning_t * w, bool verbose);

typedef enum {
  TAKE_IT,
  IGNORE,
  FILTER
} filter_rule_t;

// Could be redefined
filter_rule_t filter_error(const error_t* e);
filter_rule_t filter_warning(const warning_t* w);

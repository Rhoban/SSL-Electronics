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
  ERROR_STRING=2,  // Special error used to raise a string
  ERROR_FLOAT=3,   // Special error used to raise a float
  ERROR_INT=4,   // Special error used to raise an int32_t
  ERROR_FILE=5,   // Special error used to raise the line of a file
  ERROR_LINE=6,   // Special error used to raise the line of a file
  ERROR_ASSERTION=7,
  ERROR_SYSTEM_INITIALISATION=8,
  ERROR_USB_INITIALISATION=9,
  ERROR_ENCODER=10,
  ERROR_ENCODER_DEVICE=11,
  ERROR_ENCODER_SPI_TRANSMITRECEIVE=12,
  ERROR_ENCODER_SPI_CRASH=13,
  ERROR_TIMER_INIT_AT_LINE=14,
  ERROR_STM32_HAL_LIBRARY=15,
  ERROR_MOTOR=16,
  ERROR_EMERGENCY=17
} error_code_t;

inline const char* error_to_string( error_code_t e ){
  switch( e ){
    case ERROR_DEBUG :
      return "ERROR_DEBUG";
    case ERROR_STRING :
      return "ERROR_STRING";
    case ERROR_FLOAT :
      return "ERROR_FLOAT";
    case ERROR_INT :
      return "ERROR_INT";
    case ERROR_FILE :
      return "ERROR_FILE";
    case ERROR_LINE :
      return "ERROR_LINE";
    case ERROR_ASSERTION :
      return "ERROR_ASSERTION";
    case ERROR_SYSTEM_INITIALISATION:
      return "ERROR_SYSTEM_INITIALISATION";
    case ERROR_USB_INITIALISATION:
      return "ERROR_USB_INITIALISATION";
    case ERROR_ENCODER:
      return "ERROR_ENCODER";
    case ERROR_ENCODER_DEVICE:
      return "ERROR_ENCODER_DEVICE";
    case ERROR_ENCODER_SPI_TRANSMITRECEIVE:
      return "ERROR_ENCODER_SPI_TRANSMITRECEIVE";
    case ERROR_ENCODER_SPI_CRASH:
      return "ERROR_ENCODER_SPI_CRASH";
    case ERROR_TIMER_INIT_AT_LINE:
      return "ERROR_TIMER_INIT_AT_LINE";
    case ERROR_STM32_HAL_LIBRARY:
      return "ERROR_STM32_HAL_LIBRARY";
    case ERROR_MOTOR:
      return "ERROR_MOTOR";
    case ERROR_EMERGENCY:
      return "ERROR_EMERGENCY";
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
void raise_error_float(float value);
void raise_error_int(int32_t value);
void raise_error_string(const char* value);
void raise_error_file(const char* value);
void process_all_errors(void (*fct)(error_t e, void* data), void* data);
void process_n_errors(void (*fct)(error_t e, void* data), void* data, uint32_t n);
void process_all_filtered_errors(void (*fct)(error_t e, void* data), void* data);
void process_n_filtered_errors(void (*fct)(error_t e, void* data), void* data, uint32_t n);
void clear_errors();
void drop_errors( uint32_t nb );
void clear_filtered_errors();
void print_error(const error_t * e, bool verbose);


typedef enum {
  WARNING_DEBUG = 1,
  WARNING_STRING = 2, // Special warning used to raise a string
  WARNING_FLOAT = 3, // Special warning used to raise a float
  WARNING_INT = 4, // Special warning used to raise an int32_t
  WARNING_FILE = 5, // Special warning used to raise an int32_t
  WARNING_LINE = 6, // Special warning used to raise an int32_t
  WARNING_PRINTF = 7,
  WARNING_LAG = 8,
  WARNING_ENCODER_BUSY = 9,
  WARNING_ENCODER_ERROR_ON_AS5047D = 10,
  WARNING_ENCODER_LAG = 11,
  WARNING_ENCODER_UNEXPECTED_VALUE=12,
  WARNING_MOTOR_LAG = 13
} warning_code_t;

inline const char* warning_to_string( warning_code_t w ){
  switch( w ){
    case WARNING_DEBUG :
      return "WARNING_DEBUG"; 
    case WARNING_STRING :
      return "WARNING_STRING"; 
    case WARNING_FLOAT :
      return "WARNING_FLOAT"; 
    case WARNING_INT :
      return "WARNING_INT"; 
    case WARNING_FILE :
      return "WARNING_FILE"; 
    case WARNING_LINE :
      return "WARNING_LINE"; 
    case WARNING_PRINTF :
      return "WARNING_PRINTF"; 
    case WARNING_LAG :
      return "WARNING_LAG"; 
    case WARNING_ENCODER_BUSY :
      return "WARNING_ENCODER_BUSY"; 
    case WARNING_ENCODER_ERROR_ON_AS5047D :
      return "WARNING_ENCODER_ERROR_ON_AS5047D";
    case WARNING_ENCODER_LAG :
      return "WARNING_ENCODER_LAG";
    case WARNING_MOTOR_LAG :
      return "WARNING_MOTOR_LAG";
    case WARNING_ENCODER_UNEXPECTED_VALUE :
      return "WARNING_ENCODER_UNEXPECTED_VALUE";
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
void raise_warning_float(float value);
void raise_warning_int(int32_t value);
void raise_warning_string(const char* value);
void raise_warning_file(const char* value);
void process_all_warnings(void (*fct)(warning_t e, void* data), void* data);
void process_n_warnings(void (*fct)(warning_t e, void* data), void* data, uint32_t n);
void process_all_filtered_warnings(void (*fct)(warning_t e, void* data), void* data);
void process_n_filtered_warnings(void (*fct)(warning_t e, void* data), void* data, uint32_t n);
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

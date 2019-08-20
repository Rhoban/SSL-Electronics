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
  ERROR_ENCODER_SPI_FAILURE,
} error_code_t;

typedef struct {
  error_code_t code;
  uint32_t value;
} error_t;

bool has_error();
uint32_t nb_errors();
void append_error(error_t e);
void process_all_errors(void (*fct)(error_t e, void* data), void* data);
void clear_errors();
void print_error(const error_t * e);


typedef enum {
  WARNING_LAG = 1,
  WARNING_ENCODER_BUSY = 2,
  WARNING_ENCODER_ERROR_ON_AS5047D = 3
} warning_code_t;

typedef struct {
  warning_code_t code;
  uint32_t value;
} warning_t;

bool has_warning();
uint32_t nb_warnings();
void append_warning(warning_t e);
void process_all_warnings(void (*fct)(warning_t e, void* data), void* data);
void clear_warnings();
void print_warning(const warning_t * w);


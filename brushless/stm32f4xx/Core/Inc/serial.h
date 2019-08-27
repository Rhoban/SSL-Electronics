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

#include <stdbool.h>
#include <stdint.h>

#define TERMINAL_FD 42
#define JTAG_FD 41

typedef struct {
  void (*init)();
  bool (*is_init)() ;
  uint32_t (*write)(const uint8_t*, uint32_t);
  uint32_t (*write_char)(char);
  void (*tick)();
  bool (*available)();
  char (*read)();
} serial_t;

serial_t* get_serial();

void serial_print(serial_t* serial, const char* s);
void serial_println(serial_t* serial, const char * c);
void serial_print_char(serial_t* serial, char c);
void serial_println_char(serial_t* serial, char c);
void serial_print_int(serial_t* serial, int32_t n);
void serial_println_int(serial_t* serial, int32_t n);
void serial_print_uint(serial_t* serial, uint32_t n);
void serial_println_uint(serial_t* serial, uint32_t n);

void serial_print_int_base(serial_t* serial, int32_t n, uint8_t base);
void serial_print_uint_base(serial_t* serial, uint32_t n, uint8_t base);

void serial_print_float(serial_t* serial, float f);
void serial_println_float(serial_t* serial, float f);

void serial_print_double(serial_t* serial, double d);
void serial_println_double(serial_t* serial, double d);

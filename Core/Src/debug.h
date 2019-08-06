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

//#include <stdio.h>
// Print string for debug
//#define PRINTD(format, ...) printf("d: " format " - %s : %d\n", ##__VA_ARGS__, __FILE__, __LINE__ )

#define INTERNAL_PRINT(v, name) \
  terminal_print("d: "); \
  terminal_ ## name (v); \
  terminal_print(" - " __FILE__); \
  terminal_print(":"); \
  terminal_println_int( __LINE__ );

#define INTERNAL_PRINT1(v, name) INTERNAL_PRINT(v, print_ ## name )
#define INTERNAL_PRINT2(v, name) INTERNAL_PRINT(v, print)

#define PRINTI(v) INTERNAL_PRINT1(v, int)
#define PRINTF(v) INTERNAL_PRINT1(v, float)
#define PRINTD(v) INTERNAL_PRINT1(v, double)
#define PRINTB(v) INTERNAL_PRINT1(v, bool)
#define PRINTS(v) INTERNAL_PRINT2(v, bool) // bool parameter is not used

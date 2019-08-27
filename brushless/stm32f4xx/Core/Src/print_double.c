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

#include "print_double.h"

#define CONFIGURE_TO_PRINT_DOUBLE
#include "print_real.h"

_Static_assert( DOUBLE_PRINT_SPACE == FLOAT_MAX_PRINT_SPACE, "" );

void doubling_print(
  double v, uint32_t B, uint8_t* buf, uint32_t* N, uint32_t Nmax
){
  floating_point_print(v, B, buf, N, Nmax);
}

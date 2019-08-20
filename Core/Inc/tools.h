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

// give the next id ( (id+1)%size ) when
// size is a power of 2. 
#define NEXT(id, size) ((id+1)&(size-1))
// give the prev id ( (id-1)%size ) when
// size is a power of 2. 
#define PREV(id, size) ( (id+(size-1))&(size-1))
#define RMASK(nb) ( ~( ((~0u) >> nb) << nb ) )
#define LMASK(nb) ( ~( ((~0u) << nb) >> nb ) )

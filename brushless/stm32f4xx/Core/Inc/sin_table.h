/*
    This file is part of SSL-Electronics.

    Copyright 2019 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

    SSL-Electronics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SSL-Electronics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with SSL-Electronics.  If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once
#define SIN_RESOLUTION 2048
#define SIN_PRRECISION 2.0/SIN_RESOLUTION 

float cos_table(float theta);
float sin_table(float theta);

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

void observer_init();

float observer_get_angle();
float observer_get_velocity();

void observer_encoder_tick();

void get_estimated_angle_for_next_PWM_update(float* angle, float *speed);
void observer_update_level( float velocity );
void observer_update(float angle, uint32_t mesure_time_systick);
void observer_reset();

void observer_estimate( float * speed, float* angle);
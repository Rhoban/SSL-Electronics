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

typedef enum {
  MOTOR_PWM_INITIALISATION_1=1,
  MOTOR_PWM_INITIALISATION_2=2,
  MOTOR_PWM_INITIALISATION_3=3,
  MOTOR_EMERGENCY=4
} motor_error_t;

void motor_init();
void motor_start();
void motor_emergency();
void motor_prepare_pwm();
void motor_apply_pwm();

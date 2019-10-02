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

#include <stm32f4xx_hal.h>

typedef enum {
  ENCODER_VALUE_NOT_READY = 1,
  ENCODER_DO_NOT_START = 2,
  LAG_IN_ANGLE_COMPUTATION = 3,
  ANGLE_INCREASE_IS_TOO_BIG=4
} encoder_error_t;

void encoder_init(  
  SPI_HandleTypeDef* hspi, GPIO_TypeDef*  gpio_port_cs, uint16_t gpio_pin_cs
);
void encoder_tick();
void encoder_stop();
void encoder_start();
void start_read_encoder_position();
void encoder_set_origin();

void encoder_spi_call_back();

void encoder_error_spi_call_back();

// This function is automatically called by an interruption with a given 
// priority
float encoder_get_angle();
void encoder_compute_angle();
float encoder_get_raw_angle();
float encoder_get_absolute_angle();
float get_raw_origin();

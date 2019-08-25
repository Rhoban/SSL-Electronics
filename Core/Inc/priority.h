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

#define SUB_PRIORITY 0

#define MAINBOARD_SPI_PRIORITY 2 
#define MAINBOARD_SPI_SUBPRIORITY SUB_PRIORITY

#define ENCODER_SPI_PRIORITY 2
#define ENCODER_SPI_SUBPRIORITY SUB_PRIORITY

#define ENCODER_PRIORITY 1
#define ENCODER_SUBPRIORITY SUB_PRIORITY

#define PWM_DUTY_CYCLE_PRIORITY 4
#define PWM_DUTY_CYCLE_SUBPRIORITY SUB_PRIORITY

#define COMPUTING_TASK_PRIORITY 5
#define COMPUTING_TASK_SUBPRIORITY SUB_PRIORITY

#define SYSTEM_PRIORITY 6
#define SYSTEM_SUBPRIORITY SUB_PRIORITY

#define USB_PRIORITY 7
#define USB_SUBPRIORITY SUB_PRIORITY

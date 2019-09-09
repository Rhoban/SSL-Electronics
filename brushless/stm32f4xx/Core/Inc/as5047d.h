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
#include <stdint.h>
#include <stdbool.h>

typedef enum {
  AS5047D_OK = 0,
  AS5047D_ERROR = 1,
  AS5047D_RX_PARITY_DATA = 2, // Parity bit of the reading data is not correct
  AS5047D_TX_CMD = 4, // An error in the transmission command occurs. 
  AS5047D_TX_CMD_PARITY = 8, // Parity bit is incorrect for the command 
  AS5047D_TX_CMD_ADDRESS = 16, // Invalid adress for the command
  AS5047D_TX_CMD_FRAME = 32, // This is a non compliant SPI frame.
  AS5047D_SPI_BUSY = 64, // This is a non compliant SPI frame.
  AS5047D_SPI_ERROR = 128, // This is a non compliant SPI frame.
  AS5047D_SPI_CRASH = 256, // The spi has crached ans is restarting.
  AS5047D_SPI_ABORT = 512, // The Abort procedure was a failure :'(
  AS5047D_SPI_ABORT_FAIL = 1024 // The Abort procedure was a failure :'(
} as5047d_error_t;

typedef struct {
  bool mfs_too_low; // (mfs = Magnetic field strentgh)
  bool mfs_too_high; // (mfs = Magnetic field strentgh)
  bool cordi_overflow;
  bool offset_compensation_is_ready;
  uint8_t automatic_gain_control;
} as5047d_diagnostic_t;

inline void clear_diagnostic( as5047d_diagnostic_t* diag){
  diag->mfs_too_low = false;
  diag->mfs_too_high = false;
  diag->cordi_overflow = false;
  diag->offset_compensation_is_ready = false;
  diag->automatic_gain_control = 0;
}

typedef struct {
  uint8_t pTxData[2];
  uint8_t pRxData[2];

  volatile bool is_ready;
  volatile as5047d_error_t error;

  volatile int32_t state;
  volatile uint16_t data;
  //volatile uint32_t sysclk_count_before_transmit;
  //volatile uint32_t data_sysclk_count_before_transmit;
  volatile uint32_t old_sysclk_count;
  volatile uint32_t new_sysclk_count;
  volatile uint32_t data_sysclk_count; // This is the reception time of the 
    // data given in sysclk counter.
    // To obtain the time of the mesure (in counter),
    // you need to compute :
    // data_sysclk_counter - AS5047D_TIME_PROPAGATION_sysclk(clk_period_ns, sysclk)
    // where clk_period_ns is the clock 
    // period of the spi and sysclk is the clock frequence of the 
    // microcontroller.
  
  SPI_HandleTypeDef* hspi;
  GPIO_TypeDef* gpio_port_cs;
  uint16_t gpio_pin_cs;
} as5047d_t;

void as5047d_init(  
  as5047d_t* as5047d, 
  SPI_HandleTypeDef* hspi, GPIO_TypeDef*  gpio_port_cs, uint16_t gpio_pin_cs
);

void as5047d_data_to_diagnostic(
  as5047d_t* as5047d, as5047d_diagnostic_t * diagnostic
);

inline uint16_t as5047d_data_to_angle(as5047d_t* as5047d){
  return as5047d->data;
}

void as5047d_spi_call_back(as5047d_t* as5047d);
void as5047d_error_spi_call_back(as5047d_t* as5047d);

/*
 * Start to read a dynamic angle with an error compensation.
 * 
 * This function is not blocking.
 * 
 * returns true if the process is started, and false if the device is busy
 * (a reading process is yet in progress).
 */
bool as5047d_start_reading_dynamic_angle(as5047d_t* as5047d);
bool as5047d_start_reading_diagnostic(as5047d_t* as5047d);
bool as5047d_fast_reading_dynamic_angle(as5047d_t* as5047d);

void as5047d_call_back_when_finished(as5047d_t* as5047d);

//Macro to compute the minimal time needed to compute data with as5047d module.
#define AS5047D_TL  350
#define AS5047D_TLCSN  350
#define AS5047D_PACKET_NUMBER 2
#define AS5047D_PACKET_NUMBER_WITH_FAILURE 3
#define AS5047D_DATA_SIZE 16

#define AS5047D_TIME_ONE_FRAME_ns(clk_period_ns) ( \
    AS5047D_TLCSN + AS5047D_DATA_SIZE*(clk_period_ns) + AS5047D_TL \
)

#define AS5047D_TIME_PROPAGATION_ns(clk_period_ns) ( \
  AS5047D_DATA_SIZE*(clk_period_ns) + AS5047D_TL \
)
#define AS5047D_TIME_PROPAGATION_sysclk(clk_period_ns, sysclk) ( \
  (AS5047D_TIME_PROPAGATION_ns(clk_period_ns) * ((sysclk)/1000000)) /1000 \
)

#define AS5047D_MINIMAL_TIME_COMUNICATION_ns(clk_period_ns) ( \
  AS5047D_PACKET_NUMBER * AS5047D_TIME_ONE_FRAME_ns(clk_period_ns) \
)
#define AS5047D_MINIMAL_TIME_COMUNICATION_WITH_FAILURE_ns(clk_period_ns) ( \
  AS5047D_PACKET_NUMBER_WITH_FAILURE * AS5047D_TIME_ONE_FRAME_ns(clk_period_ns) \
)
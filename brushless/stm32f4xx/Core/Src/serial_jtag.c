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

#include <serial.h>
#include <serial_jtag.h>
#include "stm32f4xx_hal.h"

#include <errors.h>

#include <queue.h>

define_and_declare_static_queue(uint8_t, jtag_buffer, JTAG_TX_DATA_SIZE)
static bool jtag_is_init = false;

bool serial_jtag_is_init(){
  return jtag_is_init;
}

void serial_jtag_init(){
  jtag_is_init = true;
};

uint32_t serial_jtag_write(const uint8_t* data, uint32_t len){
  uint32_t free_size = JTAG_TX_DATA_SIZE- jtag_buffer_size(&jtag_buffer);
  uint32_t size = (len > free_size )? free_size : len;
  for( uint32_t i=0 ; i< size; i++ ){
    jtag_buffer_append(&jtag_buffer, data[i]);
  }
  return size;
}

uint32_t serial_jtag_write_char(char c){
  return serial_jtag_write((uint8_t*)&c, 1);
}

#include "debug.h"

void serial_jtag_tick(){
  uint32_t size = jtag_buffer_size(&jtag_buffer);
  for(uint32_t i=0; i<size; i++){
    ITM_SendChar(jtag_buffer_pop(&jtag_buffer)); // We print on JTAG
  }
}

bool serial_jtag_available(){
  return false;
}

char serial_jtag_read(){
  return 0;
}

static serial_t serial_jtag = {
  .init = serial_jtag_init,
  .is_init = serial_jtag_is_init,
  .write = serial_jtag_write,
  .write_char = serial_jtag_write_char,
  .tick = serial_jtag_tick,
  .available = serial_jtag_available,
  .read = serial_jtag_read,
};


serial_t* get_serial_jtag(){
  return &serial_jtag;
}

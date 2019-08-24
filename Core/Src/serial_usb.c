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
#include <serial_usb.h>

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "debug.h"

#include <queue.h>
define_and_declare_static_queue(uint8_t, usb_queue, USB_TX_DATA_SIZE)

static uint32_t buffer_size = 0;
static uint8_t buffer[USB_TX_DATA_SIZE];
static bool usb_is_init = false;

bool usb_have_inputs(){
  return !usb_queue_is_empty(&usb_queue);
}
bool usb_output_is_full(){
  return usb_queue_is_full(&usb_queue);
}
void collect_received_data(uint8_t* buf, uint8_t len){
  usb_queue_fill(&usb_queue, buf, len);
}

/*
 * Pop all data from usb into usb_data.
 */
void usb_pop_received_data( uint8_t* usb_data, uint32_t* len){
  usb_queue_collect(&usb_queue, usb_data, len);
}


 
bool serial_usb_is_init(){
  return usb_is_init;
}

void serial_usb_init(){
  buffer_size = 0;
  usb_is_init = true;
};

uint32_t serial_usb_write(const uint8_t* data, uint32_t len){
  uint32_t free_size = USB_TX_DATA_SIZE-buffer_size;
  uint32_t size = (len > free_size )? free_size : len;
  memcpy(buffer+buffer_size, data, size);
  buffer_size += size;  
  return size;
}

uint32_t serial_usb_write_char(char c){
  return serial_usb_write((uint8_t*)&c, 1);
}

void serial_usb_tick(){
  if( buffer_size && CDC_Transmit_FS(buffer, buffer_size)==USBD_OK ){
    buffer_size = 0;
  }
}

bool serial_usb_available(){
  return !usb_queue_is_empty(&usb_queue);
}

char serial_usb_read(){
  if( !usb_queue_is_empty(&usb_queue) ){
    return usb_queue_pop(&usb_queue);
  }
  return 0;
}

static serial_t serial_usb = {
  .init = serial_usb_init,
  .is_init = serial_usb_is_init,
  .write = serial_usb_write,
  .write_char = serial_usb_write_char,
  .tick = serial_usb_tick,
  .available = serial_usb_available,
  .read = serial_usb_read,
};


serial_t* get_serial_usb(){
  return &serial_usb;
}

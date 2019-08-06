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

uint32_t queue_head;
uint32_t queue_tail;

// THE QUEUE 
//       <=  Tail ----- Head  <=   
uint8_t usb_queue[USB_RX_DATA_SIZE];

static uint32_t buffer_size = 0;
static uint8_t buffer[USB_TX_DATA_SIZE];
static bool usb_is_init = false;

inline uint32_t next_index(uint32_t id){
  _Static_assert(
    IS_POW_2(USB_RX_DATA_SIZE), "Have to be a 2^N"
  );
  return (id+1)&(USB_RX_DATA_SIZE-1);
}
inline bool queue_is_empty(){
  return queue_head == queue_tail;
}
inline bool usb_have_inputs(){
  return !queue_is_empty();
}
inline bool queue_is_full(){
  return next_index(queue_head) == queue_tail;
}
inline bool usb_output_is_full(){
  return queue_is_full();
}
inline void append_queue(uint8_t e){
  if( queue_is_full() ) return;
  usb_queue[queue_head] = e;
  queue_head = next_index(queue_head);
}
void collect_received_data(uint8_t* buf, uint8_t len){
  for( uint32_t i=0; i<len; i++ ){
    append_queue(buf[i]);   
  }
}
inline uint8_t pop_queue(){
  uint8_t tail = usb_queue[queue_tail];
  queue_tail = next_index(queue_tail);
  return tail; 
}

/*
 * Pop all data from usb into usb_data.
 */
void usb_pop_received_data( uint8_t* usb_data, uint32_t* len){
  *len = 0;
  while(!queue_is_empty()){
    usb_data[*len] = pop_queue();
    (*len)++;
  }
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
  return !queue_is_empty();
}

char serial_usb_read(){
  if( !queue_is_empty() ){
    return pop_queue();
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

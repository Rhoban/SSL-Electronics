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
#include "stm32f4xx_hal.h"
//#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "debug.h"
#include "print_float.h"
#include "print_double.h"
#include <terminal.h>

extern serial_t serial_usb;
serial_t* get_serial_usb();

serial_t* get_serial(){
  return get_serial_usb();
}

int __io_putchar(int ch){
  // get_serial()->write_char(ch); // We print on USB
  ITM_SendChar(ch); // We print on JTAG
  return ch;
}

int _write(int file, char *ptr, int len){
  int idx;
  for(idx = 0; idx < len; idx++){
    switch( file ){
      case 41 : 
        __io_putchar(*ptr++);
        break;
      case 42 :
      default:
        if(*ptr == '\n'){
          terminal_print_char('\r');
        }
        terminal_print_char(*ptr++);
    } 
  }
  return len;
}

static char eol[2] = "\r\n";

void serial_print(serial_t* serial, const char* s){
  uint32_t len = strlen(s);
  serial->write((const uint8_t*)s, len);
}

void serial_print_eol(serial_t* serial){
  serial->write((const uint8_t*)eol, sizeof(eol));
}

void serial_println(serial_t* serial, const char* s){
  serial_print(serial, s);
  serial_print_eol(serial);
}

void serial_print_char(serial_t* serial, char c){
  serial->write_char(c);
}

void serial_println_char(serial_t* serial, char c){
  serial_print_char(serial, c);
  serial_print_eol(serial);
}

void serial_print_uint_base(serial_t* serial, uint32_t n, uint8_t base){
  if(n==0){
    serial_print_char(serial, '0');
    return;
  }
  #define NB_BIT_OF_CHAR 8
  uint8_t buf[sizeof(n)*NB_BIT_OF_CHAR]; // Size max is for an integer in base 2.
  uint32_t i = 0;
  while(n > 0){
    buf[i++] = n % base;
    n /= base;
  }
  for (; i > 0; i--) {
    serial_print_char(
      serial,
      buf[i - 1] < 10 ?
      '0' + buf[i - 1] :
      'A' + buf[i - 1] - 10
    );
  }
}

void serial_print_uint(serial_t* serial, uint32_t n){
  serial_print_uint_base(serial, n, 10);
}

void serial_println_uint(serial_t* serial, uint32_t n){
  serial_print_uint(serial, n);
  serial_print_eol(serial);
}

void serial_print_int_base(serial_t* serial, int32_t n, uint8_t base){
  if (n < 0) {
      serial_print_char(serial, '-');
      n = -n;
  }
  serial_print_uint_base(serial, n, base);
}

void serial_print_int(serial_t* serial, int32_t n){
  serial_print_int_base(serial, n, 10);
}

void serial_println_int(serial_t* serial, int32_t n){
  serial_print_int(serial, n);
  serial_print_eol(serial);
}


void serial_print_float(serial_t* serial, float f){
  uint32_t size=0;
  uint8_t buf[FLOAT_PRINT_SPACE];
  floating_print(f, 10, buf, &size, FLOAT_PRINT_SPACE);
  serial_print(serial, (char*) buf);
}

void serial_println_float(serial_t* serial, float f){
  serial_print_float(serial, f);
  serial_print_eol(serial);
}

void serial_print_double(serial_t* serial, double d){

  uint32_t size=0;
  uint8_t buf[DOUBLE_PRINT_SPACE];
  doubling_print(d, 10, buf, &size, DOUBLE_PRINT_SPACE);
  serial_print(serial, (char*) buf);
}
void serial_println_double(serial_t* serial, double d){
  serial_print_double(serial, d);
  serial_print_eol(serial);
}

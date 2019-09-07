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
#include <terminal.h>
#include <printf.h>
#include <errors.h>

// 
// The function printf and fprintf from stdio.h, call the function
// _write() to print characters in a file descriptor.
// We redfine _write() to print on jtag and on usb.
// 
int _write(int file, char *ptr, int len){
  int idx;
  switch( file ){
    case USB_FILE_DESCRIPTOR :
      if(
        get_serial_usb()->write((uint8_t*)ptr, len) != len
      ){
        raise_warning(WARNING_PRINTF, PRINTF_JTAG_BUFFER_IS_FULL);
      } 
      break;
    case JTAG_FILE_DESCRIPTOR :
      if(
        get_serial_jtag()->write((uint8_t*)ptr, len) != len
      ){
        raise_warning(WARNING_PRINTF, PRINTF_JTAG_BUFFER_IS_FULL);
      }
      break;
    case TERMINAL_FILE_DESCRIPTOR :
    default:
    {
      for(idx = 0; idx < len; idx++){
        if(*ptr == '\n'){
          terminal_print_char('\r');
        }
        terminal_print_char(*ptr++);
      }
    };
  } 
  return len;
}

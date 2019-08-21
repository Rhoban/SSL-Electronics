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

#include <system.h>
#include <errors.h>
#include <terminal.h>
#include <jump_to_bootloader.h>
#include "debug.h"

void system_init(){
}

void system_emergency(){
}

void system_restart(){
  clear_errors();
  clear_warnings();
}

void system_tick(){
  if( has_error() ){
    system_emergency();
    WATCHJ(true, 2000, "E");
    WATCHT(true, 2000, "E");
  }
  if( has_warning() ){
    WATCHJ(true, 2000, "W");
    WATCHT(true, 2000, "W");
  }
}

// Overload the default filter for warnings  defined in error.c.
#include <as5047d.h>
bool filter_warning(const warning_t* w){
  #ifdef DEBUG
  if( 
    w->code == WARNING_ENCODER_ERROR_ON_AS5047D
    &&
    w->value == (AS5047D_ERROR | AS5047D_SPI_BUSY)
  ) return false;
  #else
  #endif
  return true;
}

void display_warning( warning_t e, int* cpt){
  if( *cpt > 0  ){
    (*cpt) --;
  }else{
    print_warning(&e);
  }
}

TERMINAL_COMMAND(warnings, "print and flush warnings" ){
  uint32_t nb = nb_warnings();
  if( nb==0 ){
    terminal_println("no warnings.");
  }
  if( argc > 1 ){
    terminal_println("warnings [<number of waarning>]");
    return;
  }else if( argc == 0 ){
    nb = 0;
  }else{
    nb -= atoi(argv[0]);
  }
  process_all_warnings(
    (void (*)(warning_t,void*)) display_warning, &nb
  );
}

void display_error( error_t e, int* cpt){
  if( *cpt > 0  ){
    (*cpt) --;
  }else{
    print_error(&e);
  }
}

TERMINAL_COMMAND(errors, "print and flush errors" ){
  uint32_t nb = nb_errors();
  if( nb==0 ){
    terminal_println("no errors.");
  }
  if( argc > 1 ){
    terminal_println("errors [<number of errors>]");
    return;
  }else if( argc == 0 ){
    nb = 0;
  }else{
    nb -= atoi(argv[0]);
  }
  process_all_errors(
    (void (*)(error_t,void*)) display_error, &nb
  );
}

void system_start_bootloader(){
  system_emergency();
  jump_to_bootloader();
}

TERMINAL_COMMAND(jump_to_bootloader, "Execute the bootloader")
{
  system_start_bootloader();
}

TERMINAL_COMMAND(em, "Emergency")
{
  system_emergency();
}

TERMINAL_COMMAND(restart, "Restart")
{
  system_emergency();
  system_restart();
}

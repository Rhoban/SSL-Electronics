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
#include <frequence_definitions.h>
#include <as5047d.h>

void system_init(){
}

void system_emergency(){
}

void system_restart(){
  clear_errors();
  clear_filtered_errors();
  clear_warnings();
  clear_filtered_warnings();
}

void system_tick(){
  if( has_errors() ){
    system_emergency();
    WATCHJ(true, 2000, "E");
    WATCHT(true, 2000, "E");
  }
  if( has_warnings() ){
    WATCHJ(true, 2000, "W");
    WATCHT(true, 2000, "W");
  }
}

// Redefine the filter defined in error.c.
filter_rule_t filter_error(const error_t* e){
  return TAKE_IT;
}

// Redefine the filter defined in error.c.
filter_rule_t filter_warning(const warning_t* w){
  if( w->code == WARNING_DEBUG ){
    return FILTER; //IGNORE;
  }
  return TAKE_IT;
}

void display_warning( warning_t e, int* cpt){
  if( *cpt > 0  ){
    (*cpt) --;
  }else{
    print_warning(&e, true);
  }
}

TERMINAL_COMMAND(warn, "print and flush warnings" ){
  uint32_t total = nb_warnings();
  uint32_t nb = nb_warnings();
  if( nb==0 ){
    terminal_println("no warnings.");
  }
  if( argc > 1 ){
    terminal_println("warn [<number of waarning>]");
    return;
  }else if( argc == 0 ){
    nb = 0;
  }else{
    nb -= atoi(argv[0]);
  }
  bool is_full = warning_queue_is_full();
  process_n_warnings(
    (void (*)(warning_t,void*)) display_warning, &nb, total
  );
  if( is_full ){
    terminal_println("... and more recent warnings.");
  }
  if( has_filtered_warnings() ){
    terminal_print("filtered warnings : ");
    terminal_println_int(nb_filtered_warnings());
  }else{
    terminal_println("no filtered warnings.");
  }
}

TERMINAL_COMMAND(fwarn, "print and flush warnings" ){
  uint32_t total = nb_filtered_warnings();
  uint32_t nb = nb_filtered_warnings();
  if( nb==0 ){
    terminal_println("no filtered warnings.");
  }
  if( argc > 1 ){
    terminal_println("fwarn [<number of warning>]");
    return;
  }else if( argc == 0 ){
    nb = 0;
  }else{
    nb -= atoi(argv[0]);
  }
  bool is_full = filtered_warning_queue_is_full(); 
  process_n_filtered_warnings(
    (void (*)(warning_t,void*)) display_warning, &nb, total
  );
  if( is_full ){
    terminal_println("... and more recent filtered warnings.");
  }
}

void display_error( error_t e, int* cpt){
  if( *cpt > 0  ){
    (*cpt) --;
  }else{
    print_error(&e, true);
  }
}

TERMINAL_COMMAND(err, "print and flush errors" ){
  uint32_t total = nb_errors();
  uint32_t nb = nb_errors();
  if( nb==0 ){
    terminal_println("no errors.");
  }
  if( argc > 1 ){
    terminal_println("err [<number of errors>]");
    return;
  }else if( argc == 0 ){
    nb = 0;
  }else{
    nb -= atoi(argv[0]);
  }
  bool is_full = error_queue_is_full();
  process_n_errors(
    (void (*)(error_t,void*)) display_error, &nb, total
  );
  if( is_full ){
    terminal_println("... and more recent errors.");
  }
  if( has_filtered_errors() ){
    terminal_print("filtered errors : ");
    terminal_println_int(nb_filtered_errors());
  }else{
    terminal_println("no filtered errors.");
  }
}

TERMINAL_COMMAND(ferr, "print and flush filtered errors" ){
  uint32_t total = nb_filtered_errors();
  uint32_t nb = nb_filtered_errors();
  if( nb==0 ){
    terminal_println("no filtered errors.");
  }
  if( argc > 1 ){
    terminal_println("ferr [<number of errors>]");
    return;
  }else if( argc == 0 ){
    nb = 0;
  }else{
    nb -= atoi(argv[0]);
  }
  bool is_full = filtered_error_queue_is_full();
  process_n_filtered_errors(
    (void (*)(error_t,void*)) display_error, &nb, total
  );
  if( is_full ){
    terminal_println("... and more recent errors.");
  }
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

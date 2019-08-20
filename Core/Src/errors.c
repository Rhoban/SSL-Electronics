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

#include <errors.h>
#include <queue.h>
#include <terminal.h>

#define NUMBER_OF_ERRORS 16

define_and_declare_queue(error_t, errors, NUMBER_OF_ERRORS)

/*
 * This function return true if we want to keep the warning.
 */
 __attribute__((weak)) bool filter_error(const error_t* e){
  return true;
}


bool has_error(){
  return ! errors_is_empty(&errors);
}
uint32_t nb_errors(){
  return errors_size(&errors);
}
void append_error(error_t e){
  if( filter_error(&e) ){
    errors_append(&errors, e);
  }
}
void process_all_errors(void (*fct)(error_t e, void* data), void* data){
  errors_process(&errors, fct, data);
}
void clear_errors(){
  errors_clear(&errors);
}

void print_error(const error_t * e){
  terminal_print("E ");
  terminal_print_int( e->code );
  terminal_print("(");
  terminal_print_int( e->value );
  terminal_println(")");
}

#define NUMBER_OF_WARNINGS 32

define_and_declare_queue(warning_t, warnings, NUMBER_OF_WARNINGS)

/*
 * This function return true if we want to keep the error.
 */
 __attribute__((weak)) bool filter_warning(const warning_t* w){
  if( 
    w->code == WARNING_ENCODER_ERROR_ON_AS5047D
    &&
    w->value == 3
  ) return false;
  return true;
}

bool has_warning(){
  return ! warnings_is_empty(&warnings);
}
uint32_t nb_warnings(){
  return warnings_size(&warnings);
}
void append_warning(warning_t w){
  if( filter_warning(&w) ){
    warnings_append(&warnings, w);
  }
}
void process_all_warnings(void (*fct)(warning_t e, void* data), void* data){
  warnings_process(&warnings, fct, data);
}
void clear_warnings(){
  errors_clear(&errors);
}

void print_warning(const warning_t * w){
  terminal_print("W ");
  terminal_print_int( w->code );
  terminal_print("(");
  terminal_print_int( w->value );
  terminal_println(")");
}


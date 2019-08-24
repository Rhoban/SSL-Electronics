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

define_and_declare_static_queue(error_t, errors, NUMBER_OF_ERRORS)
define_and_declare_static_queue(error_t, filtered_errors, NUMBER_OF_ERRORS)

/*
 * This function return true if we want to keep the warning.
 */
 __attribute__((weak)) bool filter_error(const error_t* e){
  return true;
}

bool has_errors(){
  return ! errors_is_empty(&errors);
}
bool has_filtered_errors(){
  return ! filtered_errors_is_empty(&filtered_errors);
}
uint32_t nb_errors(){
  return errors_size(&errors);
}
uint32_t nb_filtered_errors(){
  return filtered_errors_size(&filtered_errors);
}
bool error_queue_is_full(){
  return errors_is_full(&errors);
}
bool filtered_error_queue_is_full(){
  return filtered_errors_is_full(&filtered_errors);
}
void append_error(error_t e){
  if( filter_error(&e) ){
    errors_append(&errors, e);
  }else{
    filtered_errors_append(&filtered_errors, e);
  }
}
void raise_error(error_code_t code, uint32_t value){
  error_t e = {code, value};
  append_error(e);
}
void process_all_errors(void (*fct)(error_t e, void* data), void* data){
  errors_process(&errors, fct, data);
}
void process_all_filtered_errors(void (*fct)(error_t e, void* data), void* data){
  filtered_errors_process(&filtered_errors, fct, data);
}
void clear_errors(){
  errors_clear(&errors);
}
void clear_filtered_errors(){
  filtered_errors_clear(&filtered_errors);
}

void print_error(const error_t * e, bool verbose){
  terminal_print("E ");
  if( verbose ){
    terminal_print(error_to_string(e->code));
  }
  terminal_print(" ");
  terminal_print_int( e->code );
  terminal_print("(");
  terminal_print_int( e->value );
  terminal_println(")");
}

#define NUMBER_OF_WARNINGS 32

define_and_declare_static_queue(warning_t, warnings, NUMBER_OF_WARNINGS)
define_and_declare_static_queue(warning_t, filtered_warnings, NUMBER_OF_ERRORS)

/*
 * This function return true if we want to keep the warning.
 */
__attribute__((weak)) bool filter_warning(const warning_t* w){
  return true;
}

bool has_warnings(){
  return ! warnings_is_empty(&warnings);
}
bool has_filtered_warnings(){
  return ! filtered_warnings_is_empty(&filtered_warnings);
}
uint32_t nb_warnings(){
  return warnings_size(&warnings);
}
uint32_t nb_filtered_warnings(){
  return filtered_warnings_size(&filtered_warnings);
}
bool warning_queue_is_full(){
  return warnings_is_full(&warnings);
}
bool filtered_warning_queue_is_full(){
  return filtered_warnings_is_full(&filtered_warnings);
}
void append_warning(warning_t w){
  if( filter_warning(&w) ){
    warnings_append(&warnings, w);
  }else{
    filtered_warnings_append(&filtered_warnings, w);
  }
}
void raise_warning(warning_code_t code, uint32_t value){
  warning_t w = {code, value};
  append_warning(w);
}
void process_all_warnings(void (*fct)(warning_t e, void* data), void* data){
  warnings_process(&warnings, fct, data);
}
void process_all_filtered_warnings(void (*fct)(warning_t e, void* data), void* data){
  filtered_warnings_process(&filtered_warnings, fct, data);
}
void clear_warnings(){
  warnings_clear(&warnings);
}
void clear_filtered_warnings(){
  filtered_warnings_clear(&filtered_warnings);
}

void print_warning(const warning_t * w, bool verbose){
  terminal_print("W ");
  if( verbose ){
    terminal_print(warning_to_string(w->code));
  }
  terminal_print(" ");
  terminal_print_int( w->code );
  terminal_print("(");
  terminal_print_int( w->value );
  terminal_println(")");
}


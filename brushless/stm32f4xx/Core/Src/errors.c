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
#define NUMBER_OF_ERRORS 32

define_and_declare_static_queue(error_t, errors, NUMBER_OF_ERRORS)
define_and_declare_static_queue(error_t, filtered_errors, NUMBER_OF_ERRORS)

/*
 * This function return true if we want to keep the warning.
 */
 __attribute__((weak)) filter_rule_t filter_error(const error_t* e){
  return TAKE_IT;
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
  switch( filter_error(&e) ){
    case FILTER : 
      filtered_errors_append(&filtered_errors, e);
    case IGNORE :
      break;
    default:
      errors_append(&errors, e);
  }
}
void raise_error(error_code_t code, uint32_t value){
  error_t e = {code, value};
  append_error(e);
}
void raise_error_float(float value){
  union {
    float from;
    uint32_t to;
  } converter = { .from = value };
  raise_error(ERROR_FLOAT, converter.to);
}
void raise_error_int(int32_t value){
  union {
    int32_t from;
    uint32_t to;
  } converter = { .from = value };
  raise_error(ERROR_INT, converter.to);
}
void raise_error_string(const char* value){
  union {
    const char * from;
    uint32_t to;
  } converter = { .from = value };
  raise_error(ERROR_STRING, converter.to);
}
void raise_error_file(const char* value){
  union {
    const char * from;
    uint32_t to;
  } converter = { .from = value };
  raise_error(ERROR_FILE, converter.to);
}
void process_all_errors(void (*fct)(error_t e, void* data), void* data){
  errors_process(&errors, fct, data);
}
void process_n_errors(void (*fct)(error_t e, void* data), void* data, uint32_t n){
  errors_nprocess(&errors, fct, data, n);
}
void process_all_filtered_errors(void (*fct)(error_t e, void* data), void* data){
  filtered_errors_process(&filtered_errors, fct, data);
}
void process_n_filtered_errors(void (*fct)(error_t e, void* data), void* data, uint32_t n){
  filtered_errors_nprocess(&filtered_errors, fct, data, n);
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
    terminal_print("(");
    terminal_print_int( e->code );
    terminal_print("), ");
  }else{
    terminal_print_int( e->code );
    terminal_print(", ");
  }
  switch( e->code ){
    case ERROR_STRING :
    case ERROR_FILE :
      terminal_println( (char*)e->value );
      break;
    case ERROR_FLOAT : {
        union {
          uint32_t from;
          float to;
        } converter = { .from = e->value };
        terminal_println_float(converter.to );
      }
      break;
    case ERROR_INT : {
        union {
          uint32_t from;
          int32_t to;
        } converter = { .from = e->value };
        terminal_println_int(converter.to);
      }
      break;
    default :
      terminal_println_int( e->value );
  }
}

#define NUMBER_OF_WARNINGS 32

define_and_declare_static_queue(warning_t, warnings, NUMBER_OF_WARNINGS)
define_and_declare_static_queue(warning_t, filtered_warnings, NUMBER_OF_ERRORS)

/*
 * This function return true if we want to keep the warning.
 */
__attribute__((weak)) filter_rule_t filter_warning(const warning_t* w){
  return TAKE_IT;
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
  switch( filter_warning(&w) ){
    case FILTER:
      filtered_warnings_append(&filtered_warnings, w);
      break;
    case IGNORE:
      break;
    default:
      warnings_append(&warnings, w);
  }
}
void raise_warning(warning_code_t code, uint32_t value){
  warning_t w = {code, value};
  append_warning(w);
}
void raise_warning_float(float value){
  union {
    float from;
    uint32_t to;
  } converter = { .from = value };
  raise_warning(WARNING_FLOAT, converter.to);
}
void raise_warning_int(int32_t value){
  union {
    int32_t from;
    uint32_t to;
  } converter = { .from = value };
  raise_warning(WARNING_INT, converter.to);
}
void raise_warning_string(const char* value){
  union {
    const char* from;
    uint32_t to;
  } converter = { .from = value };
  raise_warning(WARNING_STRING, converter.to);
}
void raise_warning_file(const char* value){
  union {
    const char* from;
    uint32_t to;
  } converter = { .from = value };
  raise_warning(WARNING_FILE, converter.to);
}

void process_all_warnings(void (*fct)(warning_t e, void* data), void* data){
  warnings_process(&warnings, fct, data);
}
void process_n_warnings(void (*fct)(warning_t e, void* data), void* data, uint32_t n){
  warnings_nprocess(&warnings, fct, data, n);
}
void process_all_filtered_warnings(void (*fct)(warning_t e, void* data), void* data){
  filtered_warnings_process(&filtered_warnings, fct, data);
}
void process_n_filtered_warnings(void (*fct)(warning_t e, void* data), void* data, uint32_t n){
  filtered_warnings_nprocess(&filtered_warnings, fct, data, n);
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
    terminal_print("(");
    terminal_print_int( w->code );
    terminal_print("), ");
  }else{
    terminal_print_int( w->code );
    terminal_print(", ");
  }
  switch( w->code ){
    case WARNING_STRING: 
    case WARNING_FILE: 
      terminal_println( (char*) w->value );
      break;
    case WARNING_FLOAT : {
        union {
          uint32_t from;
          float to;
        } converter = { .from = w->value };
        terminal_println_float( converter.to );
      }
      break;
    case WARNING_INT : {
        union {
          uint32_t from;
          int32_t to;
        } converter = { .from = w->value };
        terminal_println_int( converter.to );
      }
      break;
    default:
      terminal_println_int( w->value );
  }
}
TERMINAL_COMMAND(flush_err, "flush errors"){
  clear_errors();
}
TERMINAL_COMMAND(flush_warn, "flush warning"){
  clear_warnings();
}

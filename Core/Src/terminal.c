/*
 * Copyright (C) 2015-2019 Grégoire Passault <gregoire.passault@u-bordeaux.fr>
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

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <terminal.h>
#include "debug.h"

static bool disabled = false;

/**
 * Control bar structure
 */
typedef struct
{
    int32_t min;
    int32_t max;
    int32_t pos;
    bool escape;
} terminal_bar_t;

/**
 * Global variables terminal
 */

static char terminal_buffer[TERMINAL_BUFFER_SIZE];

static bool terminal_last_ok = false;
static uint32_t terminal_last_pos = 0;
static uint32_t terminal_pos = 0;

static const struct terminal_command *terminal_commands[TERMINAL_MAX_COMMANDS];

static uint32_t terminal_command_count = 0;

struct terminal_io_t_ {
  bool silent;
  serial_t* io;
};

static struct terminal_io_t_ terminalIO;

static inline void terminal_write_char(char c){
    if (!terminalIO.silent) {
        terminalIO.io->write_char(c);
    }
}
static inline void terminal_write(const char* buf, uint32_t len){
    if (!terminalIO.silent) {
        terminalIO.io->write((const uint8_t*)buf, len);
    }
}

static inline void println(const char* s){
  if (!terminalIO.silent) serial_println(terminalIO.io, s);
}
static inline void print(const char* s){
  if (!terminalIO.silent) serial_print(terminalIO.io, s);
}
static inline void print_int(int32_t i){
  if (!terminalIO.silent) serial_print_int(terminalIO.io, i);
}
static inline void println_int(int32_t i){
  if (!terminalIO.silent) serial_println_int(terminalIO.io, i);
}
static inline void print_float(float f){
  if (!terminalIO.silent) serial_print_float(terminalIO.io, f);
}
static inline void println_float(float f){
  if (!terminalIO.silent) serial_println_float(terminalIO.io, f);
}
static inline void print_double(double f){
  if (!terminalIO.silent) serial_print_double(terminalIO.io, f);
}
static inline void println_double(double f){
  if (!terminalIO.silent) serial_println_double(terminalIO.io, f);
}
static inline void print_char(char c){
  if (!terminalIO.silent) serial_print_char(terminalIO.io, c);
}

void terminal_println(const char* s){
  println(s);
};
void terminal_print(const char* s){
  print(s);
}
void terminal_print_int(int32_t i){
  print_int(i);
}
void terminal_println_int(int32_t i){
  println_int(i);
}
void terminal_print_float(float f){
  print_float(f);
}
void terminal_println_float(float f){
  println_float(f);
}
void terminal_print_double(double f){
  print_double(f);
}
void terminal_println_double(double f){
  println_double(f);
}
void terminal_print_char(char c){
  print_char(c);
}




static terminal_bar_t terminal_bar;

static bool terminal_echo_mode = true;

/**
 * Registers a command
 */
void terminal_register(const struct terminal_command *command)
{
    terminal_commands[terminal_command_count++] = command;
}

static void displayHelp(bool parameter)
{
    char buffer[1000];
    uint32_t i;

    if (parameter) {
        println("Available parameters:");
    } else {
        println("Available commands:");
    }
    println("");

    for (i=0; i<terminal_command_count; i++) {
        const struct terminal_command *command = terminal_commands[i];

        if (command->parameter != parameter) {
            continue;
        }

        int32_t namesize = strlen(command->name);
        int32_t descsize = strlen(command->description);
        int32_t typesize = (command->parameter_type == NULL) ? 0 : strlen(command->parameter_type);

        memcpy(buffer, command->name, namesize);
        buffer[namesize++] = ':';
        buffer[namesize++] = '\r';
        buffer[namesize++] = '\n';
        buffer[namesize++] = '\t';
        memcpy(buffer+namesize, command->description, descsize);
        if (typesize) {
            buffer[namesize+descsize++] = ' ';
            buffer[namesize+descsize++] = '(';
            memcpy(buffer+namesize+descsize, command->parameter_type, typesize);
            buffer[namesize+descsize+typesize++] = ')';
        }
        buffer[namesize+descsize+typesize++] = '\r';
        buffer[namesize+descsize+typesize++] = '\n';
        terminal_write(buffer, namesize+descsize+typesize);
    }
}

/**
 * Internal helping command
 */
TERMINAL_COMMAND(help, "Displays the help about commands")
{
    displayHelp(false);
}

void terminal_params_show()
{
    uint32_t i;

    for (i=0; i<terminal_command_count; i++) {
        const struct terminal_command *command = terminal_commands[i];

        if (command->parameter) {
            command->command(0, NULL);
        }
    }
}

/**
 * Display available parameters
 */
TERMINAL_COMMAND(params, "Displays the available parameters. Usage: params [show]")
{
    if (argc && strcmp(argv[0], "show")==0) {
        terminal_params_show();
    } else {
        displayHelp(true);
    }
}

/**
 * Switch echo mode
 */
TERMINAL_COMMAND(echo, "Switch echo mode. Usage echo [on|off]")
{
    if ((argc == 1 && strcmp("on", argv[0])) || terminal_echo_mode == false) {
        terminal_echo_mode = true;
        println("Echo enabled");
    } else {
        terminal_echo_mode = false;
        println("Echo disabled");
    }
}

/**
 * Write the terminal prompt
 */
void terminal_prompt()
{
    print(TERMINAL_PROMPT);
}

const struct terminal_command *terminal_find_command(char *command_name, uint32_t command_name_length)
{
    uint32_t i;

    for (i=0; i<terminal_command_count; i++) {
        const struct terminal_command *command = terminal_commands[i];

        if (strlen(command->name) == command_name_length && strncmp(terminal_buffer, command->name, command_name_length) == 0) {
            return command;
        }
    }

    return NULL;
}

/***
 * Executes the given command with given parameters
 */
bool terminal_execute(char *command_name, uint32_t command_name_length, 
        uint32_t argc, char **argv)
{
    uint32_t i;
    const struct terminal_command *command;

    // Try to find and execute the command
    command = terminal_find_command(command_name, command_name_length);
    if (command != NULL) {
        command->command(argc, argv);
    }

    // If it fails, try to parse the command as an allocation (a=b)
    if (command == NULL) {
        for (i=0; i<command_name_length; i++) {
            if (command_name[i] == '=') {
                command_name[i] = '\0';
                command_name_length = strlen(command_name);
                command = terminal_find_command(command_name, command_name_length);

                if (command && command->parameter) {
                    argv[0] = command_name+i+1;
                    argv[1] = NULL;
                    argc = 1;
                    command->command(argc, argv);
                } else {
                    command = NULL;
                }

                if (!command) {
                    print("Unknown parameter: ");
                    terminal_write(command_name, command_name_length);
                    println("");
                    return false;
                }
            }
        }
    }

    // If it fails again, display the "unknown command" message
    if (command == NULL) {
        print("Unknown command: ");
        terminal_write(command_name, command_name_length);
        println("");
        return false;
    }

    return true;
}

/***
 * Process the receive buffer to parse the command and executes it
 */
void terminal_process()
{
    char *saveptr;
    uint32_t command_name_length;

    uint32_t argc = 0;
    char* argv[TERMINAL_MAX_ARGUMENTS+1];

    println("");

    strtok_r(terminal_buffer, " ", &saveptr);
    while (
            (argv[argc] = strtok_r(NULL, " ", &saveptr)) != NULL && 
            argc < TERMINAL_MAX_ARGUMENTS
          ) {
        *(argv[argc]-1) = '\0';
        argc++;
    }

    if (saveptr != NULL) {
        *(saveptr - 1) = ' ';
    }

    command_name_length = strlen(terminal_buffer);

    if (command_name_length > 0) {
        terminal_last_ok = terminal_execute(terminal_buffer, command_name_length, argc, argv);
    } else {
        terminal_last_ok = false;
    }

    terminal_last_pos = terminal_pos;
    terminal_pos = 0;
    terminal_prompt();
}

/**
 * Save the Serial object globaly
 */
void terminal_init(serial_t *serial)
{
    terminalIO.io = serial;
    if (serial && serial->is_init()) {
      terminalIO.silent = false;
    }else{
      terminalIO.silent = true;
    }
    terminal_enable();
    terminal_prompt();
    terminal_bar.escape = true;
}

void terminal_reset()
{
    terminal_pos = 0;
    terminal_last_pos = 0;
    terminal_buffer[0] = '\0';
    terminal_last_ok = false;
    terminal_prompt();
}

/**
 * Stops the terminal
 */
void terminal_disable()
{
    disabled = true;
}

void terminal_enable()
{
    terminal_last_ok = false;
    disabled = false;
}

static inline bool terminal_has_io(){
  return terminalIO.io;
}

/**
 * Ticking the terminal, this will cause lookup for characters 
 * and eventually a call to the process function on new lines
 */
void terminal_tick()
{
    if (disabled || !terminal_has_io()) {
        return;
    }

    char c;
    uint8_t input;

    while (terminalIO.io->available()) {
        input = terminalIO.io->read();
        c = (char)input;
        if (c == '\0') {
            continue;
        }

        //Return key
        if (c == '\r' || c == '\n') {
            if (terminal_pos == 0 && terminal_last_ok) { 
                // If the user pressed no keys, restore the last 
                // command and run it again
                uint32_t i;
                for (i=0; i<terminal_last_pos; i++) {
                    if (terminal_buffer[i] == '\0') {
                        terminal_buffer[i] = ' ';
                    }
                }
                terminal_pos = terminal_last_pos;
            }
            terminal_buffer[terminal_pos] = '\0';
            terminal_process();
            //Back key
        } else if (c == '\x7f') {
            if (terminal_pos > 0) {
                terminal_pos--;
                print("\x8 \x8");
            }
            //Special key
        } else if (c == '\x1b') {
            while (!terminalIO.io->available());
            terminalIO.io->read();
            while (!terminalIO.io->available());
            terminalIO.io->read();
            //Others
        } else {
            terminal_buffer[terminal_pos] = c;
            if (terminal_echo_mode) {
                print_char(c);
            }

            if (terminal_pos < TERMINAL_BUFFER_SIZE-1) {
                terminal_pos++;
            }
        }
    }
}

void terminal_bar_init(int32_t min, int32_t max, int32_t pos)
{
    if (min < max && pos >= min && pos <= max) {
        terminal_bar.min = min;
        terminal_bar.max = max;
        terminal_bar.pos = pos;
        terminal_bar.escape = false;
    }
}

int32_t terminal_bar_tick()
{
    int32_t step = terminal_bar.max - terminal_bar.min;
    step = step / TERMINAL_BAR_STEP;
    if (step == 0) {
        step = 1;
    }

    if (terminal_bar.escape == false) {
        print("\x8\x8\x8\x8    ");
        print("\r");
        print("Bar: ");
        print_int(terminal_bar.pos);
        print(" | ");
        print_int(terminal_bar.min);
        print(" ");
        for (int32_t i=terminal_bar.min;i<=terminal_bar.max;i+=step) {
            if (i <= terminal_bar.pos) {
                print("=");
            } else {
                print(".");
            }
        }
        print(" ");
        print_int(terminal_bar.max);

        while (true) {
            while (!terminalIO.io->available());
            int32_t controlKey = 0;
            bool specialKey = false;
            char input = (char)terminalIO.io->read();
            //Detect control characters
            if (input == '\x1b') {
                specialKey = true;
            } else if (input == '^') {
                while (!terminalIO.io->available());
                input = (char)terminalIO.io->read();
                if (input == '[') {
                    specialKey = true;
                }
            }
            //Arrow keys
            if (specialKey) {
                char code[2];
                while (!terminalIO.io->available());
                code[0] = terminalIO.io->read();
                while (!terminalIO.io->available());
                code[1] = terminalIO.io->read();
                //Left
                if (code[0] == '[' && code[1] == 'D') {
                    controlKey = -1;
                    //Right
                } else if (code[0] == '[' && code[1] == 'C') {
                    controlKey = 1;
                }
                //Alias keys for arrows left
            } else if (input == 'h') {
                controlKey = -1;
                //Alias keys for arrows right
            } else if (input == 'l') {
                controlKey = 1;
            }
            //Apply control
            if (controlKey == 1) {
                terminal_bar.pos += step;
                break;
            } else if (controlKey == -1) {
                terminal_bar.pos -= step;
                break;
            } else if (input == '\r' || input == '\n') {
                println("");
                terminal_bar.escape = true;
                break;
            }
        }
        if (terminal_bar.pos < terminal_bar.min) {
            terminal_bar.pos = terminal_bar.min;
        } else if (terminal_bar.pos > terminal_bar.max) {
            terminal_bar.pos = terminal_bar.max;
        }
    } 

    return terminal_bar.pos;
}

bool terminal_bar_escaped()
{
    return terminal_bar.escape;
}


void terminal_silent(bool silent)
{
    terminalIO.silent = silent;
}

float terminal_atof(char *str)
{
    int32_t sign = (str[0]=='-') ? -1 : 1;
    float f = atoi(str);
    char *savePtr;
    strtok_r(str, ".", &savePtr);
    char *floatPart = strtok_r(NULL, ".", &savePtr);
    if (floatPart != NULL) {
        float divide = 1;
        for (char *tmp=floatPart; *tmp!='\0'; tmp++) {
            divide *= 10; 
        }   
        f += sign*atoi(floatPart)/divide;
    }

    return f;
}


void terminal_print_hexa(uint8_t* buf, uint32_t n){
  uint8_t first=0, last=0;
  for( uint32_t i=0; i<n; i++ ){
    first = buf[i] >> 4;
    last = buf[i] & 0x0F;
    print_char( 'x' );
    print_char( (first < 10 ? '0' : 'A'-10) + first );
    print_char( (last < 10 ? '0' : 'A'-10) + last );
  }
}
void terminal_println_hexa(uint8_t* buf, uint32_t n){
  terminal_print_hexa(buf,1);
  println("");
}

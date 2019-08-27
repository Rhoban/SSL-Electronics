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

#pragma once

#include <stdlib.h>
#include "serial.h"
#include <stdint.h>

/**
 * Maximum length of a command line
 * and its argument
 */
#define TERMINAL_BUFFER_SIZE 64

/**
 * Maximum number of command arguments
 */
#define TERMINAL_MAX_ARGUMENTS 8

/**
 * Maximum number of commands
 * which ca be registered
 */
#define TERMINAL_MAX_COMMANDS 100

/**
 * The number of terminal bar step
 */
#define TERMINAL_BAR_STEP 50

/**
 * Terminal prompt
 */
#define TERMINAL_PROMPT "$ "

/**
 * To defined a new command, use the MACRO
 * TERMINAL_COMMAND(name, description)
 *
 * @param name : the name of the command, without space, without quotes
 * @param description : the description of the command, with quote
 *
 * use terminal_print() to print on output
 * use the variables arc and argv to get the string of given arguments
 *
 * Example :
 *
 * TERMINAL_COMMAND(hello, "Print a friendly warming welcoming message")
 * {
 *     if (argc > 0) {
 *         terminal_print("Hello ");
 *         terminal_println(argv[0]);
 *         terminal_print("Other params: ");
 *         for (uint32_t i=1;i<argc;i++) {
 *             terminal_print(argv[i]);
 *             terminal_print(" ");
 *         }
 *         terminal_println("");
 *     } else {
 *         terminal_println("Hello world");
 *     }
 * }
 */

struct terminal_io_t_;
typedef struct terminal_io_t_ terminal_io_t;

/**
 * Initializing terminal
 * This function have to be called before using
 * terminal_tick()
 *
 * @param com : you have to provide to terminal_init
 * an initialized instance of USBSerial or HardwareSerial
 */
void terminal_init(serial_t *serial);

/**
 * Resets the terminal
 */
void terminal_reset();

/**
 * Activate or desactivate the terminal
 */
void terminal_disable();
void terminal_enable();

/**
 * Terminal ticking
 * Call this function in your main loop 
 * to fetch serial port and handle terminal
 */
void terminal_tick();

/**
 * Initialize the termanl bar structure
 * @param min : minimum value
 * @param max : maximum value
 * @param pos : initial position
 */
void terminal_bar_init(int32_t min, int32_t max, int32_t pos);

/**
 * Terminal bar ticking
 * Fetch the serial port and handle the bar
 * @return the bar position
 */
int32_t terminal_bar_tick();

/**
 * Returns true if the bar session
 * have been interrupted
 */
bool terminal_bar_escaped();

/**
 * Mute the terminal
 */
void terminal_silent(bool silent);

void terminal_println(const char* s);
void terminal_print(const char* s);
void terminal_print_int(int32_t i);
void terminal_print_int32(int32_t i);
void terminal_print_bool(int32_t i);
void terminal_println_bool(int32_t i);
void terminal_println_int(int32_t i);
void terminal_println_int32(int32_t i);
void terminal_print_float(float f);
void terminal_println_float(float f);
void terminal_print_double(double f);
void terminal_println_double(double f);
void terminal_print_char(char c);
void terminal_print_hexa(uint8_t* buf, uint32_t n);
void terminal_println_hexa(uint8_t* buf, uint32_t n);

/**
 * ----------------------------------------------------------------------------
 * ----------------------------------------------------------------------------
 * ----------------------------------------------------------------------------
 */

/**
 * Prototype of a terminal command
 */
typedef void terminal_command_fn(uint32_t argc, char *argv[]);

/**
 * A command definition for the terminal
 */
struct terminal_command
{
    char *name;
    char *description;
    terminal_command_fn *command;
    bool parameter;
    char *parameter_type;
};

/**
 * Registers a command
 */
void terminal_register(const struct terminal_command *command);

#define TERMINAL_COMMAND_INTERNAL(name, description, parameter, parameterType) terminal_command_fn terminal_command_ ## name; \
    \
    char terminal_command_name_ ## name [] = #name; \
    char terminal_command_description_ ## name [] = description; \
    \
    struct terminal_command terminal_command_definition_ ## name = { \
        terminal_command_name_ ## name , \
        terminal_command_description_ ## name , \
        terminal_command_ ## name, \
        parameter, \
        parameterType \
    }; \
    \
    __attribute__((constructor)) \
    void terminal_command_init_ ## name () {  \
        terminal_register(&terminal_command_definition_ ## name ); \
    } \
    \
    void terminal_command_ ## name (uint32_t argc, char *argv[])

#define TERMINAL_COMMAND(name, description) \
    TERMINAL_COMMAND_INTERNAL(name, description, false, NULL)

#define TERMINAL_PARAMETER(name, description, startValue, type, type_name, conversion) \
    volatile static type name = startValue; \
    char terminal_parameter_type_ ## name [] = #type; \
    \
    TERMINAL_COMMAND_INTERNAL(name, description, true, terminal_parameter_type_ ## name) \
    { \
        type g; \
        if (argc) { \
            g = conversion(argv[0]); \
            name = g; \
        } \
        terminal_print( #name ); \
        terminal_print("="); \
        terminal_println_ ## type_name ( name ); \
    }

float terminal_atof(char *str);

#define TERMINAL_PARAMETER_FLOAT(name, description, startValue) \
    TERMINAL_PARAMETER(name, description, startValue, float, float, terminal_atof)

#define TERMINAL_PARAMETER_DOUBLE(name, description, startValue) \
    TERMINAL_PARAMETER(name, description, startValue, double, double, terminal_atof)

#define TERMINAL_PARAMETER_INT(name, description, startValue) \
    TERMINAL_PARAMETER(name, description, startValue, int32_t, int, atoi)

#define TERMINAL_PARAMETER_BOOL(name, description, startValue) \
    TERMINAL_PARAMETER(name, description, startValue, bool, int, (bool)atoi)


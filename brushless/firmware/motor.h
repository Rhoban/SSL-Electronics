#ifndef _MOTOR_H
#define _MOTOR_H

#include <stdint.h>

/**
 * Initializes the motor pins
 */
void motor_init();

/**
 * Sets the target PWM value for the motor [0-3000]
 */
void motor_set(int value);

/**
 * Ticks the motor
 */
void motor_tick();

#endif

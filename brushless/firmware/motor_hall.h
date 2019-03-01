#ifndef _MOTOR_HALL_H
#define _MOTOR_HALL_H

#include <stdint.h>

/**
 * Initializes the motor pins
 */
void motor_hall_init();

/**
 * Sets the target PWM value for the motor [0-3000]
 */
void motor_hall_set(bool enable, int value);

/**
 * Ticks the motor
 */
void motor_hall_tick();

#endif

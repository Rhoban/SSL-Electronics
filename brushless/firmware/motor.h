#ifndef _MOTOR_H
#define _MOTOR_H

#include <stdint.h>

/**
 * Initializes the motor pins
 */
void motor_init();

void servo_set_flag_1();

/**
 * Sets the target PWM value for the motor [0-3000]
 */
void motor_set(bool enable, int value);

/**
 * Ticks the motor
 */
void motor_tick(bool irq = false);

void motor_dbg();

#endif

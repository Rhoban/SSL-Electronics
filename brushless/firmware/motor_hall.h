#ifndef _MOTOR_HALL_H
#define _MOTOR_HALL_H

#include <stdint.h>

/**
 * Initializes the motor pins
 */
void motor_hall_init();

void motor_hall_irq();

/**
 * Sets the target PWM value for the motor [0-3000]
 */
void motor_hall_set(bool enable, int value);

/**
 * Ticks the motor
 */
void motor_hall_tick();

bool motor_hall_is_on();

void enable_motor_hall(bool);

void set_hall_motor_pwm( int );

#define PWM_HALL_SUPREMUM 3000
#endif

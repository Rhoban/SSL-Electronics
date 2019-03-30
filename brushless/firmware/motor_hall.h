#ifndef _MOTOR_HALL_H
#define _MOTOR_HALL_H

#include <stdint.h>

#define PWM_SUPREMUM 3000
#define PWM_MIN ((PWM_MIN_PERCENT*PWM_SUPREMUM)/100)
#define PWM_MAX ((PWM_MAX_PERCENT*PWM_SUPREMUM)/100)

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

bool motor_hall_is_on();
#endif

#ifndef _MOTOR_H
#define _MOTOR_H

#include <stdint.h>

/**
 * Initializes the motor pins
 */
void motor_init();

void servo_set_flag_1();
void start_to_tare_motor();
void set_motor_speed_pid( float speed_p, float speed_i, float speed_d );
void set_motor_speed_consign( float speed );
bool motor_is_set();
void reset_motor();
float motor_get_speed();
int motor_get_pwm();

float servo_get_speed();

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

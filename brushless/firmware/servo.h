#ifndef _BL_SERVO_H
#define _BL_SERVO_H

#include "ssl.h"
#include "motor.h"
#include <stdint.h>

/**
 * Initializing servo system
 */
void servo_init();

/**
 * Ticks servo
 */
void servo_tick();

/**
 * Sets the servo value, enables or not the motor control, target speed is
 * in [turn/s]
 */
void servo_set(bool enable, float targetSpeed, int16_t pwm=0);

/**
 * Current speed [turn/s]
 */
float servo_get_speed();

/**
 * Current PWM output [-3000 to 3000]
 */
int servo_get_pwm();

/**
 * Sets the PID parameters
 */
void servo_set_pid(float kp, float ki, float kd);

void servo_set_speed_consign( float speed );

void servo_set_flag();
#endif

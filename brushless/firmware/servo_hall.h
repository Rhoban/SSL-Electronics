#ifndef _BL_SERVO_HALL_H
#define _BL_SERVO_HALL_H

#include <stdint.h>


/**
 * Initializing servo system
 */
void servo_hall_init();

/**
 * Ticks servo
 */
void servo_hall_tick();
void servo_hall_set_flag();

/**
 * Sets the servo value, enables or not the motor control, target speed is
 * in [turn/s]
 */
void servo_hall_set(bool enable, float targetSpeed, int16_t pwm=0);

/**
 * Sets the PID parameters
 */
void servo_hall_set_pid(float kp, float ki, float kd);

/**
 * Current speed [turn/s]
 */
float servo_hall_get_speed();

/**
 * Current PWM output [-3000 to 3000]
 */
int servo_hall_get_pwm();

void servo_hall_emergency();
void servo_hall_stop();

void reset_hall_error();

#endif

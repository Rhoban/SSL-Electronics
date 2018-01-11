#ifndef _BL_SERVO_H
#define _BL_SERVO_H

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
void servo_set(bool enable, float targetSpeed);

/**
 * Sets the PID parameters
 */
void servo_set_pid(float kp, float ki, float kd);

/**
 * Current speed [turn/s]
 */
float servo_get_speed();

#endif

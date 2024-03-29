#ifndef _BL_SERVO_FOC_H
#define _BL_SERVO_FOC_H

#include <stdint.h>

/**
 * Initializing servo system
 */
void servo_foc_init();

/**
 * Ticks servo
 */
void servo_foc_tick();

/**
 * Sets the servo value, enables or not the motor control, target speed is
 * in [turn/s]
 */
void servo_foc_set(bool enable, float targetSpeed, int16_t pwm=0);

/**
 * Current speed [turn/s]
 */
float servo_foc_get_speed();

/**
 * Current PWM output [-3000 to 3000]
 */
int servo_foc_get_pwm();


/**
 * Sets the PID parameters
 */
void servo_foc_set_pid(float kp, float ki, float kd);

void servo_foc_emergency();
void servo_foc_stop();

void servo_set_speed_consign_foc( float speed );

void set_open_loop( bool value );
bool is_open_loop();

void servo_foc_set_manual_speed(bool value);
void reset_asservissement();

/*
 * Set angular position
 *
 * input : angular
 * scale : ONE_TURN_THETA
 */
void set_foc_angular_position(int theta_csg);

void servo_foc_register_get_theta_csg(
    int (*fct)( int update_frequence)
);

#endif

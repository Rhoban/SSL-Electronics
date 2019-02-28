#ifndef _MOTOR_H
#define _MOTOR_H

#include <stdint.h>

#define IS_POW_2(x) (x && ((x & (x - 1)) == 0))

#define REFERENCE_VOLTAGE 1024 // Do not change
#define HALF_REFERENCE_VOLTAGE 512 // Do not change
_Static_assert( IS_POW_2(REFERENCE_VOLTAGE), "");
_Static_assert( REFERENCE_VOLTAGE == 2*HALF_REFERENCE_VOLTAGE , "");

/*
 * 1 turn : [ 0 - 2^14 [ = [ 0 - 16384 [ 
 */
#define ONE_TURN_THETA 0x4000

// SCALE : 1
#define PWM_FREQUENCE 2400 // Hz
// SCALE : 1
#define MOTOR_FREQUENCE 240 
// SCALE : 1
#define MOTOR_SUB_SAMPLE 10
_Static_assert( MOTOR_SUB_SAMPLE>=2, "Shanon !");
_Static_assert(
    PWM_FREQUENCE == MOTOR_FREQUENCE*MOTOR_SUB_SAMPLE, ""
);


/**
 * Initializes the motor pins
 */
void motor_init();

/*
 *   VALUE : [INT_MIN/ONE_TURN_THETA, INT_MAX/ONE_TURN_THETA]
 *   SCALE : ONE_TURN_THETA
 */
int rotor_angle();

/**
 * Ticks the motor
 */
void motor_tick();

bool motor_is_tared();

void direct_quadrature_voltage_set(int vd, int vq );

void reset_serv_flag();
bool get_serv_flag();

void motor_set(bool enable, int value);
#endif

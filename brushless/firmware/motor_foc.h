#ifndef _MOTOR_FOC_H
#define _MOTOR_FOC_H

#include "tools.h"

#define REFERENCE_VOLTAGE 1024 // Do not change
#define HALF_REFERENCE_VOLTAGE 512 // Do not change
static_assert( IS_POW_2(REFERENCE_VOLTAGE), "");
static_assert( REFERENCE_VOLTAGE == 2*HALF_REFERENCE_VOLTAGE , "");


#define PWM_SUPREMUM 750
#define PWM_MIN ((PWM_MIN_PERCENT*PWM_SUPREMUM)/100)
#define PWM_MAX ((PWM_MAX_PERCENT*PWM_SUPREMUM)/100)


#define CLOCK_FREQUENCE 72000000 // Hz
#define PWM_PRESCALE_FACTOR 1
#define PWM_OVERFLOW PWM_SUPREMUM

#define PWM_FREQUENCE 96000 // Hz
static_assert(
    PWM_FREQUENCE * PWM_PRESCALE_FACTOR * PWM_OVERFLOW == CLOCK_FREQUENCE,
    ""
);
#define MOTOR_FREQUENCE 800
#define SWAP_PWM_FREQUENCE 1
#define SERVO_UPDATE 120

#define MOTOR_UPDATE_FREQUENCE 3200
#define MOTOR_UPDATE 30
static_assert(MOTOR_UPDATE<=SERVO_UPDATE, "");
static_assert(PWM_FREQUENCE == MOTOR_UPDATE_FREQUENCE*MOTOR_UPDATE, "");
static_assert(SERVO_UPDATE%MOTOR_UPDATE == 0, "");

static_assert(
    PWM_FREQUENCE == MOTOR_FREQUENCE*SERVO_UPDATE, ""
);
static_assert( SERVO_UPDATE>=2, "Shanon !");


/**
 * Initializes the motor pins
 */
void motor_foc_init();

/*
 *   VALUE : [INT_MIN/ONE_TURN_THETA, INT_MAX/ONE_TURN_THETA]
 *   SCALE : ONE_TURN_THETA
 */
int rotor_angle();

/**
 * Ticks the motor
 */
void motor_foc_tick();

bool motor_is_tared();

void direct_quadrature_voltage_set(int vd, int vq );

void reset_serv_flag();
bool get_serv_flag();

void motor_foc_set(bool enable, int value);

void launch_tare_motor_foc();
bool motor_foc_is_on();

void register_update_theta( int (* fct)(int) );
void set_fixed_theta(bool value);


void motor_irq();
  
void enable_motor_foc(bool);
#endif

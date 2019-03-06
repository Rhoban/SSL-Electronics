#ifndef _MOTOR_FOC_H
#define _MOTOR_FOC_H

#define IS_POW_2(x) (x && ((x & (x - 1)) == 0))

#define REFERENCE_VOLTAGE 1024 // Do not change
#define HALF_REFERENCE_VOLTAGE 512 // Do not change
static_assert( IS_POW_2(REFERENCE_VOLTAGE), "");
static_assert( REFERENCE_VOLTAGE == 2*HALF_REFERENCE_VOLTAGE , "");

/*
 * 1 turn : [ 0 - 2^14 [ = [ 0 - 16384 [ 
 */
#define ONE_TURN_THETA 0x4000

#define CLOCK_FREQUENCE 72000000 // Hz
#define PWM_PRESCALE_FACTOR 1
#define PWM_OVERFLOW 3000

#define PWM_FREQUENCE 24000 // Hz
static_assert(
    PWM_FREQUENCE * PWM_PRESCALE_FACTOR * PWM_OVERFLOW == CLOCK_FREQUENCE,
    ""
);
#define MOTOR_FREQUENCE 1200
#define SWAP_PWM_FREQUENCE 1
#define MOTOR_SUB_SAMPLE 20


static_assert( MOTOR_SUB_SAMPLE>=2, "Shanon !");
static_assert(
    PWM_FREQUENCE == MOTOR_FREQUENCE*MOTOR_SUB_SAMPLE, ""
);


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

bool motor_foc_is_on();
#endif

#include "servo.h"
#include "motor.h"
#include "servo_foc.h"
#include "servo_hall.h"
#include "servo_hybrid.h"
#include "security.h"
#include "terminal.h"
#include <stdint.h>

void servo_init()
{
    #if defined(USE_HYBRID)
    servo_hybrid_init();
    #elif defined(USE_HALL)
    servo_hall_init();
    #else
    servo_foc_init();
    #endif
}

void servo_tick()
{
    if (security_get_error() != SECURITY_NO_ERROR) {
        motor_set(false, 0);
    } else {
        #if defined(USE_HYBRID)
        servo_hybrid_tick();
        #elif defined(USE_HALL)
        servo_hall_tick();
        #elif defined(USE_FOC)
        servo_foc_tick();
        #endif
    }
}

void servo_set(bool enable, float targetSpeed, int16_t pwm){
    #if defined(USE_HYBRID)
    servo_hybrid_set(enable, targetSpeed, pwm);
    #elif defined(USE_HALL)
    servo_hall_set(enable, targetSpeed, pwm);
    #elif defined(USE_FOC)
    servo_foc_set(enable, targetSpeed, pwm);
    #endif
}

void servo_set_speed_consign( float speed ){
    #if defined(USE_HYBRID)
    servo_hybrid_set_speed_consign(speed);
    #elif defined(USE_HALL)
    #elif defined(USE_FOC)
    servo_set_speed_consign_foc(speed);
    #endif
}

float servo_get_speed(){
    #if defined(USE_HYBRID)
    return servo_hybrid_get_speed();
    #elif defined(USE_HALL)
    return servo_hall_get_speed();
    #elif defined(USE_FOC)
    return servo_foc_get_speed();
    #endif
}

int servo_get_pwm(){
    #if defined(USE_HYBRID)
    return servo_hybrid_get_pwm();
    #elif defined(USE_HALL)
    return servo_hall_get_pwm();
    #elif defined(USE_FOC)
    return servo_foc_get_pwm();
    #endif
}

void servo_set_pid(float kp, float ki, float kd){
    #if defined(USE_HYBRID)
    servo_hybrid_set_pid(kp, ki, kd);
    #elif defined(USE_HALL)
    servo_hall_set_pid(kp, ki, kd);
    #elif defined(USE_FOC)
    servo_foc_set_pid(kp, ki, kd);
    #endif
}

void servo_set_flag(){
    #if defined(USE_HYBRID)
    servo_hybrid_set_flag();
    #elif defined(USE_HALL)
    servo_hall_set_flag();
    #elif defined(USE_FOC)
    #endif
}

void servo_emergency(){
    #if defined(USE_HYBRID)
    servo_hybrid_emergency();
    #elif defined(USE_HALL)
    servo_hall_emergency();
    #elif defined(USE_FOC)
    servo_foc_emergency();
    #endif
}

void servo_stop(){
    #if defined(USE_HYBRID)
    servo_hybrid_stop();
    #elif defined(USE_HALL)
    servo_hall_stop();
    #elif defined(USE_FOC)
    servo_foc_stop();
    #endif
}

TERMINAL_COMMAND(em, "Emergency")
{
    servo_emergency();
}

TERMINAL_COMMAND(st, "Stop")
{
    servo_stop();
}

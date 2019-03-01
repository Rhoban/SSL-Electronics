#include "servo.h"
#include "motor.h"
#include "servo_foc.h"
#include "servo_hall.h"
#include "security.h"
#include "terminal.h"
#include <stdint.h>

void servo_init()
{
    #ifndef DO_NOT_USE_HALL
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
        #ifndef DO_NOT_USE_HALL
        servo_hall_tick();
        #else
        servo_foc_tick();
        #endif
    }
}

void servo_set(bool enable, float targetSpeed, int16_t pwm){
    #ifndef DO_NOT_USE_HALL
    servo_hall_set(enable, targetSpeed, pwm);
    #else
    servo_foc_set(enable, targetSpeed, pwm);
    #endif
}

float servo_get_speed(){
    #ifndef DO_NOT_USE_HALL
    return servo_hall_get_speed();
    #else
    return servo_foc_get_speed();
    #endif
}

int servo_get_pwm(){
    #ifndef DO_NOT_USE_HALL
    return servo_hall_get_pwm();
    #else
    return servo_foc_get_pwm();
    #endif
}

void servo_set_pid(float kp, float ki, float kd){
    #ifndef DO_NOT_USE_HALL
    servo_hall_set_pid(kp, ki, kd);
    #else
    servo_foc_set_pid(kp, ki, kd);
    #endif
}

void servo_set_flag(){
    #ifndef DO_NOT_USE_HALL
    servo_hall_set_flag();
    #else
    #endif
}

void servo_emergency(){
    #ifndef DO_NOT_USE_HALL
    servo_hall_emergency();
    #else
    servo_foc_emergency();
    #endif
}

void servo_stop(){
    #ifndef DO_NOT_USE_HALL
    servo_hall_stop();
    #else
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

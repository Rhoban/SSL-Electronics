#include "motor.h"
#include "motor_foc.h"
#include "motor_hall.h"

bool motor_is_on(){
    #ifndef DO_NOT_USE_HALL
    return motor_hall_is_on();
    #else
    return motor_foc_is_on();
    #endif
}

void motor_init()
{
    #ifndef DO_NOT_USE_HALL
    motor_hall_init();
    #else
    motor_foc_init();
    #endif
}

void motor_set(bool enable, int value)
{
    #ifndef DO_NOT_USE_HALL
    motor_hall_set(enable,value);
    #else
    motor_foc_set(enable,value);
    #endif
}

void motor_tick()
{
    #ifndef DO_NOT_USE_HALL
    motor_hall_tick();
    #else
    motor_foc_tick();
    #endif
}

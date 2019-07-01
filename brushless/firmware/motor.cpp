#include "motor.h"
#include "motor_foc.h"
#include "motor_hall.h"
#include "motor_hybrid.h"

bool motor_is_on(){
    #if defined(USE_HYBRID)
        return motor_hybrid_is_on();
    #elif defined(USE_HALL)
        return motor_hall_is_on();
    #elif defined(USE_FOC)
        return motor_foc_is_on();
    #endif
}

void motor_init()
{
    #if defined(USE_HYBRID)
        motor_hybrid_init();
    #elif defined(USE_HALL)
        motor_hall_init();
    #elif defined(USE_FOC)
        motor_foc_init();
    #endif
}

void motor_set(bool enable, int value)
{
    #if defined(USE_HYBRID)
        motor_hybrid_set(enable,value);
    #elif defined(USE_HALL)
        // motor_hall_set(enable,value);
    #elif defined(USE_FOC)
        motor_foc_set(enable,value);
    #endif
}

void motor_tick()
{
    #if defined(USE_HYBRID)
        motor_hybrid_tick();
    #elif defined(USE_HALL)
        motor_hall_tick();
    #elif defined(USE_FOC)
        motor_foc_tick();
    #endif
}

void launch_tare_motor(){
    #if defined(USE_HYBRID)
        launch_tare_motor_hybrid();
    #elif defined(USE_HALL)
    #elif defined(USE_FOC)
        launch_tare_motor_foc();
    #endif
}

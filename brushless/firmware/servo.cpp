#include "servo.h"
#include "motor.h"
#include "servo_foc.h"
#include "servo_hall.h"
#include "servo_hybrid.h"
#include "security.h"
#include "terminal.h"
#include <stdint.h>
#include "encoder.h"
#include "security.h"
#include "errors.h"

void servo_init()
{
    #if defined(USE_HYBRID)
    servo_hybrid_init();
    #elif defined(USE_HALL)
    servo_hall_init();
    #elif defined(USE_FOC)
    servo_foc_init();
    #else
    static_assert(false,"");
    #endif
}

static int last_warning = 0;

void display_warning(){
    int warning = security_get_warning();  
    int error = security_get_error();  
    if( warning != SECURITY_NO_WARNING || error != SECURITY_NO_ERROR ){
        int val = millis();
        if( val - last_warning > 4000 ){
            if( warning != SECURITY_NO_WARNING ){
                terminal_io()->println( driver_warning(warning) );
                security_set_warning( SECURITY_NO_WARNING );
                // encoder_print_errors();
            }
            if( error != SECURITY_NO_ERROR ){
                //terminal_io()->print("E ");
                terminal_io()->print(error);
                terminal_io()->println( driver_error(error) );
            }
            last_warning = val;
        }
    }
}

void servo_tick()
{
    if (security_get_error() != SECURITY_NO_ERROR) {
        motor_set(false, 0);
    } else {
        display_warning();

        #if defined(USE_HYBRID)
        servo_hybrid_tick();
        #elif defined(USE_HALL)
        servo_hall_tick();
        #elif defined(USE_FOC)
        servo_foc_tick();
        #else
        static_assert(false,"");
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
    #else
    static_assert(false,"");
    #endif
}

void servo_set_speed_consign( float speed ){
    #if defined(USE_HYBRID)
    servo_hybrid_set_speed_consign(speed);
    #elif defined(USE_HALL)
    #elif defined(USE_FOC)
    servo_set_speed_consign_foc(speed);
    #else
    static_assert(false,"");
    #endif
}

float servo_get_speed(){
    #if defined(USE_HYBRID)
    return servo_hybrid_get_speed();
    #elif defined(USE_HALL)
    return servo_hall_get_speed();
    #elif defined(USE_FOC)
    return servo_foc_get_speed();
    #else
    static_assert(false,"");
    #endif
}

int servo_get_pwm(){
    #if defined(USE_HYBRID)
    return servo_hybrid_get_pwm();
    #elif defined(USE_HALL)
    return servo_hall_get_pwm();
    #elif defined(USE_FOC)
    return servo_foc_get_pwm();
    #else
    static_assert(false,"");
    #endif
}

void servo_set_pid(float kp, float ki, float kd){
    #if defined(USE_HYBRID)
    servo_hybrid_set_pid(kp, ki, kd);
    #elif defined(USE_HALL)
    servo_hall_set_pid(kp, ki, kd);
    #elif defined(USE_FOC)
    servo_foc_set_pid(kp, ki, kd);
    #else
    static_assert(false,"");
    #endif
}

void servo_set_flag(){
    #if defined(USE_HYBRID)
    servo_hybrid_set_flag();
    #elif defined(USE_HALL)
    servo_hall_set_flag();
    #elif defined(USE_FOC)
    #else
    static_assert(false,"");
    #endif
}

void servo_emergency(){
    #if defined(USE_HYBRID)
    servo_hybrid_emergency();
    #elif defined(USE_HALL)
    servo_hall_emergency();
    #elif defined(USE_FOC)
    servo_foc_emergency();
    #else
    static_assert(false,"");
    #endif
}

void servo_stop(){
    #if defined(USE_HYBRID)
    servo_hybrid_stop();
    #elif defined(USE_HALL)
    servo_hall_stop();
    #elif defined(USE_FOC)
    servo_foc_stop();
    #else
    static_assert(false,"");
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

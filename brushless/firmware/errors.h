#ifndef _DRIVERS_ERROR_H
#define _DRIVERS_ERROR_H

#include "security.h"

static inline const char *driver_error(int code)
{
    if (code == SECURITY_CURRENT_MAX) {
        return "Current max";
    } else if (code == SECURITY_CURRENT_LIMIT) {
        return "Current limit";
    } else if (code == SECURITY_HALL_FREEZE) {
        return "Hall freeze";
    } else if (code == SECURITY_HALL_MISSING) {
        return "Hall missing";
    } else if (code == SECURITY_ENCODER_MISSING) {
        return "Encoder missing";
    } else if (code == SECURITY_ENCODER_FAILURE) {
        return "Encoder failure";
    } else if ( code == SECURITY_PWM_MIN ){
        return "PWM command too small";
    } else if ( code == SECURITY_PWM_MAX ){
        return "PWM command too big";
    }
    return "?";
}

static inline const char *driver_warning(int code)
{
    if ( code == WARNING_MOTOR_LAG ){
        return "PWM motor LAG";
    } else if( code == WARNING_INVALID_RANGE ){
        return "Invalide range";
    } else if( code == WARNING_SERVO_LAG ){
        return "PWM servo LAG";
    }
    return "?";
}

#endif

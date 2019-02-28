#ifndef _DRIVERS_ERROR_H
#define _DRIVERS_ERROR_H

#include "security.h"

static inline const char *driver_error(int code)
{
    if (code == SECURITY_CURRENT_MAX) {
        return "1Current max";
    } else if (code == SECURITY_CURRENT_LIMIT) {
        return "2Current limit";
    } else if (code == SECURITY_HALL_FREEZE) {
        return "3Hall freeze";
    } else if (code == SECURITY_HALL_MISSING) {
        return "4Hall missing";
    } else if (code == SECURITY_ENCODER_MISSING) {
        return "5Encoder missing";
    } else if (code == SECURITY_ENCODER_FAILURE) {
        return "6Encoder failure";
    } else if ( code == SECURITY_PWM_MIN ){
        return "7PWM command too small";
    } else if ( code == SECURITY_PWM_MAX ){
        return "8PWM command too big";
    }else if ( code=SECURITY_NO_PHASE_IS_ON_THE_MASS ){
        return "9No phase is on the mass";
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

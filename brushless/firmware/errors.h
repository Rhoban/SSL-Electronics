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
    } else if (code == SECURITY_POSOTION_OUT_OF_LIMITS ){
        return "Angulat position reachs the limits";
    } else if (code == SECURITY_SPEED_LIMITS ){
        return "Angular velocity reachs the limit";
    }

    return "?";
}

#endif

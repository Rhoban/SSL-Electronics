#ifndef _SECURITY_H
#define _SECURITY_H

typedef enum {
    SECURITY_NO_ERROR = 0,
    SECURITY_HALL_FREEZE = 1,
    SECURITY_CURRENT_LIMIT = 2,
    SECURITY_CURRENT_MAX = 3,
    SECURITY_HALL_MISSING = 4,
    SECURITY_ENCODER_MISSING = 5,
    SECURITY_ENCODER_FAILURE = 6,
    SECURITY_POSOTION_OUT_OF_LIMITS = 7,
    SECURITY_SPEED_LIMITS = 8,
} SecurityError;

void security_set_error(SecurityError type);

SecurityError security_get_error();

#endif

#ifndef _SECURITY_H
#define _SECURITY_H

typedef enum {
    SECURITY_NO_ERROR = 0,
    SECURITY_HALL_FREEZE = 1,
    SECURITY_CURRENT_LIMIT = 2,
    SECURITY_CURRENT_MAX = 3
} SecurityError;

void security_set_error(SecurityError type);

SecurityError security_get_error();

#endif

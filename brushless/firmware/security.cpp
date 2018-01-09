#include <stdlib.h>
#include <terminal.h>
#include "security.h"
#include "servo.h"
#include "motor.h"

static SecurityError security_error = SECURITY_NO_ERROR;

void security_set_error(SecurityError type)
{
    if (security_error == SECURITY_NO_ERROR || type == SECURITY_NO_ERROR) {
        motor_set(0);
        security_error = type;
    }
}

SecurityError security_get_error()
{
    return security_error;
}

TERMINAL_COMMAND(err, "Error")
{
    if (security_error == SECURITY_NO_ERROR) {
        terminal_io()->println("No error");
    } else if (security_error == SECURITY_CURRENT_MAX) {
        terminal_io()->println("Error: Current max");
    } else if (security_error == SECURITY_CURRENT_LIMIT) {
        terminal_io()->println("Error: Current limit");
    } else if (security_error == SECURITY_HALL_FREEZE) {
        terminal_io()->println("Error: Hall freeze");
    }
}

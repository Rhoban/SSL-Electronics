#include <stdlib.h>
#include <terminal.h>
#include "security.h"
#include "servo.h"
#include "motor.h"
#include "errors.h"

static SecurityError security_error = SECURITY_NO_ERROR;

void security_set_error(SecurityError type)
{
    if (security_error == SECURITY_NO_ERROR || type == SECURITY_NO_ERROR) {
        motor_set(false, 0);
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
    } else {
        terminal_io()->print("Error: ");
        terminal_io()->println(driver_error(security_erro));
    }
}

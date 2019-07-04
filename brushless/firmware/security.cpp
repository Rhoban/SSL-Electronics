#include <stdlib.h>
#include <terminal.h>
#include "security.h"
#include "servo.h"
#include "errors.h"
#include "hybrid.h"

static SecurityError security_error = SECURITY_NO_ERROR;
static SecurityWarning security_warning = SECURITY_NO_WARNING;

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

void security_set_warning(SecurityWarning type)
{
    security_warning = type;
}

SecurityWarning security_get_warning()
{
    return security_warning;
}

TERMINAL_COMMAND(err, "Error")
{
    if (security_error == SECURITY_NO_ERROR) {
        terminal_io()->println("No error");
    } else {
        terminal_io()->print("Error: ");
        terminal_io()->println(driver_error(security_error));
    }
    if (security_warning == SECURITY_NO_WARNING) {
        terminal_io()->println("No warning");
    } else {
        terminal_io()->print("Warning: ");
        terminal_io()->println(driver_warning(security_warning));
    }
}

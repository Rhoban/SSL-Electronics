#include "hardware.h"
#ifdef ENCODER_NONE
#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include "encoder.h"

void encoder_init()
{
}

bool encoder_read()
{
    return true;
}

uint32_t encoder_value()
{
    return 0;
}

TERMINAL_COMMAND(cnt, "Cnt debug")
{
    terminal_io()->println("There is no encoder");
}

bool encoder_is_present()
{
    return true;
}

bool encoder_is_ok()
{
    return true;
}

#endif

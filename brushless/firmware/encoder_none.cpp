#include "hardware.h"
#ifdef ENCODER_NONE
#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include "servo.h"
#include "encoder.h"

void encoder_irq()
{
    servo_set_flag();
}

static void init_timer()
{
    HardwareTimer timer(4);

    // Configuring timer
    timer.pause();
    timer.setPrescaleFactor(72);
    timer.setOverflow(1000); // 1Khz

    timer.setChannel4Mode(TIMER_OUTPUT_COMPARE);
    timer.setCompare(TIMER_CH4, 1);
    timer.attachCompare4Interrupt(encoder_irq);

    timer.refresh();
    timer.resume();
}

void encoder_init()
{
    init_timer();
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

#include <stdlib.h>
#include <wirish/wirish.h>
#include <libmaple/iwdg.h>
#include <terminal.h>
#include <main.h>
#include <series/gpio.h>
#include <watchdog.h>
#include "encoder.h"
#include "current.h"
#include "motor.h"
#include "com.h"

/**
 * Setup function
 */
void setup()
{
    // Initalizing communication
    com_init();

    // Initializng current  sensor
    current_init();

    // Initalizing encoder
    encoder_init();

    // Initalizing motor
    motor_init();

    // Starting the watchdog
    watchdog_start(WATCHDOG_14MS);

    terminal_init(&SerialUSB);
}

/**
 * Loop function
 */
void loop()
{
    // Feeding watchdog
    watchdog_feed();

    // Updating motor phases, this is also done in the hall pin interrupt but
    // it seems safe to do it often anyway
    motor_tick();

    // Updating current sensor value
    current_tick();

    // Ticking the terminal
    terminal_tick();
}

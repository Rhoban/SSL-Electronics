#include <stdlib.h>
#include <wirish/wirish.h>
#include <libmaple/iwdg.h>
#include <terminal.h>
#include <main.h>
#include <series/gpio.h>
#include <watchdog.h>
#include "drivers.h"
#include "com.h"

/**
 * Setup function
 */
void setup()
{
    // Initalizng com
    com_init();

    // Initalizing drivers
    drivers_init();

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

    // Com
    com_tick();

    // Ticking the terminal
    terminal_tick();
}

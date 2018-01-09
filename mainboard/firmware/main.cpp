#include <stdlib.h>
#include <wirish/wirish.h>
#include <libmaple/iwdg.h>
#include <terminal.h>
#include <main.h>
#include <series/gpio.h>
#include <watchdog.h>
#include "drivers.h"
#include "com.h"
#include "buzzer.h"

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

    buzzer_init();

    if (com_is_all_ok() && drivers_is_all_ok()) {
        buzzer_play(MELODY_BOOT);
    } else {
        buzzer_play(MELODY_WARNING);
    }

    terminal_init(&SerialUSB);
}

/**
 * Loop function
 */
void loop()
{
    digitalWrite(BOARD_LED_PIN, LOW);
    pinMode(BOARD_LED_PIN, OUTPUT);

    // Feeding watchdog
    watchdog_feed();

    // Com
    com_tick();

    // Buzzer
    buzzer_tick();

    // Drivers
    drivers_tick();

    // Ticking the terminal
    terminal_tick();
}

TERMINAL_COMMAND(diag, "Diagnostic")
{
    drivers_diagnostic();
    com_diagnostic();
}

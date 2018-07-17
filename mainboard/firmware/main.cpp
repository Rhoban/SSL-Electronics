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
#include "kicker.h"
#include "voltage.h"
#include "ir.h"
#include "infos.h"
#include "kinematic.h"
#include "mux.h"
#include "infos.h"
#include "odometry.h"

/**
 * Setup function
 */
void setup()
{
    init();
    RCC_BASE->APB1ENR &= ~RCC_APB1ENR_USART2EN;

    digitalWrite(BOARD_LED_PIN, LOW);
    pinMode(BOARD_LED_PIN, OUTPUT);

    // Can be used to set the robot id
    //infos_set(5, false);

    // Multiplexer
    mux_init();

    // Initalizng com
    com_init();

    // Initalizing drivers
    drivers_init();

    // Kicker
    kicker_init();

    // Buzzer
    buzzer_init();

    // IR
    ir_init();

    // Voltage measure
    voltage_init();

    if (com_is_all_ok() && drivers_is_all_ok()) {
        // buzzer_play(MELODY_BOOT);
    } else {
        // buzzer_play(MELODY_WARNING);
    }

    terminal_init(&SerialUSB);

    // Infos
    infos_init();

    // Starting the watchdog
    watchdog_start(WATCHDOG_14MS);

    // Reiniting com
    com_init();


    odometry_init();
}

// Benchmaking main loop
int avg = 0;
int n = 0;
TERMINAL_COMMAND(bl, "")
{
    terminal_io()->println(avg/(n-1));
    avg = 0;
    n = 0;
}

/**
 * Loop function
 */
void loop()
{
    // Benchmarking main loop
    static int last = micros();
    if (n < 100) {
        int loop = micros() - last;
        last = micros();
        if (n != 0) {
            avg += loop;
        }
        n += 1;
    }

    // Feeding watchdog
    watchdog_feed();

    // Com
    com_tick();

    // Buzzer
    buzzer_tick();

    // Drivers
    drivers_tick();

    // Kick
    kicker_tick();

    // Voltage
    voltage_tick();

    // Kinematic
    kinematic_tick();

    // IR
    ir_tick();

    // Ticking the terminal
    terminal_tick();

}

TERMINAL_COMMAND(diag, "Diagnostic")
{
    drivers_diagnostic();
    com_diagnostic();
    ir_diagnostic();
}

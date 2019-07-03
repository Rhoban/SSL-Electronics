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

    pinMode(BOARD_LED_PIN, OUTPUT);
    digitalWrite(BOARD_LED_PIN, LOW);

    // Can be used to set the robot id
//    infos_set(-1, false);

//To set in master
    // infos_set(-1, false);


    // Multiplexer
    mux_init();
    // Buzzer
    buzzer_init();
    buzzer_play(MELODY_BEGIN);
    buzzer_wait_play();





    // delay_us(3600000);

    // Initalizng com
    com_init();
    delay_us(1000000);
    buzzer_beep(523,50);
    buzzer_wait_play();
    // Initalizing drivers
    drivers_init();
    delay_us(1000000);
    buzzer_beep(659,50);
    buzzer_wait_play();
    // Kicker
    kicker_init();
    delay_us(1000000);
    buzzer_beep(784,50);
    buzzer_wait_play();

    // IR
    ir_init();
    delay_us(1000000);
    // Voltage measure
    voltage_init();

    if (com_is_all_ok() ) { // && drivers_is_all_ok()) {
      // buzzer_play(MELODY_BEETHOVEN);
      buzzer_play(MELODY_BOOT);

    } else {
      buzzer_play(MELODY_WARNING);
    }

    terminal_init(&SerialUSB);

    // Infos
    infos_init();

    // Reiniting com
    com_init();

    // Starting the watchdog
    watchdog_start(WATCHDOG_58MS);
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

#if 0
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
#endif

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

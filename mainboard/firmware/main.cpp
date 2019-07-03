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

/*****************************************************************************/
/* Uncomment JUST when you want to define an ID. If not comment those lines !*/
/*                                                                           */
    // Can be used to set the robot id
    // infos_set(6, false);

    //To set in master for Match
    // infos_set(-1, false);

    //To set in master for Developers
    // infos_set(-2, false);
/*                                                                           */
/*****************************************************************************/

    // Multiplexer
    mux_init();
    // Buzzer
    buzzer_init();
    delay_us(1000);
    int hall1=mux_sample(HALL1_ADDR);
    delay_us(10000);
    int hall2=mux_sample(HALL2_ADDR);
    delay_us(10000);
    int hall3=mux_sample(HALL3_ADDR);
    delay_us(10000);
    int hall4=mux_sample(HALL4_ADDR);




    bool developer_mode=false;

    if(hall1>2000&&hall2>2000&&hall3>2000&&hall4>2000) //no magnet
      developer_mode=true;


    if(developer_mode)
    {
      buzzer_beep(C5,50);
      buzzer_wait_play();
      delay_us(50000);
      buzzer_wait_play();
      buzzer_beep(C5,50);
      buzzer_wait_play();
      delay_us(50000);
      buzzer_wait_play();

    }
    else{
      //THE ID
      buzzer_beep(C6,50);
      buzzer_wait_play();
      delay_us(100000);
      buzzer_wait_play();

    }

    delay_us(200000);


    // delay_us(3600000);

    // Initalizng com
    com_init(developer_mode);
    delay_us(800000);
    if(developer_mode)
      buzzer_beep(C6,50);
    else
      buzzer_beep(C5,50);
    buzzer_wait_play();
    // Initalizing drivers
    drivers_init();
    delay_us(800000);

    if(developer_mode)
      buzzer_beep(G5,50);
    else
      buzzer_beep(E5,50);

    buzzer_wait_play();
    // Kicker
    kicker_init();
    delay_us(800000);


    if(developer_mode)
      buzzer_beep(E5,50);
    else
      buzzer_beep(G5,50);
    buzzer_wait_play();

    // IR
    ir_init();
    delay_us(800000);
    // Voltage measure
    voltage_init();

    if (com_is_all_ok() ) { // && drivers_is_all_ok()) {
      // buzzer_play(MELODY_BEETHOVEN);
      if(developer_mode)
        buzzer_play(MELODY_BOOT_DEV);
      else
        buzzer_play(MELODY_BOOT);
      buzzer_wait_play();

    } else {
      buzzer_play(MELODY_WARNING);
    }

    terminal_init(&SerialUSB);

    // Infos
    infos_init();

    // Reiniting com
    com_init(developer_mode);

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

TERMINAL_COMMAND(hall, "Configuration hall")
{

  // int hall1=mux_sample(HALL1_ADDR);
  // delay_us(10000);
  // int hall2=mux_sample(HALL2_ADDR);
  // delay_us(10000);
  // int hall3=mux_sample(HALL3_ADDR);
  // delay_us(10000);
  // int hall4=mux_sample(HALL4_ADDR);

  // terminal_io()->println(hall1);
  terminal_io()->println(get_hall(HALL1_ADDR));

  // terminal_io()->println(hall2);
  terminal_io()->println(get_hall(HALL2_ADDR));

  // terminal_io()->println(hall3);
  terminal_io()->println(get_hall(HALL3_ADDR));

  // terminal_io()->println(hall4);
  terminal_io()->println(get_hall(HALL4_ADDR));

  terminal_io()->println("");


}

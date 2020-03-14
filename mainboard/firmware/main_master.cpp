#include <stdlib.h>
#include <wirish/wirish.h>
#include <libmaple/iwdg.h>
#include <terminal.h>
#include <main.h>
#include <series/gpio.h>
#include <watchdog.h>
#include "drivers.h"
#include "com.h"
#include "com_master.h"
#include "buzzer.h"
//#include "kicker.h"
//#include "voltage.h"
//#include "ir.h"
#include "infos.h"
//#include "kinematic.h"
#include "mux.h"
//#include "odometry.h"

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
     //infos_set(-1, false);

/*                                                                           */
/*****************************************************************************/
    // Infos
    infos_init();

    // Multiplexer
    mux_init();
    // Buzzer
    buzzer_init();
    delay_us(1000);
//    buzzer_beep(C6,100);
//   buzzer_wait_play();

    // perform a com diag:
    for(int i=0;i<10;++i){
        buzzer_beep(C4,100);
        buzzer_wait_play();
        delay_us(100);
    }

    com_init();

    if (com_is_all_ok()==false){
        buzzer_beep(C4,100);
        buzzer_wait_play();
    }


    com_master_init();
    delay_us(800000);

    buzzer_beep(C6,100);
    buzzer_wait_play();

    terminal_init(&SerialUSB);

    delay_us(800000);

    buzzer_beep(C6,100);
    buzzer_wait_play();

    // Starting the watchdog
    watchdog_start(WATCHDOG_58MS);

  //   systick_init();
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
    com_master_tick();

    // Buzzer
    buzzer_tick();
    //terminal_tick();
}

TERMINAL_COMMAND(diag, "Diagnostic")
{
  //  com_diagnostic();
    if (developer_mode)
      terminal_io()->println("in dev mode");
    else
      terminal_io()->println("in prod mode");
}



TERMINAL_COMMAND(devprod, "Switch dev/prod channel")
{
  developer_mode = !developer_mode;

  com_init();
  
  if (developer_mode)
    terminal_io()->println("in dev mode");
  else
    terminal_io()->println("in prod mode");
}

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
#include "odometry.h"
#include "com_robot.h"

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

/*                                                                           */
/*****************************************************************************/
    // Infos
    infos_init();

    // Multiplexer
    mux_init();
    // Buzzer
    buzzer_init();
    delay_us(1000);

    bool h1=get_hall(HALL1_ADDR);
    // delay_us(1000);
    bool h2=get_hall(HALL2_ADDR);
    // delay_us(1000);
    bool h3=get_hall(HALL3_ADDR);
    // delay_us(1000);
    bool h4=get_hall(HALL4_ADDR);


    int robot_id=id_from_hall(h1,h2,h3,h4);

    if(robot_id<=6) //IF IT IS A VALID ID
    {
      if(robot_id!=infos_get_id())
        infos_set(robot_id, false);
    }


    if(!h1 && !h2 && !h3 && !h4){ //no magnet
      developer_mode=true; //THIS IS A GLOBAL
    }


    if(!developer_mode)
    {

      ///RICKroll
      buzzer_play(RICKROLL);
      buzzer_wait_play();
      delay_us(2000000);

    }

/*****************************************************************************/
    /* HERE TO SWITCH BETWEEN DEVMODE/PRODMODE (until the magnets are found) */

    // developer_mode=false;
    // infos_set(4, false);


    int note=C7;

    if(developer_mode)
    {
      note=C5;
    }

    //THE ID
    int id=infos_get_id();
    if(id==0)
    {
      buzzer_beep(note,300);
      buzzer_wait_play();

    }
    else{
      for(int i=1;i<=id;i++)
      {
        buzzer_beep(note,65);
        buzzer_wait_play();
        delay_us(65000);
      }
    }


    delay_us(200000);

// Initalizng com
    com_robot_init();
    delay_us(800000);
    if(developer_mode)
      buzzer_beep(C6,50);
    else
      buzzer_beep(C5,50);
    buzzer_wait_play();
    // Initalizing drivers
    voltage_init();
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


    drivers_init();

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


    // Starting the watchdog
    watchdog_start(WATCHDOG_58MS);

    // systick_init();
}

#define TIME_STATS

#ifdef TIME_STATS
// Benchmaking main loop
int avg = 0;
int n = 0;
TERMINAL_COMMAND(bl, "")
{
    terminal_io()->println(avg/(n-1));
    avg = 0;
    n = 0;
}
#endif

/**
 * Loop function
 */

void loop()
{

#ifdef TIME_STATS
    // Benchmarking main loop
    static int last = micros();
#endif


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
#ifdef TIME_STATS
    if (n<100)
        last=micros();
#endif
    // Com
    com_robot_tick();
#ifdef TIME_STATS
    if (n<100){
        int loop = micros() - last;
        if (n != 0) {
            avg += loop;
        }
        n += 1;
    }
#endif


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

TERMINAL_COMMAND(hallid, "ID from hall")
{
  bool h1=get_hall(HALL1_ADDR);
  bool h2=get_hall(HALL2_ADDR);
  bool h3=get_hall(HALL3_ADDR);
  bool h4=get_hall(HALL4_ADDR);

  // terminal_io()->println(h1);
  // terminal_io()->println(h2);
  // terminal_io()->println(h3);
  // terminal_io()->println(h4);


  // bool developer_mode=false;

  // if(!h1 && !h2 && !h3 && !h4) //no magnet
  //   developer_mode=true;
  terminal_io()->println("ID:");
  terminal_io()->println(id_from_hall(h1,h2,h3,h4));
  terminal_io()->println("Dev mode:");
  terminal_io()->println(developer_mode?1:0);
  // int index=(h1) + (h2<<1) + (h3<<2) + (h4<<3);
  // terminal_io()->println(index);
}

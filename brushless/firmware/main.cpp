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
#include "servo.h"
#include "hardware.h"
#include "info.h"
#include <flash_write.h>
#include "motor_foc.h"

#define TEST_LED

//ONLY ONCE WHEN CONFIGURING THE MOTOR
#define SHOULD_DO_THE_TARE
//////


static bool is_tared=false;
static struct motor_info info;
/**
 * Setup function
 */
void setup()
{
  // init();
  // info_init();


#ifndef SHOULD_DO_THE_TARE
  //Read the f*cking flash
  flash_read(INFO_FLASH_ADDR, (void *)&info, sizeof(motor_info));
  set_origin(info.tare_value);
#else
  //WE WRITE THE TARE INTO THE FLASH
  //1) Activate SHOULD_DO_THE_TARE
  //2) Start the motor and type the tare cmd in terminak
  //3) put the returned value below
  //4) compile/upload/restart
  //5) Comment the SHOULD_DO_THE_TARE
  //6) compile/upload/restart
  info.tare_value=42; //PUT HERE THE VALUE GIVEN BY THE TARE CMD
  info_set(info);
#endif



  // Initalizing communication
  com_init();

  // Initializng current  sensor
#ifndef CURRENT_DISABLE
  current_init();
#endif

  // Initalizing encoder
  encoder_init();

  // Initalizing motor
  motor_init();

  // Initializing servo
  servo_init();

  // Starting the watchdog
  watchdog_start(WATCHDOG_14MS);

  terminal_init(&SerialUSB);

#ifdef TEST_LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
#endif

#ifdef SHOULD_DO_THE_TARE
  if(!is_tared){
    is_tared=true;
    launch_tare_motor();
  }
#endif
}


static int last_led = 0;

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
    #ifndef CURRENT_DISABLE
    current_tick();
    #endif

    // Ticking the terminal
    terminal_tick();

    // Ticking encoder
    encoder_tick();

    // Ticking servo
    servo_tick();

    // Ticking com
    com_tick();
#ifdef TEST_LED
    int val = millis();
    static int cnt_led = 0;
    if( val - last_led > 100 and cnt_led < 4 ){
        cnt_led ++;
        //terminal_io()->println( "ok" );
        if( cnt_led % 2 ){
          digitalWrite(LED_PIN, HIGH);
        }else{
          digitalWrite(LED_PIN, LOW);
        }
        last_led = val;
    }
#endif


}

TERMINAL_COMMAND(term, "term test")
{
    terminal_io()->println("term works !");
}



TERMINAL_COMMAND(read_info, "Read motor info in flash")
{
  flash_read(INFO_FLASH_ADDR, (void *)&info, sizeof(motor_info));
  terminal_io()->print("Flash value: ");
  terminal_io()->println(info.tare_value);
}

TERMINAL_COMMAND(save_info, "Save info on flash")
{
  info_set(info);
  terminal_io()->println("Done");
}


TERMINAL_COMMAND(show_tare_flash, "Show read tare info")
{
  terminal_io()->print("Tare: ");
  terminal_io()->println(info.tare_value);

}

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
/**
 * Setup function
 */
void setup()
{
    // Initalizing communication
//    com_init();

    // Initializng current  sensor
//    current_init();

    // Initalizing encoder
//    encoder_init();

    // Initalizing motor
//    motor_init();

    // Initializing servo
//    servo_init();

    // Starting the watchdog
    //watchdog_start(WATCHDOG_14MS);

    terminal_init(&SerialUSB);
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
}


static int last_led = 0;
static int cnt_led = 0;

/**
 * Loop function
 */
void loop()
{
    // Feeding watchdog
    //watchdog_feed();

    // Updating motor phases, this is also done in the hall pin interrupt but
    // it seems safe to do it often anyway
//    motor_tick();

    // Updating current sensor value
//    current_tick();

    // Ticking the terminal
    terminal_tick();

    // Ticking encoder
//    encoder_tick();

    // Ticking servo
//    servo_tick();

    // Ticking com
//    com_tick();

    int val = millis();
    if( val - last_led > 500 ){
        cnt_led ++;
        //terminal_io()->println( "ok" );
        if( cnt_led % 2 ){
          digitalWrite(LED_PIN, HIGH);
        }else{
          digitalWrite(LED_PIN, LOW);
        }
        last_led = val;
    }
}

TERMINAL_COMMAND(term, "term test")
{
    terminal_io()->println("term works !");
}

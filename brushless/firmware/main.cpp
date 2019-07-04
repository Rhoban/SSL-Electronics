#include <stdlib.h>
#include <wirish/wirish.h>
#include <libmaple/iwdg.h>
#include <terminal.h>
#include <stdint.h>
#include <series/gpio.h>
#include <watchdog.h>
#include "encoder.h"
#include "current.h"
#include "motor_hall.h"
#include "motor_foc.h"
#include "com.h"
#include "servo_hall.h"
#include "servo_foc.h"
#include "hardware.h"
#include "security.h"
#include "errors.h"
#include "hybrid.h"

#define TEST_LED

bool motor_is_on(){
    return motor_foc_is_on();
    //return motor_hall_is_on();
}

void init_pwm_pins(){
  // Initalizing motor pins
  pinMode(U_SD_PIN, OUTPUT);
  digitalWrite(U_SD_PIN, LOW);
  pinMode(V_SD_PIN, OUTPUT);
  digitalWrite(V_SD_PIN, LOW);
  pinMode(W_SD_PIN, OUTPUT);
  digitalWrite(W_SD_PIN, LOW);

  pinMode(U_IN_PIN, PWM);
  pwmWrite(U_IN_PIN, 0);
  pinMode(V_IN_PIN, PWM);
  pwmWrite(V_IN_PIN, 0);
  pinMode(W_IN_PIN, PWM);
  pwmWrite(W_IN_PIN, 0);
}

static void _init_timer(int number)
{
    HardwareTimer timer(number);

    // Configuring timer
    timer.pause();
    timer.setPrescaleFactor(PWM_PRESCALE_FACTOR);
    timer.setOverflow(PWM_OVERFLOW);
    timer.refresh();

    #if BOARD == GREG
    if (number == 3) {
        timer.setMode(TIMER_CH2, TIMER_OUTPUT_COMPARE);
        timer.setCompare(TIMER_CH2, 1);
        timer.attachInterrupt(TIMER_CH2, motor_irq);
    }
    #endif
    #if BOARD == CATIE
    if (number == 1) {
        timer.setMode(TIMER_CH4, TIMER_OUTPUT_COMPARE);
        timer.setCompare(TIMER_CH4, 1);
        timer.attachInterrupt(TIMER_CH4, motor_irq);
    }
    #endif
    timer.refresh();
    timer.resume();
}

void init_timers(){
    // Configuring timers
    #if BOARD == GREG
    _init_timer(2);
    _init_timer(3);
    #endif
    #if BOARD == CATIE
    _init_timer(1);
    #endif
}

void init_hall_hybrid(){
    // Initializing hall sensors input
    pinMode(HALLU_PIN, INPUT_PULLUP);
    pinMode(HALLV_PIN, INPUT_PULLUP);
    pinMode(HALLW_PIN, INPUT_PULLUP);

    // Attach interrupts on phase change
    attachInterrupt(HALLU_PIN, motor_hall_irq, CHANGE);
    attachInterrupt(HALLV_PIN, motor_hall_irq, CHANGE);
    attachInterrupt(HALLW_PIN, motor_hall_irq, CHANGE);
}

/**
 * Setup function
 */
void setup()
{
  // init();
  // info_init();

  // Initalizing communication
  com_init();

  // Initializng current  sensor
#ifndef CURRENT_DISABLE
  current_init();
#endif

  // Initalizing encoder
  encoder_init();

  // Initalizing motor
  init_hall_hybrid();
  init_timers();
  init_pwm_pins();

  // Initializing servo
  servo_foc_init();
  servo_hall_init();

  // Starting the watchdog
  watchdog_start(WATCHDOG_14MS);

  terminal_init(&SerialUSB);

#ifdef TEST_LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
#endif

  switch_to_foc();
  launch_tare_motor_foc();
}

static int last_warning = 0;
static int last_led = 0;

void display_warning(){
    int warning = security_get_warning();  
    int error = security_get_error();  
    if( warning != SECURITY_NO_WARNING || error != SECURITY_NO_ERROR ){
        int val = millis();
        if( val - last_warning > 4000 ){
            if( warning != SECURITY_NO_WARNING ){
                terminal_io()->println( driver_warning(warning) );
                security_set_warning( SECURITY_NO_WARNING );
                // encoder_print_errors();
            }
            if( error != SECURITY_NO_ERROR ){
                //terminal_io()->print("E ");
                //terminal_io()->print(error);
                terminal_io()->println( driver_error(error) );
            }
            last_warning = val;
        }
    }
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
    hybrid_tick();

    // Updating current sensor value
    #ifndef CURRENT_DISABLE
    current_tick();
    #endif

    // Ticking the terminal
    terminal_tick();

    // Ticking encoder
    encoder_tick();

    // Ticking servo
    if (security_get_error() != SECURITY_NO_ERROR) {
        motor_set(false, 0);
    } else {
      // update_hybrid_state();
      // change_motor_mode();
      servo_foc_tick();
      servo_hall_tick();
    }

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


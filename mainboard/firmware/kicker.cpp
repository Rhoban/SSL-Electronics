#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include <watchdog.h>
#include "hardware.h"
#include "kicker.h"
#include "mux.h"

static bool kicking = false;
static int kick_end = 0;
static bool clearing = false;
static int clear = 0;
static bool charging = false;

#define GREG 1
#define CATIE 2

#ifdef CATIE_BOARD
  #define BOARD CATIE
#endif
#ifdef GREG_BOARD
  #define BOARD GREG
#endif
#ifndef BOARD
  static_assert(false);
#endif

#if BOARD == GREG
  #define KICKER_ON HIGH
  #define KICKER_OFF LOW
  #define BOOST_ON 500
  #define BOOST_OFF 0
#else
  #define KICKER_ON LOW
  #define KICKER_OFF HIGH
  #define BOOST_ON LOW
  #define BOOST_OFF HIGH
#endif

void enable_boost(){
  #if BOARD == GREG
    pwmWrite(BOOSTER_PIN, BOOST_ON);
  #endif
  #if BOARD == CATIE
    digitalWrite(BOOSTER_PIN, BOOST_ON);
  #endif
}

void disable_boost(){
  #if BOARD == GREG
    pwmWrite(BOOSTER_PIN, BOOST_OFF);
  #endif
  #if BOARD == CATIE
    digitalWrite(BOOSTER_PIN, BOOST_OFF);
  #endif
}

void init_boost(){
    disable_boost();
    #if BOARD == GREG
      pinMode(BOOSTER_PIN, PWM);
    #endif
    #if BOARD == CATIE
      pinMode(BOOSTER_PIN, OUTPUT);
    #endif
    disable_boost();
}



TERMINAL_PARAMETER_FLOAT(cap, "Capacitor charge", 0.0);

static void _kicker_irq()
{
    digitalWrite(KICKER1_PIN, KICKER_ON);
    digitalWrite(KICKER2_PIN, KICKER_ON);

    HardwareTimer timer(KICKER_TIMER);
    timer.pause();

    if (charging) {
        enable_boost();
    }
}

void kicker_init()
{
    // Configuring booster timer
    HardwareTimer timer(BOOSTER_TIMER);

    timer.pause();
    timer.setPrescaleFactor(1);
    timer.setOverflow(3000); // 24 Khz
    timer.refresh();
    timer.resume();

    // Kicker timer
    HardwareTimer kickerTimer(KICKER_TIMER);
    kickerTimer.setChannel4Mode(TIMER_OUTPUT_COMPARE);
    kickerTimer.attachCompare4Interrupt(_kicker_irq);

    // Configuring booster pin
    init_boost();

    // Kicker pin
    pinMode(KICKER1_PIN, OUTPUT);
    digitalWrite(KICKER1_PIN, KICKER_ON);
    pinMode(KICKER2_PIN, OUTPUT);
    digitalWrite(KICKER2_PIN, KICKER_ON);
}

void kicker_clear()
{
    clearing = true;
    clear = 0;
}

void kicker_boost_enable(bool enable)
{
    charging = enable;

    if (enable) {
        clearing = false;
        enable_boost();
    } else {
        disable_boost();
    }
}

void kicker_kick(int kicker, int power)
{
    if (power < 0) {
        return;
    }
    if (power > 65000) {
        power = 65000;
    }

    if (kicker == 0) {
        digitalWrite(KICKER1_PIN, KICKER_OFF);
    } else {
        digitalWrite(KICKER2_PIN, KICKER_OFF);
    }

    HardwareTimer timer(KICKER_TIMER);
    timer.pause();
    timer.setPrescaleFactor(72); // 1uS per tick
    timer.setOverflow(0xffff);
    timer.setCompare(TIMER_CH4, power);
    timer.refresh();
    timer.resume();

    if (charging) {
        disable_boost();
    }
}

void kicker_tick()
{
    static int lastClear = millis();
    static int lastSample = millis();

    // Sampling capacitor voltage
    if (millis() - lastSample > 5) {
        float voltage = 3.3*mux_sample(CAP_ADDR)/4096;
        voltage = voltage*(CAP_R1+CAP_R2)/CAP_R2;

        lastSample = millis();
        cap = voltage*0.99 + cap*0.01;

        if (cap > 24 && !charging) {
            kicker_clear();
        }
    }

    // Triggering kicks for clearing
    if (clearing) {
        if ((millis() - lastClear) >= 10) {
            lastClear = millis();
            kicker_kick(1, 150);
            clear++;

            if (clear > 500) {
                clearing = false;
            }
        }
    }
}

float kicker_cap_voltage()
{
    return cap;
}

TERMINAL_COMMAND(cd, "Cap debug")
{
    terminal_io()->println(mux_sample(CAP_ADDR));
}

TERMINAL_COMMAND(boost, "Enable/disable the booster")
{
    if (argc) {
        kicker_boost_enable(atoi(argv[0])!=0);
    } else {
        terminal_io()->println("Usage: boost [0|1]");
    }
}

TERMINAL_COMMAND(kick, "Kicks")
{
    if (argc == 2) {
        kicker_kick(atoi(argv[0]), atoi(argv[1]));
    } else {
        terminal_io()->println("Usage: kick [kicker] [power]");
    }
}

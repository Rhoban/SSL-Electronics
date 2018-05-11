#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include <watchdog.h>
#include "hardware.h"
#include "kicker.h"
#include "mux.h"

static bool kicking = false;
static int kick_end = 0;

TERMINAL_PARAMETER_FLOAT(cap, "Capacitor charge", 0.0);

static void _kicker_irq()
{
    digitalWrite(KICKER1_PIN, HIGH);
    digitalWrite(KICKER2_PIN, HIGH);

    HardwareTimer timer(KICKER_TIMER);
    timer.pause();
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
    pwmWrite(BOOSTER_PIN, 0);
    pinMode(BOOSTER_PIN, PWM);
    pwmWrite(BOOSTER_PIN, 0);

    // Kicker pin
    digitalWrite(KICKER1_PIN, HIGH);
    digitalWrite(KICKER2_PIN, HIGH);
    pinMode(KICKER1_PIN, OUTPUT);
    digitalWrite(KICKER1_PIN, HIGH);
    pinMode(KICKER2_PIN, OUTPUT);
    digitalWrite(KICKER2_PIN, HIGH);
}

void kicker_boost_enable(bool enable)
{
    if (enable) {
        pwmWrite(BOOSTER_PIN, 500);
    } else {
        pwmWrite(BOOSTER_PIN, 0);
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
        digitalWrite(KICKER1_PIN, LOW);
    } else {
        digitalWrite(KICKER2_PIN, LOW);
    }

    HardwareTimer timer(KICKER_TIMER);
    timer.pause();
    timer.setPrescaleFactor(72); // 1uS per tick
    timer.setOverflow(0xffff);
    timer.setCompare(TIMER_CH4, power);
    timer.refresh();
    timer.resume();
}

void kicker_tick()
{
    static int lastSample = millis();

    float voltage = 3.3*mux_sample(CAP_ADDR)/4096;
    voltage = voltage*(CAP_R1+CAP_R2)/CAP_R2;

    if (millis() - lastSample > 5) {
        lastSample = millis();
        cap = voltage*0.99 + cap*0.01;
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

TERMINAL_COMMAND(cc, "Clear cap")
{
    kicker_boost_enable(0);

    for (int k=0; k<300; k++) {
        watchdog_feed();
        kicker_kick(1, 250);
        delay(5);
    }

    for (int k=0; k<5; k++) {
        kicker_kick(1, 5000);
        delay(5);
        watchdog_feed();
        delay(5);
        watchdog_feed();
    }
}

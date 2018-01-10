#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include "hardware.h"
#include "kicker.h"

static bool kicking = false;
static int kick_end = 0;

TERMINAL_PARAMETER_FLOAT(cap, "Capacitor charge", 0.0);

static void _kicker_irq()
{
    digitalWrite(KICKER1_PIN, HIGH);

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
    pinMode(KICKER1_PIN, OUTPUT);
    digitalWrite(KICKER1_PIN, HIGH);

    // Cap voltage
    pinMode(CAP_PIN, INPUT);
}

void kicker_boost_enable(bool enable)
{
    if (enable) {
        pwmWrite(BOOSTER_PIN, 500);
    } else {
        pwmWrite(BOOSTER_PIN, 0);
    }
}

void kicker_kick(int power)
{
    if (power > 65000) {
        power = 65000;
    }

    digitalWrite(KICKER1_PIN, LOW);

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

    float voltage = 3.3*analogRead(CAP_PIN)/4096;
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
    if (argc) {
        kicker_kick(atoi(argv[0]));
    } else {
        terminal_io()->println("Usage: kick [power]");
    }
}

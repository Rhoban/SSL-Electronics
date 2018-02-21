#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include "hardware.h"
#include "motor.h"
#include "security.h"

// Motor pins
static int motor_pins[6] = {
    U_LOW_PIN, U_HIGH_PIN,
    V_LOW_PIN, V_HIGH_PIN,
    W_LOW_PIN, W_HIGH_PIN
};

// Target PWM speed [0-3000]
static int motor_pwm = 0;

// Hall current phase
static int hall_current_phase = 0;
static int hall_last_change = 0;

// Consecutive phases
static int motor_phases[6][3] = {
//    U   V   W
    { 0,  1, -1},
    { 1,  0, -1},
    { 1, -1,  0},
    { 0, -1,  1},
    {-1,  0,  1},
    {-1,  1,  0},
};

// Estimating current phase (see above) depending on hall
// sensor value
static int hall_phases[8] = {
    -1,                     // 0b000 (impossible)
    0,                      // 0b001
    2,                      // 0b010
    1,                      // 0b011
    4,                      // 0b100
    5,                      // 0b101
    3,                      // 0b110
    -1,                     // 0b111 (impossible)
};

static void _init_timer(int number)
{
    HardwareTimer timer(number);

    // Configuring timer
    timer.pause();
    timer.setPrescaleFactor(1);
    timer.setOverflow(3000); // 24Khz
    timer.refresh();
    timer.resume();
}

static int hall_value()
{
    uint8_t hall = 0;

    hall  = ((digitalRead(HALLU_PIN)&1)<<2);
    hall |= ((digitalRead(HALLV_PIN)&1)<<1);
    hall |= ((digitalRead(HALLW_PIN)&1)<<0);

    return hall;
}

TERMINAL_PARAMETER_INT(dts, "", 0);

static void set_phases(int u, int v, int w, int phase)
{
    if (u != 0 && v != 0 && w != 0) {
        u = v = w = 0;
    }

    static int last_phase = -2;
    bool update = false;

    // Checking if we should update the mos phase, and generate
    // a deadtime
    update = (last_phase != phase);
    last_phase = phase;
    if (u == 0 && v == 0 && w == 0) {
        last_phase = -1;
    }

    if (update) {
        dts++;
    }

    if (update) {
        // Setting every output to low
        for (int k=0; k<6; k++) {
            digitalWrite(motor_pins[k], LOW);
            pinMode(motor_pins[k], OUTPUT);
        }

        // Dead-time
        delay_us(1);
    }

    if (u >= 0) {
        if (update) pinMode(U_HIGH_PIN, PWM);
        pwmWrite(U_HIGH_PIN, u);
    } else {
        // if (update) pinMode(U_LOW_PIN, PWM);
        // pwmWrite(U_LOW_PIN, -u);

        if (update) digitalWrite(U_LOW_PIN, HIGH);
    }

    if (v >= 0) {
        if (update) pinMode(V_HIGH_PIN, PWM);
        pwmWrite(V_HIGH_PIN, v);
    } else {
        // if (update) pinMode(V_LOW_PIN, PWM);
        // pwmWrite(V_LOW_PIN, -v);

        if (update) digitalWrite(V_LOW_PIN, HIGH);
    }

    if (w >= 0) {
        if (update) pinMode(W_HIGH_PIN, PWM);
        pwmWrite(W_HIGH_PIN, w);
    } else {
        // if (update) pinMode(W_LOW_PIN, PWM);
        // pwmWrite(W_LOW_PIN, -w);

        if (update) digitalWrite(W_LOW_PIN, HIGH);
    }
}

void motor_init()
{
    // Initializing hall sensors input
    pinMode(HALLU_PIN, INPUT_PULLUP);
    pinMode(HALLV_PIN, INPUT_PULLUP);
    pinMode(HALLW_PIN, INPUT_PULLUP);

    // Attach interrupts on phase change
    attachInterrupt(HALLU_PIN, motor_tick, CHANGE);
    attachInterrupt(HALLV_PIN, motor_tick, CHANGE);
    attachInterrupt(HALLW_PIN, motor_tick, CHANGE);

    // Configuring timers
    _init_timer(2);
    _init_timer(3);

    // Initalizing motor pins
    for (int k=0; k<6; k++)  pwmWrite(motor_pins[k], 0);
    for (int k=0; k<6; k++)  digitalWrite(motor_pins[k], LOW);
    for (int k=0; k<6; k++)  pinMode(motor_pins[k], OUTPUT);
}

TERMINAL_COMMAND(hall, "Test the hall sensors")
{
    terminal_io()->print(digitalRead(HALLU_PIN));
    terminal_io()->print(" ");
    terminal_io()->print(digitalRead(HALLV_PIN));
    terminal_io()->print(" ");
    terminal_io()->print(digitalRead(HALLW_PIN));
    terminal_io()->println();
}

void motor_set(int value)
{
    if (value > 0) value += PWM_MIN;
    if (value < 0) value -= PWM_MIN;

    if (value < -PWM_MAX) value = -PWM_MAX;
    if (value > PWM_MAX) value = PWM_MAX;

    motor_pwm = value;
}

void motor_tick()
{
    static bool motor_ticking = false;

    if (motor_ticking) {
        return;
    }
    motor_ticking = true;

    // Current phase
    int phase = hall_phases[hall_value()];
    // phase = 0; // XXX: To debug current & heat, to be removed

    if (phase >= 0 && phase < 6) {
        set_phases(
            motor_phases[phase][0]*motor_pwm,
            motor_phases[phase][1]*motor_pwm,
            motor_phases[phase][2]*motor_pwm,
            phase
        );
    } else {
        // XXX: This is not a normal state, not sure what should be done
        // in this situation
        set_phases(0, 0, 0, -1);
    }

    if (phase != hall_current_phase || motor_pwm == 0) {
        hall_last_change = millis();
    }
    hall_current_phase = phase;

    if ((millis() - hall_last_change) > 500 && abs(motor_pwm) >= 300) {
        // Stop everything
        // Will trigger watchdog and reset
        security_set_error(SECURITY_HALL_FREEZE);
    }

    motor_ticking = false;
}

TERMINAL_COMMAND(pwm, "Motor set PWM")
{
    if (argc > 0) {
        motor_set(atoi(argv[0]));
    } else {
        terminal_io()->print("usage: pwm [0-3000] (current: ");
        terminal_io()->print(abs(motor_pwm));
        terminal_io()->print(")");
        terminal_io()->println();
    }
}

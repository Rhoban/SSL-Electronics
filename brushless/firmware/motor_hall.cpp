#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include "encoder.h"
#include "hardware.h"
#include "motor.h"
#include "motor_hall.h"
#include "security.h"

#define PI_CST 3.14159265358979323846 

#define PWM_SUPREMUM PWM_HALL_SUPREMUM
#define PWM_MIN ((PWM_MIN_PERCENT*PWM_SUPREMUM)/100)
#define PWM_MAX ((PWM_MAX_PERCENT*PWM_SUPREMUM)/100)

static bool enable_hall = true;

// Motor pins
static int motor_pins[6] = {
    U_SD_PIN, U_IN_PIN,
    V_SD_PIN, V_IN_PIN,
    W_SD_PIN, W_IN_PIN
};

// Target PWM speed [0-3000]
static int motor_pwm = 0;
static bool motor_on = false;

// Hall current phase
static int hall_current_phase = -2;
static int hall_last_change = 0;
static int hall_last_change_moving = 0;
static int encoder_last_ok = 0;
static bool safe_mode = true;

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

static float angular_velocity;
        
void update_velocity(
  int new_phase, int previous_phase, int time, int hall_last_change
){
  int jump = new_phase - previous_phase;
  if( jump >= 3 ){
    jump -= 6;
  }
  if( jump <= -3 ){
    jump += 6;
  }
  // Jump is equal to -2, -1, 0, 1, 2, 3  
  float angle = jump * (2.0 * PI_CST/(NB_POSITIVE_MAGNETS));
  float velocity = 1000000.0*angle/(time-hall_last_change);
  angular_velocity = 0.9*angular_velocity + 0.1 * velocity;
}

static void _bc_load()
{
    // digitalWrite(W_SD_PIN, LOW);
    // digitalWrite(W_SD_PIN, HIGH);
}

static void _init_timer(int number)
{
    HardwareTimer timer(number);

    // Configuring timer
    timer.pause();
    timer.setPrescaleFactor(1);
    timer.setOverflow(3000); // 24Khz
    timer.refresh();

    if (number == 3) {
        // timer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
        // timer.setCompare(TIMER_CH1, 2550);
        // timer.attachCompare1Interrupt(_bc_load);
    }

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
    bool stopped = false;
    if (!motor_on || (u != 0 && v != 0 && w != 0)) {
        u = v = w = 0;
    }
    if (u == 0 && v == 0 && w == 0) {
        stopped = true;
    }

    static int last_phase = -8;
    static bool last_stopped = true;
    bool update = false;

    // Checking if we should update the mos phase, and generate
    // a deadtime
    update = (last_phase != phase) || (stopped != last_stopped);
    last_phase = phase;
    last_stopped = stopped;
    if (!motor_on) {
        last_phase = -2;
    }

    if (update) {
        dts++;
    }

    if (update) {
        // Setting every output to low
        digitalWrite(U_SD_PIN, LOW);
        digitalWrite(V_SD_PIN, LOW);
        digitalWrite(W_SD_PIN, LOW);

        pwmWrite(U_IN_PIN, 0);
        pwmWrite(V_IN_PIN, 0);
        pwmWrite(W_IN_PIN, 0);
    }

    if (motor_on) {
        if (u >= 0) {
            pwmWrite(U_IN_PIN, u);
        }

        if (stopped || u != 0) {
            if (update) digitalWrite(U_SD_PIN, HIGH);
        }

        if (v >= 0) {
            pwmWrite(V_IN_PIN, v);
        }

        if (stopped || v != 0) {
            if (update) digitalWrite(V_SD_PIN, HIGH);
        }

        if (w >= 0) {
            pwmWrite(W_IN_PIN, w);
        }

        if (stopped || w != 0) {
            if (update) digitalWrite(W_SD_PIN, HIGH);
        }
    }
}


void init_hall(){
    // Initializing hall sensors input
    pinMode(HALLU_PIN, INPUT_PULLUP);
    pinMode(HALLV_PIN, INPUT_PULLUP);
    pinMode(HALLW_PIN, INPUT_PULLUP);

    // Attach interrupts on phase change
    attachInterrupt(HALLU_PIN, motor_hall_irq, CHANGE);
    attachInterrupt(HALLV_PIN, motor_hall_irq, CHANGE);
    attachInterrupt(HALLW_PIN, motor_hall_irq, CHANGE);
}

void motor_hall_init()
{
    init_hall();

    // Configuring timers
    #if BOARD == GREG
    _init_timer(2);
    _init_timer(3);
    #endif
    #if BOARD == CATIE
    _init_timer(1);
    #endif

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

TERMINAL_COMMAND(hall, "Test the hall sensors")
{
    terminal_io()->print(digitalRead(HALLU_PIN));
    terminal_io()->print(" ");
    terminal_io()->print(digitalRead(HALLV_PIN));
    terminal_io()->print(" ");
    terminal_io()->print(digitalRead(HALLW_PIN));
    terminal_io()->println();
}

void motor_hall_set(bool enable, int value)
{
    motor_on = enable;

    if (value > 0) value += PWM_MIN;
    if (value < 0) value -= PWM_MIN;

    if (value < -PWM_MAX) value = -PWM_MAX;
    if (value > PWM_MAX) value = PWM_MAX;

    motor_pwm = value;
}

TERMINAL_COMMAND(bdw, "Bdw")
{
    int start = micros();
    for (int k=0; k<10000; k++) {
        digitalWrite(U_SD_PIN, LOW);
    }
    terminal_io()->println(micros()-start);
}

TERMINAL_COMMAND(ang, "angle")
{
    // terminal_io()->println("Jump :");
    // terminal_io()->println(jump);
    terminal_io()->println("ang velo :");
    terminal_io()->println(angular_velocity);
}

    
static bool motor_irq_is_active = false;
int current_phase;
int past_phase;

void motor_hall_irq(){

    if (motor_irq_is_active) {
        return;
    }
    motor_irq_is_active = true;

    current_phase = hall_value();
    if( ! enable_hall ) return;
    int phase = hall_phases[current_phase];
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
    
    motor_irq_is_active = false;
}

int previous_different_phase = 0;

void motor_hall_tick()
{
    // TODO : RETIRER motor_ticking : INUTILE
    static bool motor_ticking = false;

    if (motor_ticking) {
        return;
    }
    motor_ticking = true;
    motor_hall_irq();

    // Current phase
    int phase = hall_phases[current_phase];

    int time = millis();
    int time_ang = micros();
    if (phase != hall_current_phase) {
        update_velocity( phase, previous_different_phase, time_ang, hall_last_change);
        hall_last_change_moving = time_ang;
        hall_last_change = time_ang;
        previous_different_phase = phase;
    }else{
        #define VELOCITY_MIN 2 // turn/s
        #define DIVISION_HALL 6
        #define SCALE_TIME_UNIT 1000
        static const float  min_time_hall_irq =  SCALE_TIME_UNIT/(
          VELOCITY_MIN* DIVISION_HALL *NB_POSITIVE_MAGNETS
        );
        float dt = time - hall_last_change;
        if( dt > min_time_hall_irq ){
          angular_velocity = 0.0;
        }
    }
    if (abs(motor_pwm) < 300) {
        hall_last_change_moving = time;
    }
    hall_current_phase = phase;

    if ((time - hall_last_change) > 500 && hall_current_phase == -1) {
        security_set_error(SECURITY_HALL_MISSING);
    }

    if ((time - hall_last_change_moving) > 500 && abs(motor_pwm) >= 500) {
        // Stop everything
        security_set_error(SECURITY_HALL_FREEZE);
    }

    if (safe_mode) {
        if (encoder_is_present() && encoder_is_ok()) {
            encoder_last_ok = time;
        } else {
            if ((time - encoder_last_ok) > 500) {
                if (!encoder_is_present()) {
                    security_set_error(SECURITY_ENCODER_MISSING);
                } else if (!encoder_is_ok()) {
                    security_set_error(SECURITY_ENCODER_FAILURE);
                }
            }
        }
    }

    // if (!encoder_is_present()) {
    //     security_set_error(ENCODER
    // }

    motor_ticking = false;
}

TERMINAL_COMMAND(safe, "Safe mode")
{
    if (argc) {
        safe_mode = (atoi(argv[0]) != 0);
    } else {
        terminal_io()->println("Usage: safe [0|1]");
    }
}

TERMINAL_COMMAND(pwm, "Motor set PWM")
{
    if (argc > 0) {
        motor_set(true, atoi(argv[0]));
    } else {
        terminal_io()->print("usage: pwm [0-3000] (current: ");
        terminal_io()->print(abs(motor_pwm));
        terminal_io()->print(")");
        terminal_io()->println();
    }
}

TERMINAL_COMMAND(itest, "Interference test")
{
    for (int k=0; k<6; k++) {
        for (int j=0; k<6; k++) {
            if (j != k) {
                int pin1 = motor_pins[j];
                int pin2 = motor_pins[k];

                pinMode(pin1, INPUT_PULLUP);
                pinMode(pin2, OUTPUT);
                digitalWrite(pin2, LOW);

                if (digitalRead(pin1) == LOW) {
                    terminal_io()->print("Interference between ");
                    terminal_io()->print(motor_pins[j]);
                    terminal_io()->print(" and ");
                    terminal_io()->print(motor_pins[k]);
                    terminal_io()->println();
                }
            }
        }
    }
}

bool motor_hall_is_on(){
    return motor_on;
}

void switch_to_hall();
void switch_to_foc();

void enable_motor_hall(bool value){
    enable_hall = value;
}

float hall_to_float_speed(){
  return angular_velocity;
}

TERMINAL_COMMAND(enable_hall, "Enable hall"){
    if (argc == 1) {
        int val = atoi(argv[0]);
        if( val == 0 ){
            enable_motor_hall(false);
        }else{
            enable_motor_hall(true);
        }
    }
}

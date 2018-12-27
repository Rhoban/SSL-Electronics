#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include "encoder.h"
#include "hardware.h"
#include "motor.h"
#include "security.h"
#include "sin_lut.h"

// Motor pins
static int motor_pins[6] = {
    U_LOW_PIN, U_HIGH_PIN,
    V_LOW_PIN, V_HIGH_PIN,
    W_LOW_PIN, W_HIGH_PIN
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

static int rotor_last_valid_phase = 0;
static int rotor_angle = 0;
TERMINAL_PARAMETER_INT(rotor_pos, "Rotor angle", 0);

static int hall_angle(int prev_phase, int phase)
{
    if (phase < prev_phase) {
        int tmp = phase;
        phase = prev_phase;
        prev_phase = tmp;
    }

    if (prev_phase == 1 && phase == 2) {
        return 0;
    }
    if (prev_phase == 2 && phase == 3) {
        return 1365;
    }
    if (prev_phase == 3 && phase == 4) {
        return 2731;
    }
    if (prev_phase == 4 && phase == 5) {
        return 4096;
    }
    if (prev_phase == 0 && phase == 5) {
        return 5461;
    }
    if (prev_phase == 0 && phase == 1) {
        return 6827;
    }

    return 0;
}

static void _bc_load()
{
    // digitalWrite(W_LOW_PIN, LOW);
    // digitalWrite(W_LOW_PIN, HIGH);
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

static void set_sin_phases(int u, int v, int w)
{
    if (u < 0) u = 0;
    if (u > PWM_MAX) u = PWM_MAX;
    if (v < 0) v = 0;
    if (v > PWM_MAX) v = PWM_MAX;
    if (w < 0) v = 0;
    if (w > PWM_MAX) w = PWM_MAX;

    if (!motor_on) {
        digitalWrite(U_LOW_PIN, LOW);
        digitalWrite(V_LOW_PIN, LOW);
        digitalWrite(W_LOW_PIN, LOW);
    } else {
        digitalWrite(U_LOW_PIN, HIGH);
        digitalWrite(V_LOW_PIN, HIGH);
        digitalWrite(W_LOW_PIN, HIGH);

        pwmWrite(U_HIGH_PIN, u);
        pwmWrite(V_HIGH_PIN, v);
        pwmWrite(W_HIGH_PIN, w);
    }
}

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
        digitalWrite(U_LOW_PIN, LOW);
        digitalWrite(V_LOW_PIN, LOW);
        digitalWrite(W_LOW_PIN, LOW);

        pwmWrite(U_HIGH_PIN, 0);
        pwmWrite(V_HIGH_PIN, 0);
        pwmWrite(W_HIGH_PIN, 0);
    }

    if (motor_on) {
        if (u >= 0) {
            pwmWrite(U_HIGH_PIN, u);
        }

        if (stopped || u != 0) {
            if (update) digitalWrite(U_LOW_PIN, HIGH);
        }

        if (v >= 0) {
            pwmWrite(V_HIGH_PIN, v);
        }

        if (stopped || v != 0) {
            if (update) digitalWrite(V_LOW_PIN, HIGH);
        }

        if (w >= 0) {
            pwmWrite(W_HIGH_PIN, w);
        }

        if (stopped || w != 0) {
            if (update) digitalWrite(W_LOW_PIN, HIGH);
        }
    }
}

void motor_tick_irq();

void motor_init()
{
    // Initializing hall sensors input
    pinMode(HALLU_PIN, INPUT_PULLUP);
    pinMode(HALLV_PIN, INPUT_PULLUP);
    pinMode(HALLW_PIN, INPUT_PULLUP);

    // Attach interrupts on phase change
    attachInterrupt(HALLU_PIN, motor_tick_irq, CHANGE);
    attachInterrupt(HALLV_PIN, motor_tick_irq, CHANGE);
    attachInterrupt(HALLW_PIN, motor_tick_irq, CHANGE);

    // Configuring timers
    _init_timer(2);
    _init_timer(3);

    // Initalizing motor pins
    for (int k=0; k<6; k++)  pwmWrite(motor_pins[k], 0);
    digitalWrite(U_HIGH_PIN, LOW);
    digitalWrite(V_HIGH_PIN, LOW);
    digitalWrite(W_HIGH_PIN, LOW);
    digitalWrite(U_LOW_PIN, LOW);
    digitalWrite(V_LOW_PIN, LOW);
    digitalWrite(W_LOW_PIN, LOW);

    pinMode(U_LOW_PIN, OUTPUT);
    pinMode(V_LOW_PIN, OUTPUT);
    pinMode(W_LOW_PIN, OUTPUT);

    pinMode(U_HIGH_PIN, PWM);
    pinMode(V_HIGH_PIN, PWM);
    pinMode(W_HIGH_PIN, PWM);
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

void motor_set(bool enable, int value)
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
        digitalWrite(U_LOW_PIN, LOW);
    }
    terminal_io()->println(micros()-start);
}

TERMINAL_PARAMETER_INT(fp, "Force phase", -1);
TERMINAL_PARAMETER_INT(vel, "Velocity", 8);

TERMINAL_PARAMETER_BOOL(mdb, "", false);

void motor_tick_irq()
{
    // motor_tick(true);
}

TERMINAL_PARAMETER_INT(sphase, "", 0);
TERMINAL_PARAMETER_INT(sphase_delta, "", 1);

TERMINAL_PARAMETER_BOOL(rotor_enc, "", false);

static int last_tmp = 0;
static int phase = -1;


void compute_current_phase(){
    // Current phase
    phase = hall_phases[hall_value()];
    if (fp >= 0) {
        phase = fp;
    }
    if (phase != hall_current_phase) {
        hall_last_change_moving = millis();
        hall_last_change = millis();
    }
    if (abs(motor_pwm) < 300) {
        hall_last_change_moving = millis();
    }
    hall_current_phase = phase;
}

void compute_rotor_angle_from_hall(){
    if (phase >= 0 && phase < 6) {
        if (phase != rotor_last_valid_phase) {
            rotor_angle = hall_angle(phase, rotor_last_valid_phase);
            rotor_last_valid_phase = phase;
        }
    }
}

static bool old_angle_exists = false;
static float old_angle;

static float theta; // Angular position
static float old_theta;

void compute_rotor_position(){
    old_theta = theta;
    if (rotor_enc) {
        float angle = encoder_value()/16384.0;
        if( ! old_angle_exists ){
            theta = angle;
            old_angle_exists = true;
        }else{
            float delta = angle-old_angle;    
            if( fabs(delta) < .5 ) {
                delta = delta - 1.0;
            }
            theta += delta;
        }
        old_angle = angle;
    } else {
        theta = (rotor_angle&8191) / 8192.0;
    }
}

void display_some_message(bool irq){
    static int last_display_time = millis();
    if (((millis() - last_display_time) > 1) && mdb && !irq) {
        //sphase += sphase_delta;
        last_display_time = millis();
        terminal_io()->print(theta);
        terminal_io()->print(" ");
        terminal_io()->println();
    }
}

void make_safety_work(){
    if ((millis() - hall_last_change) > 500 && hall_current_phase == -1) {
        security_set_error(SECURITY_HALL_MISSING);
    }

    if (fp < 0 && (millis() - hall_last_change_moving) > 500 && abs(motor_pwm) >= 800) {
        // Stop everything
        security_set_error(SECURITY_HALL_FREEZE);
    }
    if (safe_mode) {
        if (encoder_is_present() && encoder_is_ok()) {
            encoder_last_ok = millis();
        } else {
            if ((millis() - encoder_last_ok) > 500) {
                if (!encoder_is_present()) {
                    security_set_error(SECURITY_ENCODER_MISSING);
                } else if (!encoder_is_ok()) {
                    security_set_error(SECURITY_ENCODER_FAILURE);
                }
            }
        }
    }
}

static float dt;
static float speed;

void compute_rotor_velocity(){
    speed = (theta - old_theta)/dt; // To Improve !
}

TERMINAL_PARAMETER_FLOAT(k_pos_p, "Posiition P", 10.0);
TERMINAL_PARAMETER_FLOAT(k_speed_ff, "Speed FF", 0.0);
TERMINAL_PARAMETER_FLOAT(k_speed_p, "Speed P", 10.0);
TERMINAL_PARAMETER_FLOAT(k_speed_i, "Speed I", 10.0);
TERMINAL_PARAMETER_FLOAT(k_current_p, "Current P", 1.0);
TERMINAL_PARAMETER_FLOAT(k_current_i, "Current I", 0.0);
TERMINAL_PARAMETER_FLOAT(theta_c, "Angular position consign", 10.0);


static float speed_ff; // Speed feedforward
static float speed_p; // Speed proportional
static float speed_c; // Speed consign

void compute_velocity_consign(){
    // w* = wff + pos_p * ( theta* - tetha )
    speed_ff = k_speed_ff * (theta_c/dt);
    speed_p = k_pos_p * ( theta_c - theta );
    speed_c = speed_ff + speed_p ;
}

static float direct_current_c; // direct current consign
static float quadrature_current_c; // quadrature current consign
static float speed_error;
static float integral_speed_error = 0.0;

void compute_direct_quadrature_current_consign(){
    speed_error = speed_c - speed;
    integral_speed_error += ( dt * speed_error );

    direct_current_c = 0; // direct current consign. Always equals to 0.
    quadrature_current_c = (
        k_speed_p * speed_error + k_speed_i * integral_speed_error
    ); // quadrature current consign
}

static float direct_voltage_c; // direct current consign
static float quadrature_voltage_c; // quadrature current consign
static float direct_current_error;
static float integral_direct_current_error = 0.0;
static float direct_current;
static float quadrature_current_error;
static float integral_quadrature_current_error = 0.0;
static float quadrature_current;

void compute_direct_and_quadrature_voltage_consign(){
    direct_current_error = direct_current_c - direct_current;
    integral_direct_current_error += ( dt * direct_current_error );
    
    direct_voltage_c = (
        k_current_p * direct_current_error
        +
        k_current_i * integral_quadrature_current_error
    );

    quadrature_current_error = quadrature_current_c - quadrature_current;
    integral_quadrature_current_error += ( dt * quadrature_current_error );
    
    quadrature_voltage_c = (
        k_current_p * quadrature_current_error
        +
        k_current_i * integral_quadrature_current_error
    );
}

static float phase_voltage_u;
static float phase_voltage_v;
static float phase_voltage_w;

void convert_direct_and_quadrature_voltage_to_phase_voltage(){
    // ( vu )                ( c1 -s1 )
    // ( vv ) = sqrt(2/3) .  ( c2 -s2 ) . ( vd )
    // ( vw )                ( c3 -s3 )   ( vq )
    //
    // c1 = cos( theta )   s1 = cos( theta ) 
    // c2 = cos( theta - 2pi/3 )   s2 = cos( theta - 2pi/3 ) 
    // c3 = cos( theta + 2pi/3 )   s3 = cos( theta + 2pi/3 ) 

    const float sqrt_2_3 = 0.816496580927726;

    const int t1 = theta;
    const int t2 = theta - 1.0/3.0;
    const int t3 = theta + 1.0/3.0;

    const float c1 = cos_t(t1);
    const float s1 = sin_t(t1);
    const float c2 = cos_t(t2);
    const float s2 = sin_t(t2);
    const float c3 = cos_t(t3);
    const float s3 = sin_t(t3);

	phase_voltage_u = sqrt_2_3 * (c1 * direct_voltage_c - s1 * quadrature_voltage_c);
	phase_voltage_v = sqrt_2_3 * (c2 * direct_voltage_c - s2 * quadrature_voltage_c);
	phase_voltage_w = sqrt_2_3 * (c3 * direct_voltage_c - s3 * quadrature_voltage_c);
}

static float time = 0;

void update_dt(){
    float new_time = millis();
    dt = new_time - time;
    time = new_time;
    if( dt == 0 ) dt = 0.00001;
}

        
void compute_phase_current(){
    // TODO : to make in electronics
    // Not present in electronic
}
        
void convert_phase_current_to_direct_and_quadrature_current(){
    // TODO : when phase current will be measured, we need to implement
    // the conversion from pahse current to direct and quadrature current
    direct_current = direct_current_c;
    quadrature_current = quadrature_current_c;
}

static int phase_pwm_u = 0;
static int phase_pwm_v = 0;
static int phase_pwm_w = 0;

        
void convert_direct_and_quadrature_voltage_to_phase_pwm(){
    phase_pwm_u = phase_voltage_u * abs(motor_pwm);
    phase_pwm_v = phase_voltage_v * abs(motor_pwm);
    phase_pwm_w = phase_voltage_w * abs(motor_pwm);
}

void motor_tick(bool irq)
{
    update_dt();
    if( dt <= 0.0 ) return;

    compute_current_phase();

    if (phase >= 0 && phase < 6) {
        // Inputs
        compute_rotor_angle_from_hall();
        compute_rotor_position();
        compute_rotor_velocity();
        // compute_phase_current(); TODO when electronic is able to get current
        convert_phase_current_to_direct_and_quadrature_current();

        // Outputs
        compute_velocity_consign();
        compute_direct_quadrature_current_consign();
        compute_direct_and_quadrature_voltage_consign();
        convert_direct_and_quadrature_voltage_to_phase_voltage();
        convert_direct_and_quadrature_voltage_to_phase_pwm();

        // Apply control
        set_sin_phases( phase_pwm_u, phase_pwm_v, phase_pwm_w );

        display_some_message(irq);
    } else {
        // XXX: This is not a normal state, not sure what should be done
        // in this situation
        set_phases(0, 0, 0, -1);
    }
    make_safety_work();
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

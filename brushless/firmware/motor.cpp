#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include "encoder.h"
#include "hardware.h"
#include "motor.h"
#include "ssl.h"
#include "sin_lut.h"
#include "security.h"
#include "errors.h"

#define REFERENCE_VOLTAGE 12
#define HALF_REFERENCE_VOLTAGE REFERENCE_VOLTAGE/2 

void display();

enum Motor_mode {
    PHASE_MODE,
    VECTORIAL_MODE
};

static Motor_mode mode = PHASE_MODE;

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

static void _bc_load()
{
    // digitalWrite(W_SD_PIN, LOW);
    // digitalWrite(W_SD_PIN, HIGH);
}

#define MOTOR_SUB_SAMPLE 10
static unsigned int count_irq = 0;
static bool motor_flag = false;


void motor_irq(){
    count_irq = (count_irq + 1)%(MOTOR_SUB_SAMPLE);
    if( ! count_irq ){
        if( motor_flag ){
            security_set_warning(WARNING_MOTOR_LAG);
        }
        motor_flag = true;
    }
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
        timer.setMode(TIMER_CH2, TIMER_OUTPUT_COMPARE);
        timer.setCompare(TIMER_CH2, 1);
        timer.attachInterrupt(TIMER_CH2, motor_irq);
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

void disable_all_motors(){
    pwmWrite(U_IN_PIN, 0);
    pwmWrite(V_IN_PIN, 0);
    pwmWrite(W_IN_PIN, 0);
    
    digitalWrite(U_SD_PIN, LOW);
    digitalWrite(V_SD_PIN, LOW);
    digitalWrite(W_SD_PIN, LOW);
} 

void apply_pwm( int low_pin, int high_pin, int pwm ){
    //
    // Commands for the ir2104 controller
    //
    // SD = 0 => HO = 0 and L0 = 0 
    // SD = 1 and IN = 0 => HO = 0 and LO = 1
    // SD = 1 and IN = 1 => HO = 1 and LO = 0
    //

    // Last security check ! Do not remove !
    if( pwm < PWM_MIN ){
        pwm = PWM_MIN; 
        security_set_error(SECURITY_PWM_MIN);
    }
    if( pwm > PWM_MAX ){
        pwm = PWM_MAX;
        security_set_error(SECURITY_PWM_MAX);
    }

    // We apply pwm
    digitalWrite(low_pin, HIGH);
    pwmWrite(high_pin, pwm);
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
    digitalWrite(U_IN_PIN, LOW);
    digitalWrite(V_IN_PIN, LOW);
    digitalWrite(W_IN_PIN, LOW);
    digitalWrite(U_SD_PIN, LOW);
    digitalWrite(V_SD_PIN, LOW);
    digitalWrite(W_SD_PIN, LOW);

    pinMode(U_SD_PIN, OUTPUT);
    pinMode(V_SD_PIN, OUTPUT);
    pinMode(W_SD_PIN, OUTPUT);

    pinMode(U_IN_PIN, PWM);
    pinMode(V_IN_PIN, PWM);
    pinMode(W_IN_PIN, PWM);
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
    mode = PHASE_MODE;
    motor_on = enable;

    if (value > 0) value += PWM_MIN;
    if (value < 0) value -= PWM_MIN;

    if (value < -PWM_MAX) value = -PWM_MAX;
    if (value > PWM_MAX) value = PWM_MAX;

    motor_pwm = value;
}

static float direct_voltage_c = 0.0;
static float quadrature_voltage_c = 0.0;

float compute_maximal_voltage(float vd, float vq){
    // We need to evaluate the maximum value
    // of the difference between two phases voltages when theta is going from 
    // 0 to 2 pi.
    // If that maximum it too important, then we scale the voltage in such a 
    // way tha difference is borned by MAX_PHASE_VOLTAGE
    // 
    // MAX diffrence PHASE VOLTAGE : 
    // 
    //  phase_voltage_u - phase_voltage_v = sqrt_2_3 * (
    //     (c1-c2) * direct_voltage_c
    //   - (s1-s2) * quadrature_voltage_c
    //  );
    //
    // Let J = max( a*(c1-c2) - b*(s1-s2) )
    //
    // max occurs
    // when deriv( a*(c1-c2) - b*(s1-s2) ) = -a*(s1-s2) - b*(c1-c2) = 0
    // so
    // a * J = (c1-c2) * ( a^2 + b^2 )     and     c1-c2 = a * J /(a^2+b^2)
    // b * J = (s1-s2) * ( -a^2 -b^2 )     and     s1-s2 = - b * J /(a^2+b^2)
    //
    // (c1-c2)^2 + (s1-s2)^2 = 2 - 2 (c1*c2+s1*s2) = 
    // 2 - 2*cos( theta - (theta +- 2pi/3) ) = 2 - 2 cos(2pi/3) = 3
    //
    // 3 = ( a^2 + b^2 ) * J^2 / (a^2 + b^2)^2
    // 
    // J = sqrt(3) * sqrt( a^2 + b^2 ) 
    //
    // So the maximal difference voltage is equal to 
    // sqrt(2/3) * sqrt(3) * sqrt(a^2 + b^2) = sqrt(2) * sqrt(a^2+b^2)

    const float sqrt_2 = 1.4142135623730951;
    return sqrt_2 * sqrt( vd * vd + vq * vq );
}

void direct_quadrature_voltage_set(float vd, float vq ){
    float max_voltage = compute_maximal_voltage(vd, vq);
    direct_voltage_c = vd;
    quadrature_voltage_c = vq;
    if( max_voltage > REFERENCE_VOLTAGE ){
        //terminal_io()->println( "M" ); //AX VOLTAGE !" );
        //terminal_io()->println();
        direct_voltage_c = ( REFERENCE_VOLTAGE / max_voltage ) * direct_voltage_c;
        quadrature_voltage_c = ( REFERENCE_VOLTAGE / max_voltage ) * quadrature_voltage_c;
    }
}

static int last_tare_time = 0;
static int tare_is_set = false;
static int tare_is_done = false;

TERMINAL_COMMAND(dqv, "Set direct and quadrature voltage")
{
    if (argc == 2) {
        if( ! tare_is_done ){
            terminal_io()->println("Please tare the motor first, by typing : ");
            terminal_io()->println("  tare");
            return;
        }

        float vd = atoi(argv[0]);
        float vq = atoi(argv[1]);
        float max_voltage = compute_maximal_voltage(vd, vq);
        if( max_voltage <= REFERENCE_VOLTAGE ){
            direct_quadrature_voltage_set(vd, vq);
        }else{
            terminal_io()->print(  "    Fail : sqrt(2)*sqrt(d^2 + q^2) !< " );
            terminal_io()->println(REFERENCE_VOLTAGE);
        }
    } else {
        terminal_io()->print("usage: dqv [0-");
        terminal_io()->print(REFERENCE_VOLTAGE);
        terminal_io()->print("] [0-");
        terminal_io()->print(REFERENCE_VOLTAGE);
        terminal_io()->println("]");
        terminal_io()->println("  first parameter d : direct voltage");
        terminal_io()->println("  second parameter q : quadrature voltage");
        terminal_io()->print(  "  sqrt(2)*sqrt(d^2 + q^2) < " );
        terminal_io()->print(REFERENCE_VOLTAGE);
        terminal_io()->println(".");
    }
}

TERMINAL_COMMAND(bdw, "Bdw")
{
    int start = micros();
    for (int k=0; k<10000; k++) {
        digitalWrite(U_SD_PIN, LOW);
    }
    terminal_io()->println(micros()-start);
}

TERMINAL_PARAMETER_INT(fp, "Force phase", -1);

void control_motor_with_phases(){
    // Current phase
    int phase = hall_phases[hall_value()];
    if (fp >= 0) {
        phase = fp;
    }

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

    if (phase != hall_current_phase) {
        hall_last_change_moving = millis();
        hall_last_change = millis();
    }
    if (abs(motor_pwm) < 300) {
        hall_last_change_moving = millis();
    }
    hall_current_phase = phase;
}

void phase_security_check(){
    if ((millis() - hall_last_change) > 500 && hall_current_phase == -1) {
        security_set_error(SECURITY_HALL_MISSING);
    }

    if (fp < 0 && (millis() - hall_last_change_moving) > 500 && abs(motor_pwm) >= 500) {
        // Stop everything
        security_set_error(SECURITY_HALL_FREEZE);
    }
}
void encoder_security_check(){
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

static float angle_origin = 0.0;
static int vectorial_pwm = CONFIG_PWM;
static int save_vectorial_pwm = CONFIG_PWM;

void control_motor_with_vectorial( float theta ){

    ///////////////////////////////////////////////////////////////////////////
    // Compute phase voltage
    ///////////////////////////////////////////////////////////////////////////
    // ( vu )                ( c1 -s1 )
    // ( vv ) = sqrt(2/3) .  ( c2 -s2 ) . ( vd )
    // ( vw )                ( c3 -s3 )   ( vq )
    //
    // c1 = cos( theta )   s1 = cos( theta ) 
    // c2 = cos( theta - 2pi/3 )   s2 = cos( theta - 2pi/3 ) 
    // c3 = cos( theta + 2pi/3 )   s3 = cos( theta + 2pi/3 ) 

    const float sqrt_2_3 = 0.816496580927726;

    // float r = theta - ( (int) ( theta / NB_POSITIVE_MAGNETS ) );
    float theta_park = NB_POSITIVE_MAGNETS*theta;

    theta_park -= ( (int) theta_park );
    if( theta_park < 0 ) theta_park += 1;
    
    //DEBUG 
    if( theta_park < 0.0 or theta_park >= 1.0 ){
        security_set_warning( WARNING_MOTOR_LAG );
    }

    const float t1 = theta_park;
    const float t2 = theta_park - 1.0/3.0;
    const float t3 = theta_park + 1.0/3.0;
    
    const float c1 = cos_t(t1);
    const float s1 = sin_t(t1);
    const float c2 = cos_t(t2);
    const float s2 = sin_t(t2);
    const float c3 = cos_t(t3);
    const float s3 = sin_t(t3);

//    terminal_io()->print("t1 : ");
//    terminal_io()->print( t1 );
//    terminal_io()->print(", t2 : ");
//    terminal_io()->print(t2);
//    terminal_io()->print(", t3 : ");
//    terminal_io()->print(t3);
//    terminal_io()->println();
//        
//    terminal_io()->print("c1 : ");
//    terminal_io()->print( c1 );
//    terminal_io()->print(", c2 : ");
//    terminal_io()->print(c2);
//    terminal_io()->print(", c3 : ");
//    terminal_io()->print(c3);
//    terminal_io()->println();
//    
//    terminal_io()->print("s1 : ");
//    terminal_io()->print( s1 );
//    terminal_io()->print(", s2 : ");
//    terminal_io()->print(s2);
//    terminal_io()->print(", s3 : ");
//    terminal_io()->print(s3);
//    terminal_io()->println();


	float phase_voltage_u = sqrt_2_3 * (c1 * direct_voltage_c - s1 * quadrature_voltage_c);
	float phase_voltage_v = sqrt_2_3 * (c2 * direct_voltage_c - s2 * quadrature_voltage_c);
	float phase_voltage_w = sqrt_2_3 * (c3 * direct_voltage_c - s3 * quadrature_voltage_c);

//    terminal_io()->print("phase_voltage_u : ");
//    terminal_io()->print( phase_voltage_u );
//    terminal_io()->print("phase_voltage_v : ");
//    terminal_io()->print( phase_voltage_v );
//    terminal_io()->print("phase_voltage_w : ");
//    terminal_io()->print( phase_voltage_w );
//    terminal_io()->println();

    ///////////////////////////////////////////////////////////////////////////
    // Convert phase voltage to phase PWM
    ///////////////////////////////////////////////////////////////////////////

    float min_voltage = phase_voltage_u;
    if( min_voltage > phase_voltage_v) min_voltage = phase_voltage_v;
    if( min_voltage > phase_voltage_w) min_voltage = phase_voltage_w;

    int pwm_max = ( PWM_SUPREMUM * vectorial_pwm ) / 100;
    if( pwm_max < 0 ) pwm_max = 0;

    // We center the sinusoide to have phase from 0V to REFERENCE_VOLTAGE
    int phase_pwm_u = ((phase_voltage_u - min_voltage)/REFERENCE_VOLTAGE) * pwm_max;
    int phase_pwm_v = ((phase_voltage_v - min_voltage)/REFERENCE_VOLTAGE) * pwm_max;
    int phase_pwm_w = ((phase_voltage_w - min_voltage)/REFERENCE_VOLTAGE) * pwm_max;

            
    ///////////////////////////////////////////////////////////////////////////
    // Set PWM 
    ///////////////////////////////////////////////////////////////////////////
    
    if (!motor_on) {
        disable_all_motors();
    } else {
        apply_pwm(U_SD_PIN, U_IN_PIN, phase_pwm_u);
        apply_pwm(V_SD_PIN, V_IN_PIN, phase_pwm_v);
        apply_pwm(W_SD_PIN, W_IN_PIN, phase_pwm_w);
    }
}

float rotor_angle(){
    #ifdef REVERSE_PHASE
        return - ( encoder_to_turn() - angle_origin ); // REVERSE SPEED
    #else
        return ( encoder_to_turn() - angle_origin );
    #endif
}

static float dt;
static float time = 0;


#define DT_MIN .1
void update_dt(){
    float new_time = millis();
    dt = new_time - time;
    if( dt <= DT_MIN ) return; //dt = 0.00001;
    time = new_time;
}

void motor_tick()
{
    static bool motor_ticking = false;

    if (motor_ticking) {
        return;
    }
    motor_ticking = true;

    // update_dt();
    // if( dt <= DT_MIN ){
    //    motor_ticking = false;
    //    return; //dt = 0.00001;
    //}
    
    if( !motor_flag ){
        motor_ticking = false;
        return;
    }
    motor_flag = false;
    
    display();

    if( mode == PHASE_MODE ){
        control_motor_with_phases();
        phase_security_check();
    }else{
        float theta;
        if( tare_is_set ){
            theta = 0.0;
            if( millis() - last_tare_time > 1000 ){
                angle_origin = encoder_to_turn();
                terminal_io()->println("Tare is done.");
                tare_is_done = true;
                tare_is_set = false;
                vectorial_pwm = save_vectorial_pwm;
                direct_voltage_c = 0.0;
                quadrature_voltage_c = 0.0;
            }
        }else{
            theta = rotor_angle();
        }
        if( tare_is_done || tare_is_set ){
            control_motor_with_vectorial(theta);
        }else{
            disable_all_motors();
        }
    }
    
    //encoder_security_check();
    
    motor_ticking = false;
}

void start_to_tare_motor(){
    if(!tare_is_set){
        motor_on = true,
        last_tare_time = millis();
        direct_voltage_c = HALF_REFERENCE_VOLTAGE/2.0; 
        quadrature_voltage_c = 0.0;
        save_vectorial_pwm = vectorial_pwm;
        vectorial_pwm = CONFIG_PWM;
        mode = VECTORIAL_MODE;
        tare_is_set = true;
    }
}

TERMINAL_COMMAND(info_motor, "Info on motor")
{
    if( mode == VECTORIAL_MODE){
        terminal_io()->print("quadrature voltage : ");
        terminal_io()->println(quadrature_voltage_c);
        terminal_io()->print("direct voltage : ");
        terminal_io()->println(direct_voltage_c);
        terminal_io()->print("vectorial pwm : ");
        terminal_io()->println(vectorial_pwm);
        terminal_io()->print("theta : ");
        terminal_io()->println(rotor_angle());
        terminal_io()->print("motor on : ");
        terminal_io()->println(motor_on);
    }
}


TERMINAL_COMMAND(tare, "Tare origin")
{
    tare_is_set = false;
    int max_pwm = (100*PWM_MAX)/PWM_SUPREMUM;
    int min_pwm = (100*PWM_MIN)/PWM_SUPREMUM;
    if( vectorial_pwm < min_pwm or vectorial_pwm > max_pwm ){
        terminal_io()->print("First enable motors by defining a pwm. For example :" );
        terminal_io()->println();
        terminal_io()->println();
        terminal_io()->print("    vec_pwm");
        terminal_io()->println();
    }else{
        start_to_tare_motor();
    }
}

TERMINAL_COMMAND(phase_mode, "control motor with phases")
{
    mode = PHASE_MODE;
    terminal_io()->println("Phase mode is set.");
}

TERMINAL_COMMAND(vectorial_mode, "control motor with phases")
{
    mode = VECTORIAL_MODE;
    terminal_io()->println("Vectorial mode is set.");
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

TERMINAL_COMMAND(vec_pwm, "set Vectorial PWM for motor")
{
    bool success = false;
    int max_pwm = (100*PWM_MAX)/PWM_SUPREMUM;
    int min_pwm = (100*PWM_MIN)/PWM_SUPREMUM;
    if (argc <= 1 ){
        int p = CONFIG_PWM;
        if( argc == 1 ){
            p = atoi(argv[0]);
        }
        if( min_pwm<=p and p<=max_pwm ){;
            vectorial_pwm = p;
            success = true;
        }else{
            vectorial_pwm = min_pwm;
        }
    } 
    if( success ){
        mode = VECTORIAL_MODE;
        terminal_io()->print("vectorial pwm set to : ");
        terminal_io()->println(vectorial_pwm);
    }else{
        terminal_io()->print("usage: vec_pwm [");
        terminal_io()->print(min_pwm);
        terminal_io()->println("-");
        terminal_io()->print(max_pwm);
        terminal_io()->println("]");
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

static unsigned int cnt = 0;
static int display_time = 0; 

void display_data(){
        terminal_io()->println( "" );
}

void display(){
    cnt ++;
    int val = millis();
    if( val  -  display_time > 4000 ){
        int dt = val - display_time;
        terminal_io()->print("D(");
        terminal_io()->print(cnt);
        terminal_io()->print(" - ");
        terminal_io()->print( (cnt*1000)/dt );
        terminal_io()->print("Hz - ");
        terminal_io()->print( dt/1000 );
        terminal_io()->print("s");
        int error = security_get_error();  
        if( error != SECURITY_NO_ERROR ){
            terminal_io()->print(" - ");
            terminal_io()->print( driver_error(error) );
        }
        int warning = security_get_warning();  
        if( warning != SECURITY_NO_WARNING ){
            terminal_io()->print(" - ");
            terminal_io()->print( driver_warning(warning) );
            security_set_warning( SECURITY_NO_WARNING );
        }
        terminal_io()->print(") ");
        display_data();
        display_time = val;
        cnt = 0;
    }
}

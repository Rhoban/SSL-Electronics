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

static int rotor_last_valid_phase = 0;
static int rotor_angle = 0;

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

void apply_pwm( int low_pin, int high_pin, int pwm ){
    //
    // Commands for the ir2104 controller
    //
    // SD = 0 => HO = 0 and L0 = 0 
    // SD = 1 and IN = 0 => HO = 0 and LO = 1
    // SD = 1 and IN = 1 => HO = 1 and LO = 0
    //
    if( pwm == 0 ){
        digitalWrite(low_pin, HIGH);
        pwmWrite(high_pin, 0);
    }else{
        digitalWrite(low_pin, HIGH);
        pwmWrite(high_pin, pwm);
    }
}

void disable_all_motors(){
    digitalWrite(U_SD_PIN, LOW);
    digitalWrite(V_SD_PIN, LOW);
    digitalWrite(W_SD_PIN, LOW);

    pwmWrite(U_IN_PIN, 0);
    pwmWrite(V_IN_PIN, 0);
    pwmWrite(W_IN_PIN, 0);
} 


static void set_pwm_on_all_phases(int u, int v, int w)
{
    if (!motor_on) {
        disable_all_motors();
    } else {
        apply_pwm(U_SD_PIN, U_IN_PIN, u);
        apply_pwm(V_SD_PIN, V_IN_PIN, v);
        apply_pwm(W_SD_PIN, W_IN_PIN, w);
    }
}

void motor_tick_irq();
void compute_encoder_value_when_rotor_is_at_origin();




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
    motor_on = enable;
    
    if (value > 100) value = 100;
    if (value < 0 ) value = 0;

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

TERMINAL_PARAMETER_BOOL(mdb, "", false);

void motor_tick_irq()
{
    // motor_tick(true);
}

TERMINAL_PARAMETER_INT(sphase, "", 0);

TERMINAL_PARAMETER_BOOL(rotor_enc, "", true);

static int last_tmp = 0;
static int phase = -1;


void compute_current_phase(){
    // Current phase
    phase = hall_phases[hall_value()];
    if (phase != hall_current_phase) {
        hall_last_change_moving = millis();
        hall_last_change = millis();
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

static float theta = 0.0; // Angular position
static float theta_open_loop = 0.0; // Angular position
static float old_theta = 0.0;
static float angle_origin = 0;

void compute_rotor_position(){
    theta = ( encoder_to_turn() - angle_origin );
}

static int last_display_time = 0;
static float dt;
static float speed;
static float cumul_dt = 0.0;

static float direct_current_c; // direct current consign
static float quadrature_current_c; // quadrature current consign
static float speed_error;
static float integral_speed_error = 0.0;

static float direct_voltage_c; // direct current consign
static float quadrature_voltage_c; // quadrature current consign
static float direct_current_error;
static float integral_direct_current_error = 0.0;
static float direct_current;
static float quadrature_current_error;
static float integral_quadrature_current_error = 0.0;
static float quadrature_current;

static float phase_voltage_u;
static float phase_voltage_v;
static float phase_voltage_w;


void display_some_message(bool irq){
    if (((millis() - last_display_time) > 500) && mdb && !irq) {
        last_display_time = millis();
        terminal_io()->print("theta : ");
        terminal_io()->print(theta);
        terminal_io()->print(", velocity : ");
        terminal_io()->print(speed);
        terminal_io()->print(", cumul_dt : ");
        terminal_io()->print(cumul_dt);
        terminal_io()->print(", dt : ");
        terminal_io()->print(dt);
        terminal_io()->print(", speed_error : ");
        terminal_io()->print(speed_error);
        terminal_io()->print(", quad_volt : ");
        terminal_io()->print(quadrature_voltage_c);

        terminal_io()->print("phase_voltage_u : ");
    terminal_io()->print( phase_voltage_u );
    terminal_io()->print("phase_voltage_v : ");
    terminal_io()->print( phase_voltage_v );
    terminal_io()->print("phase_voltage_w : ");
    terminal_io()->print( phase_voltage_w );
    terminal_io()->println();


        terminal_io()->println();
    }
}


void make_safety_work(){
    if ((millis() - hall_last_change) > 500 && hall_current_phase == -1) {
        security_set_error(SECURITY_HALL_MISSING);
    }

    if(
        speed != 0 && 
        (millis() - hall_last_change_moving) > 1.0/fabs(speed)
    ){
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


void compute_rotor_velocity(){
    cumul_dt += dt;
    if( cumul_dt >= 50.0 ){
        speed = (theta - old_theta)*1000.0/cumul_dt; // To Improve !
        old_theta = theta;
        cumul_dt = 0.0;
    }
}

TERMINAL_PARAMETER_FLOAT(k_pos_p, "Posiition P", 10.0);
TERMINAL_PARAMETER_FLOAT(k_speed_ff, "Speed FF", 0.0);
TERMINAL_PARAMETER_FLOAT(k_speed_p, "Speed P", 0.5);
TERMINAL_PARAMETER_FLOAT(k_speed_i, "Speed I", 0.0);
TERMINAL_PARAMETER_FLOAT(k_current_p, "Current P", 1.0);
TERMINAL_PARAMETER_FLOAT(k_current_i, "Current I", 0.0);
TERMINAL_PARAMETER_FLOAT(theta_c, "Angular position consign", 0.5);


static float speed_ff; // Speed feedforward
static float speed_p; // Speed proportional
static float speed_c; // Speed consign

#define MAX_SPEED_CONSIGN 5.0 // Nb turn . s^-1


TERMINAL_PARAMETER_BOOL(manual_speed, "Enable manual Speed consign", true);
TERMINAL_PARAMETER_FLOAT(speed_csg, "Speed consign", 1.0);


void compute_velocity_consign(){
    if(!manual_speed){
        // w* = wff + pos_p * ( theta* - tetha )
        speed_ff = k_speed_ff * (theta_c/dt);
        speed_p = k_pos_p * ( theta_c - theta );
        // speed_c = speed_ff + speed_p ;
    }else{    
        speed_c = speed_csg; // TODO
    }
    if( fabs(speed_c) > MAX_SPEED_CONSIGN ){
        speed_c = ((speed_c<0)?-1:1) * MAX_SPEED_CONSIGN;
    } 
}

void compute_direct_quadrature_current_consign(){
    speed_error = speed_c - speed;
    integral_speed_error += ( dt * speed_error );

    direct_current_c = 0; // direct current consign. Always equals to 0.
    quadrature_current_c = (
        k_speed_p * speed_error + k_speed_i * integral_speed_error
    ); // quadrature current consign
}



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

#define MAX_VOLTAGE 13
#define HALF_MAX_VOLTAGE MAX_VOLTAGE/2 

void convert_direct_and_quadrature_voltage_to_phase_voltage(){
    // ( vu )                ( c1 -s1 )
    // ( vv ) = sqrt(2/3) .  ( c2 -s2 ) . ( vd )
    // ( vw )                ( c3 -s3 )   ( vq )
    //
    // c1 = cos( theta )   s1 = cos( theta ) 
    // c2 = cos( theta - 2pi/3 )   s2 = cos( theta - 2pi/3 ) 
    // c3 = cos( theta + 2pi/3 )   s3 = cos( theta + 2pi/3 ) 

    const float sqrt_2_3 = 0.816496580927726;

    #define NB_POSITIVE_MAGNETS 8
    #define NB_BOBINES 12
    #define NB_PHASES 3
    #define NB_BOBINES_BY_PHASE (NB_BOBINES/NB_PHASES) // 12/3
    float r = theta - ( (int) ( theta / NB_POSITIVE_MAGNETS ) );
    float theta_park = NB_POSITIVE_MAGNETS*r;

    theta_park -= ( (int) theta_park );
    if( theta_park < 0 ) theta_park += 1;

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

    // Before computing phase voltage, we need to evaluate the maximum of the 
    // voltage whe theta is going from 0 to  2pi.
    // If that maximum it too important, then we scale the voltage in such a 
    // way the maximum go to MAX_PHASE_VOLTAGE
    // 
    // MAX PHASE VOLTAGE
    //  phase_coltage_u = sqrt_2_3 * (c1 * direct_voltage_c - s1 * quadrature_voltage_c);
    //
    // J = max( c * a - s * b ) ?
    //
    // max when  deriv( c*a - s*b ) = -s*a -c*b = 0
    // so
    // a * J = c * ( a^2 + b^2 )     and     c = a * J /(a^2+b^2)
    // b * J = s * ( -a^2 -b^2 )     and     s = - b * J /(a^2+b^2)
    //
    // 1 = c^2 + s^2 = J^2 ( a^2 + b^2 )/(a^2+b^2)^2
    // so
    // J^2 = a^2 + b^2
    static float max_voltage = sqrt_2_3 * sqrt(
        direct_voltage_c * direct_voltage_c 
        +
        quadrature_voltage_c * quadrature_voltage_c 
    );
    if( max_voltage > HALF_MAX_VOLTAGE ){
        terminal_io()->print( "MAX VOLTAGE !" );
        terminal_io()->println();
        direct_voltage_c = ( HALF_MAX_VOLTAGE / max_voltage ) * direct_voltage_c;
        quadrature_voltage_c = ( HALF_MAX_VOLTAGE / max_voltage ) * quadrature_voltage_c;
    }

	phase_voltage_u = sqrt_2_3 * (c1 * direct_voltage_c - s1 * quadrature_voltage_c);
	phase_voltage_v = sqrt_2_3 * (c2 * direct_voltage_c - s2 * quadrature_voltage_c);
	phase_voltage_w = sqrt_2_3 * (c3 * direct_voltage_c - s3 * quadrature_voltage_c);

//    terminal_io()->print("phase_voltage_u : ");
//    terminal_io()->print( phase_voltage_u );
//    terminal_io()->print("phase_voltage_v : ");
//    terminal_io()->print( phase_voltage_v );
//    terminal_io()->print("phase_voltage_w : ");
//    terminal_io()->print( phase_voltage_w );
//    terminal_io()->println();

}


static float time = 0;


#define DT_MIN 1.0
void update_dt(){
    float new_time = millis();
    dt = new_time - time;
    if( dt <= DT_MIN ) return; //dt = 0.00001;
    time = new_time;
}

        
void compute_phase_current(){
    // TODO : to make in electronics
    // Not present in electronic
}
        
void convert_phase_current_to_direct_and_quadrature_current(){
    // TODO : when phase current will be measured, we need to implement
    // the conversion from pahse current to direct and quadrature current
    direct_current = 0; //direct_current_c;
    quadrature_current = 0; //quadrature_current_c;
}

static int phase_pwm_u = 0;
static int phase_pwm_v = 0;
static int phase_pwm_w = 0;

void convert_direct_and_quadrature_voltage_to_phase_pwm(){
    float min_voltage = phase_voltage_u;
    if( min_voltage > phase_voltage_v) min_voltage = phase_voltage_v;
    if( min_voltage > phase_voltage_w) min_voltage = phase_voltage_w;

    int pwm_max = PWM_MAX * motor_pwm / 100;
    if( pwm_max < 0 ) pwm_max = 0;

/*
    float u = phase_voltage_u/MAX_VOLTAGE;
    float v = phase_voltage_v/MAX_VOLTAGE;
    float w = phase_voltage_w/MAX_VOLTAGE;

    phase_pwm_u = u * PWM_MAX;
    phase_pwm_v = v * PWM_MAX;
    phase_pwm_w = w * PWM_MAX;
*/
    // We center the sinusoide to have phase from 0V to MAX_VOLTAGE
    phase_pwm_u = ((phase_voltage_u - min_voltage)/MAX_VOLTAGE) * pwm_max;
    phase_pwm_v = ((phase_voltage_v - min_voltage)/MAX_VOLTAGE) * pwm_max;
    phase_pwm_w = ((phase_voltage_w - min_voltage)/MAX_VOLTAGE) * pwm_max;
}

static int phase_pwm_u_save = 0;
static int phase_pwm_v_save = 0;
static int phase_pwm_w_save = 0;


static bool tare_is_set = false;
static bool tare_is_done = false;
static int last_tare_time = 0;

void display_tare_information(){
        terminal_io()->print("angle origin : ");
        terminal_io()->print(angle_origin);
        terminal_io()->print(", phase pwm u : ");
        terminal_io()->print(phase_pwm_u_save);
        terminal_io()->print(", phase pwm v : ");
        terminal_io()->print(phase_pwm_v_save);
        terminal_io()->print(", phase pwm w : ");
        terminal_io()->print(phase_pwm_w_save);
        terminal_io()->println();
}

void compute_encoder_value_when_rotor_is_at_origin(){
    if( millis() - last_tare_time > 1000 ){
        angle_origin = encoder_to_turn();
        theta = 0.0;
        display_tare_information();
        tare_is_done = true;
        tare_is_set = false;
    }
}

void motor_tick(bool irq)
{
    update_dt();
    if( dt <= DT_MIN ) return;

    compute_current_phase();

    if( tare_is_set ){
        compute_encoder_value_when_rotor_is_at_origin();
    }

    if( tare_is_done ){
    //if (phase >= 0 && phase < 6) {
        // Inputs
        // compute_rotor_angle_from_hall();
        compute_rotor_position();
        compute_rotor_velocity();
       
        #define MAX_COUPLE 1
        #define CLOSED_LOOP 0
        #if CLOSED_LOOP
        // compute_phase_current(); TODO when electronic is able to get current
        convert_phase_current_to_direct_and_quadrature_current();

        // Outputs
        compute_velocity_consign();
        compute_direct_quadrature_current_consign();
        compute_direct_and_quadrature_voltage_consign();
        convert_direct_and_quadrature_voltage_to_phase_voltage();
        convert_direct_and_quadrature_voltage_to_phase_pwm();
        
        // Apply control
        set_pwm_on_all_phases( phase_pwm_u, phase_pwm_v, phase_pwm_w );
        #else
        // IN OPEN LOOP 
        #if MAX_COUPLE != 1
            float save_theta = theta;
            theta_open_loop += (( speed_csg * dt)/1000.0 );
            theta = theta_open_loop;
        
            direct_voltage_c = HALF_MAX_VOLTAGE; 
            quadrature_voltage_c = 0;
        #else
            direct_voltage_c = 0.0;
            quadrature_voltage_c = HALF_MAX_VOLTAGE; 
        #endif
        convert_direct_and_quadrature_voltage_to_phase_voltage();
        convert_direct_and_quadrature_voltage_to_phase_pwm();
        set_pwm_on_all_phases( phase_pwm_u, phase_pwm_v, phase_pwm_w );
        #if MAX_COUPLE != 1
            theta = save_theta;
        #endif
        #endif

        display_some_message(irq);
    //} else {
        // XXX: This is not a normal state, not sure what should be done
        // in this situation
    //    set_pwm_on_all_phases(0, 0, 0);
    //}
    //make_safety_work();
    }    
    if( !motor_on ){
        disable_all_motors();
    }
}

TERMINAL_COMMAND(safe, "Safe mode")
{
    if (argc) {
        safe_mode = (atoi(argv[0]) != 0);
    } else {
        terminal_io()->println("Usage: safe [0|1]");
    }
}

TERMINAL_COMMAND(pwm, "Motor set Maximal PWM")
{
    if (argc > 0) {
        motor_set(true, atoi(argv[0]));
    } else {
        terminal_io()->print("usage: pwm [0-100] (current: ");
        terminal_io()->print(abs(motor_pwm));
        terminal_io()->print(")");
        terminal_io()->println();
    }
}

TERMINAL_COMMAND(tare, "Tare origin")
{
    tare_is_set = false;
    if( ! motor_on or motor_pwm <= 0 ){
        terminal_io()->print("First enable motors by defining a pwm. For example :" );
        terminal_io()->println();
        terminal_io()->println();
        terminal_io()->print("    pwm 10");
        terminal_io()->println();
    }else{
        last_tare_time = millis();
        theta = 0;
        direct_voltage_c = HALF_MAX_VOLTAGE; 
        quadrature_voltage_c = 0;
        convert_direct_and_quadrature_voltage_to_phase_voltage();
        convert_direct_and_quadrature_voltage_to_phase_pwm();
        set_pwm_on_all_phases( phase_pwm_u, phase_pwm_v, phase_pwm_w );
        phase_pwm_u_save = phase_pwm_u;
        phase_pwm_v_save = phase_pwm_v;
        phase_pwm_w_save = phase_pwm_w;
        tare_is_set = true;
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

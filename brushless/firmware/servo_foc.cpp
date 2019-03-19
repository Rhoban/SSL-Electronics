#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include "hardware.h"
#include "motor.h"
#include "motor_foc.h"
#include "servo_foc.h"
#include "encoder.h"
#include "security.h"
#include "ssl.h"

void servo_foc_init()
{
}

TERMINAL_PARAMETER_BOOL(manual_speed, "Enable manual Speed consign", MANUAL_SPEED);

static bool servo_ticking = false;
static bool open_loop = OPEN_LOOP_FOC;

void set_open_loop( bool value ){
    open_loop = value;
}

bool is_open_loop(){
    return open_loop;
}

// begin of automatic code - generated by integer_calculus.py
#define OUTPUT_VOLTAGE_Q_SCALE 1
#define REFERENCE_VOLTAGE_Q_SCALE 262144
#define VOLTAGE_Q_SCALE 262144
#define VOLTAGE_P_SCALE 524288
#define LIMITED_SPEED_ERROR_SCALE 33554432
#define SPEED_ERROR_SCALE 33554432
#define SPEED_C_SCALE 67108864
#define SPEED_P_SCALE 134217728
#define LIMITED_THETA_ERROR_SCALE 32768
#define THETA_ERROR_SCALE 32768
#define THETA_C_SCALE 65536
#define K_POS_P_SCALE 134217728
#define SPEED_I_SCALE 134217728
#define SPEED_I__ACC_SUM_SCALE 134217728
#define SPEED_LOAD_SCALE 8589934592
#define K_POS_I_SCALE 16777216
#define K_SPEED_P_SCALE 1048576
#define VOLTAGE_I_SCALE 1048576
#define VOLTAGE_I__ACC_SUM_SCALE 1048576
#define VOLTAGE_LOAD_SCALE 134217728
#define K_SPEED_I_SCALE 262144
#define ELECTROMAGNETIC_FORCE_SCALE 524288
#define K_FEM_SCALE 16777216
// end of automatic code - generated by integer_calculus.py

// begin of automatic code - generated by integer_calculus.py
#define SPEED_C_SCALE 67108864
#define SPEED_CSG_SCALE 134217728
// end of automatic code - generated by integer_calculus.py

// begin of automatic code - generated by integer_calculus.py
static int speed_csg = 0*SPEED_CSG_SCALE;
// end of automatic code - generated by integer_calculus.py

// begin of automatic code - generated by integer_calculus.py
int theta_c = 0*THETA_C_SCALE;
int k_pos_p = K_POS_P*K_POS_P_SCALE;
int speed_i = 0*SPEED_I_SCALE;
int k_pos_i = K_POS_I*K_POS_I_SCALE;
int k_speed_p = K_SPEED_P*K_SPEED_P_SCALE;
int voltage_i = 0*VOLTAGE_I_SCALE;
int k_speed_i = K_SPEED_I*K_SPEED_I_SCALE;
int k_fem = KFEM*K_FEM_SCALE;
// end of automatic code - generated by integer_calculus.py


static int output_voltage_d = 0;


// begin of automatic code - generated by integer_calculus.py
int output_voltage_q = 0*OUTPUT_VOLTAGE_Q_SCALE;
int reference_voltage_q = 0*REFERENCE_VOLTAGE_Q_SCALE;
int voltage_q = 0*VOLTAGE_Q_SCALE;
int voltage_p = 0*VOLTAGE_P_SCALE;
int limited_speed_error = 0*LIMITED_SPEED_ERROR_SCALE;
int speed_error = 0*SPEED_ERROR_SCALE;
int speed_c = 0*SPEED_C_SCALE;
int speed_p = 0*SPEED_P_SCALE;
int limited_theta_error = 0*LIMITED_THETA_ERROR_SCALE;
int theta_error = 0*THETA_ERROR_SCALE;
int speed_i__acc_sum = 0*SPEED_I__ACC_SUM_SCALE;
int speed_load = 0*SPEED_LOAD_SCALE;
int voltage_i__acc_sum = 0*VOLTAGE_I__ACC_SUM_SCALE;
int voltage_load = 0*VOLTAGE_LOAD_SCALE;
int electromagnetic_force = 0*ELECTROMAGNETIC_FORCE_SCALE;
// end of automatic code - generated by integer_calculus.py

static int max_output_voltage = 0;
static int speed=0;
static int theta=0;

static int max_theta = (int) (MAX_THETA_LIMIT*ONE_TURN_THETA);
static int min_theta = (int) (MIN_THETA_LIMIT*ONE_TURN_THETA);

        
void reset_asservissement(){
    speed_i = 0;
    voltage_i = 0;
}

void restart(){
    reset_asservissement();
    motor_set( true, 0 );
}

TERMINAL_COMMAND(restart, "Restart FOC")
{
    restart();
}

void rest_consign_and_asservissement(){
    theta_c = 0;
    reset_asservissement();
}

void control_with_vectorial_command(){
    if( servo_ticking ){
        return;
    }
    servo_ticking = true;

    if( ! get_serv_flag() ){
        servo_ticking = false;
        return;
    }

    theta = rotor_angle();
    _Static_assert( SPEED_NOMRALISATION==1048576, "");
    speed = encoder_to_speed();
    
    if( motor_is_tared() and motor_foc_is_on() ){
        #ifdef STOP_OUTSIDE_LIMITS 
        if( theta < min_theta or theta > max_theta ){
            motor_set( false, 0 );
        }
        #endif

        // Compute velocity consign 
        if(!manual_speed){
            // begin of automatic code - generated by integer_calculus.py
            theta_error = ((-theta)*2 + theta_c/2);
            limited_theta_error = ( (theta_error > 29491) ? 29491 : ( (theta_error < -29491) ? -29491 : theta_error ) );
            speed_p = ((limited_theta_error/1)*(k_pos_p/32768));
            speed_load = ((((limited_theta_error/1)*(k_pos_i/32768))/65536)*((2684354)/64));
            speed_i__acc_sum = (speed_i/1 + speed_load/64);
            speed_i = (( (speed_i__acc_sum > 1342177280) ? 1342177280 : ( (speed_i__acc_sum < -1342177280) ? -1342177280 : speed_i__acc_sum ) )*1);
            speed_c = ( ((speed_p/2 + speed_i/2) > 671088640) ? 671088640 : ( ((speed_p/2 + speed_i/2) < -671088640) ? -671088640 : (speed_p/2 + speed_i/2) ) );
            // end of automatic code - generated by integer_calculus.py
        }else{
            // begin of automatic code - generated by integer_calculus.py
            speed_c = (speed_csg/2);
            // end of automatic code - generated by integer_calculus.py
        }

        // begin of automatic code - generated by integer_calculus.py
        speed_error = ((-speed)*32 + speed_c/2);
        limited_speed_error = ( (speed_error > 80530636) ? 80530636 : ( (speed_error < -80530636) ? -80530636 : speed_error ) );
        voltage_p = ((limited_speed_error/2048)*(k_speed_p/32768));
        voltage_load = ((((limited_speed_error/2048)*(k_speed_i/32768))/32768)*((2684354)/64));
        voltage_i__acc_sum = (voltage_i/1 + voltage_load/128);
        voltage_i = (( (voltage_i__acc_sum > 1073741824) ? 1073741824 : ( (voltage_i__acc_sum < -1073741824) ? -1073741824 : voltage_i__acc_sum ) )*1);
        electromagnetic_force = ((speed/1024)*(k_fem/32768));
        voltage_q = ((electromagnetic_force/2 + voltage_i/4)/1 + voltage_p/2);
        reference_voltage_q = ( (voltage_q > 268435456) ? 268435456 : ( (voltage_q < -268435456) ? -268435456 : voltage_q ) );
        output_voltage_q = (reference_voltage_q/262144);
        // end of automatic code - generated by integer_calculus.py

        if( max_output_voltage<output_voltage_q){
            max_output_voltage = output_voltage_q;
        }
        output_voltage_d = 0;
        direct_quadrature_voltage_set(output_voltage_d, output_voltage_q);
    }else{
        if(motor_is_tared()){
            reset_asservissement(); 
        }else{
            rest_consign_and_asservissement();
        }
    }
    
    reset_serv_flag();
    servo_ticking = false;
}

void servo_foc_tick()
{
    if (security_get_error() != SECURITY_NO_ERROR) {
        motor_foc_set(false, 0);
    } else {
        if(open_loop){
            reset_serv_flag();
        }else{
            control_with_vectorial_command();
        }
    }
}

void set_speed_csg( float speed_csg_input );

void servo_foc_set(bool enable, float target, int16_t pwm)
{
    if (!enable) {
        motor_foc_set(false, 0);
        security_set_error(SECURITY_NO_ERROR);
    }else{
        set_speed_csg( target );
    }
}

void servo_foc_emergency(){
    servo_foc_set(false, 0);
}
void servo_foc_stop(){
    servo_foc_set(false, 0);
    motor_foc_set(true, 0);
}

TERMINAL_COMMAND(kpp, "Set P for position PID")
{
    if (argc == 1) {
        // begin of automatic code - generated by integer_calculus.py
        float k_pos_p_input = atof(argv[0]);
        if(k_pos_p_input < 0.0) k_pos_p_input = 0.0;
        if(k_pos_p_input > 11.11111111111111) k_pos_p_input = 11.11111111111111;
        k_pos_p = (int) (k_pos_p_input * K_POS_P_SCALE);
        // end of automatic code - generated by integer_calculus.py
    } else {
        if(argc==0){
            terminal_io()->println(((double)k_pos_p)/K_POS_P_SCALE);
            return;
        }
        terminal_io()->println("Usage: kpp [0.0 - 11.11]");
    }
}
TERMINAL_COMMAND(kpi, "Set I for position PID")
{
    if (argc == 1) {
        // begin of automatic code - generated by integer_calculus.py
        float k_pos_i_input = atof(argv[0]);
        if(k_pos_i_input < 0.0) k_pos_i_input = 0.0;
        if(k_pos_i_input > 111.1111111111111) k_pos_i_input = 111.1111111111111;
        k_pos_i = (int) (k_pos_i_input * K_POS_I_SCALE);
        // end of automatic code - generated by integer_calculus.py
    } else {
        if(argc==0){
            terminal_io()->println(((double)k_pos_i)/K_POS_I_SCALE);
            return;
        }
        terminal_io()->println("Usage: kpi [0.0 - 111.11]");
    }
}
TERMINAL_COMMAND(ksp, "Set P for speed PID")
{
    if (argc == 1) {
        // begin of automatic code - generated by integer_calculus.py
        float k_speed_p_input = atof(argv[0]);
        if(k_speed_p_input < 0.0) k_speed_p_input = 0.0;
        if(k_speed_p_input > 1280.0) k_speed_p_input = 1280.0;
        k_speed_p = (int) (k_speed_p_input * K_SPEED_P_SCALE);
        // end of automatic code - generated by integer_calculus.py
    } else {
        if(argc==0){
            terminal_io()->println(((double)k_speed_p)/K_SPEED_P_SCALE);
            return;
        }
        terminal_io()->println("Usage: ksp [ 0.0 - 1280.0 ]");
    }
}
TERMINAL_COMMAND(ksi, "Set I for speed PID")
{
    if (argc == 1) {
        // begin of automatic code - generated by integer_calculus.py
        float k_speed_i_input = atof(argv[0]);
        if(k_speed_i_input < 0.0) k_speed_i_input = 0.0;
        if(k_speed_i_input > 3011.7647058823527) k_speed_i_input = 3011.7647058823527;
        k_speed_i = (int) (k_speed_i_input * K_SPEED_I_SCALE);
        // end of automatic code - generated by integer_calculus.py
    } else {
        if(argc==0){
            terminal_io()->println(((double)k_speed_i)/K_SPEED_I_SCALE);
            return;
        }
        terminal_io()->println("Usage: ksi [ 0.0 - 3011.764 ]");
    }
}
TERMINAL_COMMAND(kem, "Set I for speed PID")
{
    if (argc == 1) {
        // begin of automatic code - generated by integer_calculus.py
        float k_fem_input = atof(argv[0]);
        if(k_fem_input < 0.0) k_fem_input = 0.0;
        if(k_fem_input > 122.88) k_fem_input = 122.88;
        k_fem = (int) (k_fem_input * K_FEM_SCALE);
        // end of automatic code - generated by integer_calculus.py
    } else {
        if(argc==0){
            terminal_io()->println(((double)k_fem)/K_FEM_SCALE);
            return;
        }
        terminal_io()->println("Usage: kem [ 0.0 - 122.88 ]");
    }
}

TERMINAL_COMMAND(theta, "Get motor angle")
{
    if (argc == 0) {
        // begin of automatic code - generated by integer_calculus.py
        terminal_io()->println(((double)theta)/ONE_TURN_THETA);
        // end of automatic code - generated by integer_calculus.py
    } else {
        terminal_io()->println("Usage: theta");
    }
}

TERMINAL_COMMAND(theta_c, "Set angle consign")
{
    if (argc == 1) {
        // begin of automatic code - generated by integer_calculus.py
        float theta_c_input = atof(argv[0]);
        if(theta_c_input < -24000.0) theta_c_input = -24000.0;
        if(theta_c_input > 24000.0) theta_c_input = 24000.0;
        theta_c = (int) (theta_c_input * THETA_C_SCALE);
        // end of automatic code - generated by integer_calculus.py
    } else {
        if(argc==0){
            terminal_io()->println(((double)theta_c)/THETA_C_SCALE);
            return;
        }
        terminal_io()->println("Usage: theta_c [ -24000.0 - 24000.0 ]");
    }
}

TERMINAL_COMMAND(info, "Info")
{
    terminal_io()->print("  motor name : ");
    terminal_io()->println(MOTOR_NAME);
    terminal_io()->print("  k_fem : ");
    terminal_io()->println(((double)k_fem)/K_FEM_SCALE);
    terminal_io()->print("  k speed p : ");
    terminal_io()->println(((double)k_speed_p)/K_SPEED_P_SCALE);
    terminal_io()->print("  k speed i : ");
    terminal_io()->println(((double)k_speed_i)/K_SPEED_I_SCALE);
    terminal_io()->print("  k pos p : ");
    terminal_io()->println(((double)k_pos_p)/K_POS_P_SCALE);
    terminal_io()->print("  k pos i : ");
    terminal_io()->println(((double)k_pos_i)/K_POS_I_SCALE);
    terminal_io()->print("  high speed pwm : ");
    terminal_io()->println(CONFIG_PWM);
    terminal_io()->print("  manual speed : ");
    terminal_io()->println(manual_speed);
    terminal_io()->print("  open loop : ");
    terminal_io()->println(open_loop);
    terminal_io()->print("  motor on : ");
    terminal_io()->println(motor_is_on());

    terminal_io()->print("  speed_error : ");
    terminal_io()->println(speed_error);
    terminal_io()->print("  limited_speed_error : ");
    terminal_io()->println(limited_speed_error);
    terminal_io()->print("  voltage_p : ");
    terminal_io()->println(voltage_p);
    terminal_io()->print("  voltage_load : ");
    terminal_io()->println(voltage_load);
    terminal_io()->print("  voltage_i__acc_sum : ");
    terminal_io()->println(voltage_i__acc_sum);
    terminal_io()->print("  voltage_i : ");
    terminal_io()->println(voltage_i);
    terminal_io()->print("  electromagnetic_force : ");
    terminal_io()->println(electromagnetic_force);
    terminal_io()->print("  voltage_q : ");
    terminal_io()->println(voltage_q);
    terminal_io()->print("  reference_voltage_q : ");
    terminal_io()->println(reference_voltage_q);
    terminal_io()->print("  output_voltage_q : ");
    terminal_io()->println(output_voltage_q);





//      terminal_io()->print("theta error: ");
//      terminal_io()->println(theta_error);
//    terminal_io()->print("theta : ");
//    terminal_io()->println(theta);
//    terminal_io()->print("speed : ");
//    terminal_io()->println(speed);
//    terminal_io()->println(speed_c);
//    terminal_io()->println(speed_error);
//    terminal_io()->print("em : ");
//    terminal_io()->println(electromagnetic_force);
//    terminal_io()->print("out_q : ");
//    terminal_io()->println(output_voltage_q);
//    terminal_io()->print("max_out_q : ");
//    terminal_io()->println(max_output_voltage);
//      terminal_io()->print("s_e : ");
//      terminal_io()->println(speed_error);
//      terminal_io()->print("l_s_e : ");
//      terminal_io()->println(limited_speed_error);
//      terminal_io()->print("v_p : ");
//      terminal_io()->println(voltage_p);
//      terminal_io()->print("v_l : ");
//      terminal_io()->println(voltage_load);
//      terminal_io()->print("v_a : ");
//      terminal_io()->println(voltage_i__acc_sum);
//      terminal_io()->print("v_i : ");
//      terminal_io()->println(voltage_i);
//terminal_io()->println(theta_error);
//terminal_io()->println(limited_theta_error);
//terminal_io()->println(speed_p);
//terminal_io()->println(speed_load);
//terminal_io()->println(speed_i__acc_sum);
//terminal_io()->println(speed_i);
//terminal_io()->println(speed_c);

}

void set_speed_csg( float speed_csg_input ){
    // begin of automatic code - generated by integer_calculus.py
    if(speed_csg_input < -10.0) speed_csg_input = -10.0;
    if(speed_csg_input > 10.0) speed_csg_input = 10.0;
    speed_csg = (int) (speed_csg_input * SPEED_CSG_SCALE);
    // end of automatic code - generated by integer_calculus.py
}

TERMINAL_COMMAND(speed_csg, "Set speed consign")
{
    if (argc == 1) {
        float speed_csg_input = atof(argv[0]);
        set_speed_csg( speed_csg_input );
    } else {
        if(argc==0){
            terminal_io()->println((double)speed_csg/SPEED_CSG_SCALE);
            return;
        }
        terminal_io()->println("Usage: speed_csg [ -10.0 - 10.0 ]");
    }
}

/**
 * Current speed [turn/s]
 */
float servo_foc_get_speed(){
    return speed/SPEED_NOMRALISATION;
}

/**
 * Current PWM output [-3000 to 3000]
 */
int servo_foc_get_pwm(){
    return 0; 
}

void servo_foc_set_pid(float kp, float ki, float kd){
}

TERMINAL_COMMAND(set_min_angle, "Set minimum angle")
{
    if( !motor_is_tared() ){
        terminal_io()->print("First define the origin by typing :" );
        terminal_io()->println();
        terminal_io()->println();
        terminal_io()->print("    set_origin");
        terminal_io()->println();
        return;
    }
    if( theta > 0 ){
        terminal_io()->print("Minimal angle should be negative.");
        terminal_io()->println();
        return;
    }
    min_theta = (int)( theta*ONE_TURN_THETA );
}

TERMINAL_COMMAND(open_loop, "Set open loop mode (foc)")
{
    open_loop = true;
}

TERMINAL_COMMAND(closed_loop, "Set closed loop mode (foc)")
{
    open_loop = false;
}

TERMINAL_COMMAND(set_max_angle, "Set maximum angle")
{
    if( !motor_is_tared() ){
        terminal_io()->print("First define the origin by typing :" );
        terminal_io()->println();
        terminal_io()->println();
        terminal_io()->print("    set_origin");
        terminal_io()->println();
        return;
    }
    if( theta < 0 ){
        terminal_io()->print("Maximal angle should be positive.");
        terminal_io()->println();
        return;
    }
    max_theta = (int)( theta*ONE_TURN_THETA );
}

TERMINAL_COMMAND(set_position, "Set position")
{
    if( !motor_is_tared() ){
        terminal_io()->print("First define the origin by typing :" );
        terminal_io()->println();
        terminal_io()->println();
        terminal_io()->print("    tare");
        terminal_io()->println();
        return;
    }
    if( max_theta <= 0){
        terminal_io()->print("First define maximal angle by typing :" );
        terminal_io()->println();
        terminal_io()->println();
        terminal_io()->print("    set_max_angle");
        terminal_io()->println();
        return;
    } 
    if( min_theta >= 0){
        terminal_io()->print("First define minimal angle by typing :" );
        terminal_io()->println();
        terminal_io()->println();
        terminal_io()->print("    set_min_angle");
        terminal_io()->println();
        return;
    } 
    char* endptr = NULL;
    float consign = strtod(argv[0], &endptr);
    if( consign == 0.0 and endptr == argv[0] ){
        terminal_io()->print("Invalid parameter.");
        terminal_io()->println();
        return;
    }
    int theta_csg = (int)(consign * ONE_TURN_THETA);
    if( theta_csg < min_theta ){
        terminal_io()->print("Angle is too small." );
        terminal_io()->println();
        return;
    }
    if( theta_csg > max_theta ){
        terminal_io()->print("Angle is too big." );
        terminal_io()->println();
        return;
    }

    if (argc > 0) {
        // begin of automatic code - generated by integer_calculus.py
        theta_c = (theta_csg*4);
        // end of automatic code - generated by integer_calculus.py
    } else {
        terminal_io()->println("Usage: set_position angle");
    }
}

TERMINAL_COMMAND(limits, "Print limits")
{
    terminal_io()->print("min angle : " );
    terminal_io()->println(((double)min_theta)/ONE_TURN_THETA);
    terminal_io()->print("max angle : " );
    terminal_io()->println(((double)max_theta)/ONE_TURN_THETA);
    terminal_io()->println();
}

void servo_set_speed_consign_foc( float speed ){
    set_speed_csg( speed );
}

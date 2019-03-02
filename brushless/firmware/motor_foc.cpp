#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include "encoder.h"
#include "hardware.h"
#include "motor_foc.h"
#include "ssl.h"
#include "sin_lut.h"
#include "security.h"
#include "errors.h"

inline int mod(int n, int d){
    int r = n%d;
    return (r>=0) ? r : r+d;
}

void display(bool);
void display_warning();

// Target PWM speed [0-3000]
static bool motor_on = false;

static unsigned int count_irq = 0;
static bool motor_flag = false;
static bool serv_flag = false;

void reset_serv_flag(){
    serv_flag = false;
}

bool get_serv_flag(){
    return serv_flag;
}

void motor_irq(){
    count_irq = (count_irq + 1)%(MOTOR_SUB_SAMPLE);
    if( ! count_irq ){
        if( motor_flag ){
            security_set_warning(WARNING_MOTOR_LAG);
        }
        if( serv_flag ){
            security_set_warning(WARNING_SERVO_LAG);
        }
        motor_flag = true;
        serv_flag = true;
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

void motor_foc_init()
{
    // Configuring timers
    _init_timer(2);
    _init_timer(3);

    // Initalizing motor pins
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

void motor_foc_set(bool enable, int value)
{
    motor_on = enable;
    if(!motor_on){
        direct_quadrature_voltage_set(0,0);
    }
}

static int direct_voltage_c = 0;
static int quadrature_voltage_c = 0;

/*
 * vd [-REFERENCE_VOLTAGE, REFERENCE_VOLTAGE]
 * vq in [-REFERENCE_VOLTAGE, REFERENCE_VOLTAGE]
 * 
 * returns maximal difference beetween two phase volge. Result is in 
 * [0, REFERENCE_VOLTAGE * REFERENCE_VOLTAGE] 
 */
int32_t compute_maximal_voltage_2(int32_t vd, int32_t vq){
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

    return 2 * ( vd * vd + vq * vq );
}

// TODO : Ce n'est pas mieux de multiplier systématiquement par sqrt(2) ? 
void direct_quadrature_voltage_set(int vd, int vq ){
    int max_voltage_2 = compute_maximal_voltage_2(vd, vq);
    direct_voltage_c = vd;
    quadrature_voltage_c = vq;
    if( max_voltage_2 > REFERENCE_VOLTAGE * REFERENCE_VOLTAGE ){
        //terminal_io()->println( "M" ); //AX VOLTAGE !" );
        //terminal_io()->println();
        int max_voltage = (int) ( sqrt(max_voltage_2) );
        direct_voltage_c = ( 
            REFERENCE_VOLTAGE * direct_voltage_c 
        )/ max_voltage;
        quadrature_voltage_c = (
            REFERENCE_VOLTAGE * quadrature_voltage_c
        )/max_voltage;
    }
}

enum TareProcess {
    TARE_NOT_SET,
    TODO_TARE,
    DEFINE_ORIGIN,
    TARE_ANGLE,
    COMPUTE_ANGLE,
    TARE_IS_DONE
};

static int last_tare_time = 0;
static int tare_is_set = false;
static TareProcess tare_state = TARE_NOT_SET;

/*
 * Angle consign in turn
 *
 * VALUE : [INT_MIN/ONE_TURN_THETA, INT_MAX/ONE_TURN_THETA]
 * SCALE : ONE_TURN_THETA
 *
 */
static int theta_c;
static bool go_theta = false;

void reset_vectorial();

TERMINAL_COMMAND(go_theta, "Set theta")
{
    if (argc == 1) {
        reset_vectorial();
        float val = atof(argv[0]);
        theta_c = (int) ( val * ONE_TURN_THETA );
        direct_quadrature_voltage_set( HALF_REFERENCE_VOLTAGE, 0);
        go_theta = true;
    }
}

TERMINAL_COMMAND(dqv, "Set direct and quadrature voltage")
{
    bool in_err = true;
    if (argc == 2) {
        if( tare_state != TARE_IS_DONE ){
            terminal_io()->println("Please tare the motor first, by typing : ");
            terminal_io()->println("  tare");
            return;
        }

        int vd = atoi(argv[0]);
        int vq = atoi(argv[1]);
        if( -100 <= vd and -100 <= vq and vd <= 100 and vq <= 100 ){
            vd = (REFERENCE_VOLTAGE*vd)/100;
            vq = (REFERENCE_VOLTAGE*vq)/100;
            int max_voltage_2 = compute_maximal_voltage_2(vd, vq);
            if( max_voltage_2 <= REFERENCE_VOLTAGE*REFERENCE_VOLTAGE ){
                reset_vectorial();
                direct_quadrature_voltage_set(vd, vq);
                in_err = false;
            }
        }
    }
    if( in_err ){
        terminal_io()->print("usage: dqv [0-100] [0-100]");
        terminal_io()->println("  first parameter d : direct voltage");
        terminal_io()->println("  second parameter q : quadrature voltage");
        terminal_io()->print(  "  sqrt(2)*sqrt(d^2 + q^2) <= 1.0" );
        terminal_io()->println(".");
    }
}

static int angle_origin = 0;
static int vectorial_pwm = CONFIG_PWM;
static int save_vectorial_pwm = CONFIG_PWM;

static int theta_park; //D/

/*
 * Angle in turn,
 * 
 * VALUE : [INT_MIN/ONE_TURN_THETA, INT_MAX/ONE_TURN_THETA]
 * SCALE :  ONE_TURN_THETA 
 */
static int theta_s; //D/

/*
 * Angle of each phase in turn.
 *
 * VALUE : [INT_MIN/ONE_TURN_THETA, INT_MAX/ONE_TURN_THETA]
 * SCALE :  ONE_TURN_THETA 
 */
static int16_t t1; //D/
static int16_t t2; //D/
static int16_t t3; //D/

/*
 * Cos and sinus 
 *
 * Value : [0, 1]
 * SCALE : SIN_OUPUT_MAX 
 */
static int c1; //D/
static int s1; //D/
static int c2; //D/
static int s2; //D/
static int c3; //D/
static int s3; //D/

static int phase_voltage_u; //D/
static int phase_voltage_v; //D/
static int phase_voltage_w; //D/

static int phase_pwm_u; //D/
static int phase_pwm_v; //D/
static int phase_pwm_w; //D/

/*
 * Apply control command on motors.
 *
 * theta : angle of the rotor in turn
 *   VALUE : [INT_MIN/ONE_TURN_THETA, INT_MAX/ONE_TURN_THETA]
 *   SCALE : ONE_TURN_THETA
 */
void control_motor_with_vectorial( int theta ){

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

    //int r = theta % NB_POSITIVE_MAGNETS;
    //int theta_park = NB_POSITIVE_MAGNETS*r;

    // VALUE : [0, NB_POSITIVE_MAGNETS]
    // SCALE : ONE_TURN_THETA
    theta_park = NB_POSITIVE_MAGNETS * mod( theta, ONE_TURN_THETA ); //D/

    // VALUE : [0, NB_POSITIVE_MAGNETS]
    // SCALE : ONE_TURN_THETA
    t1 = theta_park; //D/
    // VALUE : [-1/3, NB_POSITIVE_MAGNETS-1/3]
    // SCALE : ONE_TURN_THETA
    t2 = theta_park - ONE_TURN_THETA/3; //D/
    // VALUE : [+1/3, NB_POSITIVE_MAGNETS+1/3]
    // SCALE : ONE_TURN_THETA
    t3 = theta_park + ONE_TURN_THETA/3; //D/

    // VALUE : [0, 1]
    // SCALE : SIN_INPUT_RESOLUTION
    static_assert( SIN_INPUT_RESOLUTION <  ONE_TURN_THETA , "");
    t1 = mod(t1, ONE_TURN_THETA)/(ONE_TURN_THETA/SIN_INPUT_RESOLUTION);
    t2 = mod(t2, ONE_TURN_THETA)/(ONE_TURN_THETA/SIN_INPUT_RESOLUTION);
    t3 = mod(t3, ONE_TURN_THETA)/(ONE_TURN_THETA/SIN_INPUT_RESOLUTION);
    
    // display(false);

    // VALUE : [-1, 1]
    // SCALE : SIN_OUPUT_MAX
    c1 = discrete_cos(t1);
    s1 = discrete_sin(t1);
    c2 = discrete_cos(t2);
    s2 = discrete_sin(t2);
    c3 = discrete_cos(t3);
    s3 = discrete_sin(t3);

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

    // MAX phase_voltage is scaled by REFERENCE_VOLTAGE * SIN_OUPUT_MAX
    const int sqrt_2_3__256 = 209; // sqrt_2_3 * 256

    // VALUE : [-1, 1]
    // REFERENCE_VOLTAGE * SIN_OUPUT_MAX
    static_assert( REFERENCE_VOLTAGE * SIN_OUPUT_MAX <= 2147483648, "" );
	
    phase_voltage_u = sqrt_2_3__256 * ( //D/
        (c1 * direct_voltage_c - s1 * quadrature_voltage_c)/256
    );
	phase_voltage_v = sqrt_2_3__256 * ( //D/
        (c2 * direct_voltage_c - s2 * quadrature_voltage_c)/256
    );
	phase_voltage_w = sqrt_2_3__256 * ( //D/
        (c3 * direct_voltage_c - s3 * quadrature_voltage_c)/256
    );

    //display(false);
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

    // VALUE : [-1, 1]
    // REFERENCE_VOLTAGE * SIN_OUPUT_MAX
    int min_voltage = phase_voltage_u;
    if( min_voltage > phase_voltage_v) min_voltage = phase_voltage_v;
    if( min_voltage > phase_voltage_w) min_voltage = phase_voltage_w;

    // VALUE : [0, PWM_SUPREMUM]
    // SCALE : 1
    int pwm_max = ( PWM_SUPREMUM * vectorial_pwm ) / 100;
    if( pwm_max < 0 ) pwm_max = 0;

    #define POWER_GREATER_THAN_PWM_SUPREMUM 4096
    #define SCALE 8192
    static_assert( SCALE== 2*POWER_GREATER_THAN_PWM_SUPREMUM, "");
    static_assert( POWER_GREATER_THAN_PWM_SUPREMUM > PWM_SUPREMUM, "");

    // We center the sinusoide to have phase from 
    // 0 to REFERENCE_VOLTAGE * SIN_OUPUT_MAX / SCALE
    // VALUE : [-PWM_SUPREMUM, PWM_SUPREMUM]
    // SCALE : REFERENCE_VOLTAGE * SIN_OUPUT_MAX / SCALE
    phase_pwm_u = ( (phase_voltage_u - min_voltage)/SCALE ) * pwm_max; //D/
    phase_pwm_v = ( (phase_voltage_v - min_voltage)/SCALE ) * pwm_max; //D/
    phase_pwm_w = ( (phase_voltage_w - min_voltage)/SCALE ) * pwm_max; //D/

    // We compute the pwm from 0 to PWM_SUPREMUM
    // VALUE : [-PWM_SUPREMUM, PWM_SUPREMUM]
    // SCALE : 1
    phase_pwm_u = phase_pwm_u/( REFERENCE_VOLTAGE * SIN_OUPUT_MAX / SCALE );
    phase_pwm_v = phase_pwm_v/( REFERENCE_VOLTAGE * SIN_OUPUT_MAX / SCALE );
    phase_pwm_w = phase_pwm_w/( REFERENCE_VOLTAGE * SIN_OUPUT_MAX / SCALE );

    // We center the sinusoide to have phase from 0V to REFERENCE_VOLTAGE
            
    ///////////////////////////////////////////////////////////////////////////
    // Set PWM 
    ///////////////////////////////////////////////////////////////////////////
    
    if (!motor_on) {
        disable_all_motors();
    } else {
        // We always want to have a phase on the mass, so we control, first, 
        // the phase going to the mass.
        if( phase_pwm_u==0 ){
            apply_pwm(U_SD_PIN, U_IN_PIN, phase_pwm_u);
            apply_pwm(V_SD_PIN, V_IN_PIN, phase_pwm_v);
            apply_pwm(W_SD_PIN, W_IN_PIN, phase_pwm_w);
        } else if( phase_pwm_v==0 ){
            apply_pwm(V_SD_PIN, V_IN_PIN, phase_pwm_v);
            apply_pwm(W_SD_PIN, W_IN_PIN, phase_pwm_w);
            apply_pwm(U_SD_PIN, U_IN_PIN, phase_pwm_u);
        } else if( phase_pwm_w==0 ){
            apply_pwm(W_SD_PIN, W_IN_PIN, phase_pwm_w);
            apply_pwm(U_SD_PIN, U_IN_PIN, phase_pwm_u);
            apply_pwm(V_SD_PIN, V_IN_PIN, phase_pwm_v);
        } else {
            security_set_error(SECURITY_NO_PHASE_IS_ON_THE_MASS);
            disable_all_motors();
        }
    }
}


int rotor_angle(){
    return encoder_to_int() - angle_origin;
}

int nb_pass = 0;

#define RESOLUTION 1024 // Should be a power of two lesser thant ONE_TURN_THETA 
static_assert( RESOLUTION < ONE_TURN_THETA, "");

static int tare_cnt;
static int all_angle[RESOLUTION];
static int precision = 4;
static int nb_turn = 0;

//#define FULL_TARE_PROCESS

int tare_process(){
    int theta = 0;
    switch( tare_state ){
        case TARE_NOT_SET:
            break;
        case TODO_TARE:
            theta = 0;
            motor_on = true,
            last_tare_time = millis();
            direct_voltage_c = HALF_REFERENCE_VOLTAGE/2; 
            quadrature_voltage_c = 0;
            save_vectorial_pwm = vectorial_pwm;
            vectorial_pwm = CONFIG_PWM;
            tare_is_set = true;
            tare_state = DEFINE_ORIGIN;
            break;
        case DEFINE_ORIGIN:
            theta = 0;
            if( millis() - last_tare_time > 1000 ){
                angle_origin = encoder_to_int();
                #ifdef FULL_TARE_PROCESS 
                tare_state = TARE_ANGLE;
                #else
                tare_state = TARE_IS_DONE;
                #endif
                for( tare_cnt = 0; tare_cnt<RESOLUTION; tare_cnt ++ ){
                    all_angle[tare_cnt] = (ONE_TURN_THETA/RESOLUTION)*tare_cnt;
                }
            }
            break;
        case TARE_ANGLE:
            tare_cnt = 0;
            tare_state = COMPUTE_ANGLE;
            last_tare_time = millis();
            nb_turn = 0;
            nb_pass = 0;
            precision = 10;
        case COMPUTE_ANGLE:
            if( millis() - last_tare_time > 5 ){
                int mesure = rotor_angle();
                int error = (
                    (ONE_TURN_THETA/RESOLUTION)*tare_cnt + 
                    ONE_TURN_THETA*nb_turn 
                    - mesure
                );
                if( 2*error < ONE_TURN_THETA/RESOLUTION ){
                    tare_cnt ++;
                }else{
                    all_angle[tare_cnt] = all_angle[tare_cnt] + error/precision;
                }
                last_tare_time = millis();
                if( tare_cnt == RESOLUTION ){
                    tare_cnt = 0;
                    nb_turn ++;
                    nb_pass ++ ;
                    switch( nb_pass ){
                        case 1:
                            precision = 10;
                            break;
//                        case 2:
//                            precision = 8;
//                            break;
//                        case 3:
//                            precision = 8;
//                            break;
                        default:
                            tare_state = TARE_IS_DONE;
                    }
                }
            }
            theta = all_angle[tare_cnt%RESOLUTION];
            break;
        case TARE_IS_DONE:
            theta = rotor_angle(); 
            terminal_io()->println("Tare is done.");
            vectorial_pwm = save_vectorial_pwm;
            direct_voltage_c = 0;
            quadrature_voltage_c = 0;
            tare_is_set = false;
            break;
        default:;
    }
    return theta;
}

    
void dispatch_display();

int filter(int angle){
    if(tare_is_set) return angle;
    return all_angle[mod(angle, ONE_TURN_THETA) / (ONE_TURN_THETA/RESOLUTION)];
}

void motor_foc_tick()
{
    static bool motor_ticking = false;

    if (motor_ticking) {
        return;
    }
    motor_ticking = true;

    if( !motor_flag ){
        motor_ticking = false;
        return;
    }
    motor_flag = false;
    
    display_warning();
    //dispatch_display();

    // display(false);
    //D/ int theta;
    if( tare_is_set ){
        theta_s = tare_process();
    }else{
        theta_s = rotor_angle();
    }
    if( tare_state == TARE_IS_DONE || tare_is_set ){
        if( go_theta ){
            //display(false);
            #ifdef FULL_TARE_PROCESS
            control_motor_with_vectorial(filter(theta_c));
            #else
            control_motor_with_vectorial(theta_c);
            #endif
        }else{
            //display(false);
            #ifdef FULL_TARE_PROCESS
            control_motor_with_vectorial(filter(theta_s));
            #else
            control_motor_with_vectorial(theta_s);
            #endif
        }
    }else{
        disable_all_motors();
    }
    
    motor_ticking = false;
}

void reset_vectorial(){
    go_theta = false;
}

void start_to_tare_motor(){
    tare_state = TODO_TARE;
    tare_process();
}

TERMINAL_COMMAND(info_motor, "Info on motor")
{
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

static int last_warning = 0;

void display_warning(){
    int warning = security_get_warning();  
    int error = security_get_error();  
    if( warning != SECURITY_NO_WARNING || error != SECURITY_NO_ERROR ){
        int val = millis();
        if( val - last_warning > 4000 ){
            if( warning != SECURITY_NO_WARNING ){
                terminal_io()->print("W ");
                terminal_io()->println( driver_warning(warning) );
                security_set_warning( SECURITY_NO_WARNING );
            }
            if( error != SECURITY_NO_ERROR ){
                //terminal_io()->print("E ");
                terminal_io()->print(error);
                terminal_io()->println( driver_error(error) );
            }
            last_warning = val;
        }
    }
}


static unsigned int cnt = 0;
static int display_time = 0; 

void display_data(){
    terminal_io()->print("theta : ");
    terminal_io()->print( (1000.0 * theta_s)/ONE_TURN_THETA );
    terminal_io()->print(", theta park : ");
    terminal_io()->print( (1000.0 *theta_park)/ONE_TURN_THETA );
    terminal_io()->print(", t1 : ");
    terminal_io()->print( (1000.0 *t1) );
    terminal_io()->print(", t2 : ");
    terminal_io()->print( (1000.0 *t2) );
    terminal_io()->print(", t3 : ");
    terminal_io()->print( (1000.0 *t3) );
    terminal_io()->print(", c1 : ");
    terminal_io()->print((1000.0 * c1)/SIN_OUPUT_MAX );
    terminal_io()->print(", c2 : ");
    terminal_io()->print( (1000.0 *c2)/SIN_OUPUT_MAX );
    terminal_io()->print(", c3 : ");
    terminal_io()->println( (1000.0 *c3)/SIN_OUPUT_MAX );

    terminal_io()->print(", V u : ");
    terminal_io()->print( phase_voltage_u );
    terminal_io()->print(", V v : ");
    terminal_io()->print( phase_voltage_v );
    terminal_io()->print(", V w : ");
    terminal_io()->print( phase_voltage_w );

    terminal_io()->print(", pwm u : ");
    terminal_io()->print( phase_pwm_u );
    terminal_io()->print(", pmw v : ");
    terminal_io()->print( phase_pwm_v );
    terminal_io()->print(", pmw w : ");
    terminal_io()->print( phase_pwm_w );

    
    terminal_io()->print(", direct_v : ");
    terminal_io()->print( direct_voltage_c );
    terminal_io()->print(", quadrature_v : ");
    terminal_io()->print( quadrature_voltage_c );
    terminal_io()->println( "" );
}

void display( bool force ){
    cnt ++;
    int val = millis();
    //int warning = security_get_warning();
    //int error = security_get_error();
    if( force || val  -  display_time > 4000 ){
        int dt = val - display_time;
        terminal_io()->print("D(");
        terminal_io()->print(cnt);
        terminal_io()->print(" - ");
        terminal_io()->print( (cnt*1000)/dt );
        terminal_io()->print("Hz - ");
        terminal_io()->print( dt/1000 );
        terminal_io()->print("s");
        terminal_io()->print(") ");
        display_data();
        display_time = val;
        cnt = 0;
    }
}

static int dispatch_cnt = 0;
int last_dispatch = 0;
bool dispatch_display_b = false;

void dispatch_display(){
    if( ! dispatch_display_b ) return;
    int val = millis();
    if( val  -  last_dispatch > 1000 ){
        last_dispatch = val;
        int current = dispatch_cnt;
        for( ; dispatch_cnt < current + 20 and dispatch_cnt < RESOLUTION; dispatch_cnt++ ){
            terminal_io()->print("A ");
            terminal_io()->print( (1000.0*dispatch_cnt)/RESOLUTION );
            terminal_io()->print( " M " );
            terminal_io()->println( 1000.0* all_angle[dispatch_cnt] );
        }
        if( dispatch_cnt >= RESOLUTION ){
            dispatch_display_b = false;
        }
    }
}

TERMINAL_COMMAND(rot, "Rotor angle")
{
    terminal_io()->println( 1000.0* rotor_angle() );
}

TERMINAL_COMMAND(disp_f, "Display")
{
    dispatch_cnt = 0;
    dispatch_display_b = true;
}

TERMINAL_COMMAND(motor_info, "Display")
{
    terminal_io()->print("q v : ");
    terminal_io()->println(quadrature_voltage_c);
    terminal_io()->print("d v : ");
    terminal_io()->println(direct_voltage_c);
    terminal_io()->print("u pwm : ");
    terminal_io()->println(phase_pwm_u);
    terminal_io()->print("v pwm : ");
    terminal_io()->println(phase_pwm_v);
    terminal_io()->print("w pwm : ");
    terminal_io()->println(phase_pwm_w);
    terminal_io()->print("theta park : ");
    terminal_io()->println(theta_park);
    terminal_io()->print("theta park mod 3: ");
    terminal_io()->println(mod( theta_park, ONE_TURN_THETA/3 ));
}

TERMINAL_COMMAND(disp, "Display")
{
    display( true );
}

bool motor_is_tared(){
    return tare_state == TARE_IS_DONE;
}

bool motor_foc_is_on(){
    return motor_on;
}

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

// #define TEST_LED_FOC

static bool enable_foc = true;

inline int mod(int n, int d){
    int r = n%d;
    return (r>=0) ? r : r+d;
}

// Target PWM speed [0-3000]
static bool motor_on = false;

static bool motor_flag = false;
static bool serv_flag = false;

void reset_serv_flag(){
    serv_flag = false;
}

bool get_serv_flag(){
    return serv_flag;
}

#ifdef PHASE_OPPOSITION 
static int sd_pin_1;
static int in_pin_1;
static int pwm_1;

static int sd_pin_2;
static int in_pin_2;
static int pwm_2;

static int sd_pin_low;
static int in_pin_low;

static int lock=true;
#endif

inline void set_to_high_impedance( int low_pin, int high_pin ){
    digitalWrite(low_pin, LOW);
    //pwmWrite(high_pin, 0);
}

inline void apply_pwm( int low_pin, int high_pin, int pwm ){
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

static unsigned int nb_motor_update_by_servo_update = 0;

void motor_irq(){
    static unsigned int count_motor_irq = 0;
    static unsigned int count_servo_irq = 0;
    static unsigned int count_irq_2 = 0;
    #ifdef TEST_LED_FOC
    if( count_irq_2 % 2 ){
      digitalWrite(LED_PIN, HIGH);
    }else{
      digitalWrite(LED_PIN, LOW);
    }
    #endif
    count_motor_irq++;
    count_servo_irq++;
    count_irq_2++;
    if(count_motor_irq == MOTOR_UPDATE){
        count_motor_irq = 0;
        nb_motor_update_by_servo_update++;
    }
    if(count_servo_irq == SERVO_UPDATE){
        count_servo_irq = 0;
        nb_motor_update_by_servo_update = 0;
    }
    if( ! count_servo_irq ){
        if( serv_flag ){
            security_set_warning(WARNING_SERVO_LAG);
        }
        serv_flag = true;
    }
    if( ! count_motor_irq ){
        if( motor_flag ){
            security_set_warning(WARNING_MOTOR_LAG);
        }
        motor_flag = true;
    }
    #ifdef HIGH_IMPEDENCE_MODE
      #define PWM_SHIFT_PERCENT 0
    #else
      #define PWM_SHIFT_PERCENT 15 
      //#define PWM_SHIFT_PERCENT 0 
    #endif
    #define PWM_SHIFT  ((PWM_SHIFT_PERCENT*PWM_SUPREMUM)/100)
    #ifdef PHASE_OPPOSITION 
        if(count_irq_2%SWAP_PWM_FREQUENCE == 0){
            if(motor_on){
                if( (count_irq_2/SWAP_PWM_FREQUENCE)%2 == 0 ){
                    if( !lock ){
                        apply_pwm(sd_pin_low, in_pin_low, PWM_SHIFT);
                        #ifndef HIGH_IMPEDENCE_MODE
                            apply_pwm(sd_pin_1, in_pin_1, PWM_SHIFT);
                        #else
                            set_to_high_impedance(sd_pin_1, in_pin_1);
                        #endif
                        apply_pwm(sd_pin_2, in_pin_2, pwm_2);
                    }
                }else{
                    if( !lock ){
                        apply_pwm(sd_pin_low, in_pin_low, PWM_SHIFT);
                        #ifndef HIGH_IMPEDENCE_MODE
                            apply_pwm(sd_pin_2, in_pin_2, PWM_SHIFT);
                        #else
                            set_to_high_impedance(sd_pin_2, in_pin_2);
                        #endif
                        apply_pwm(sd_pin_1, in_pin_1, pwm_1);
                    }
                }
            }
        }
    #endif
}

void disable_all_motors(){
    pwmWrite(U_IN_PIN, 0);
    pwmWrite(V_IN_PIN, 0);
    pwmWrite(W_IN_PIN, 0);
    
    digitalWrite(U_SD_PIN, LOW);
    digitalWrite(V_SD_PIN, LOW);
    digitalWrite(W_SD_PIN, LOW);
} 

inline void set_pwm(int phase_pwm_u, int phase_pwm_v, int phase_pwm_w){ 
    // We always want to have a phase on the mass, so we control, first, 
    // the phase going to the mass.
    #define PWM_MAX_FOR_CENTER_ALIGNED_PWM 434 // Max value of the pwm for a 
        // phase.
        // This value is computed to avoid that high levels of two different
        // phases occur at the same time (the two pwms should be in
        // phase opposition by using the center-alingned mode of the STM32FXXX).
        // The complete calculus of this value is given in the comments of
        // the function : void control_motor_with_vectorial( int theta ).
        // This value should be round up because, it is used in the denominator
        // for the calculus of the final pwm. And that value have to be lesser 
        // than PWM_MAX.
    static_assert(
        PWM_MAX_FOR_CENTER_ALIGNED_PWM * PWM_MAX_FOR_CENTER_ALIGNED_PWM >= 
        PWM_SUPREMUM*PWM_SUPREMUM/3, ""
    );
    static_assert(
        (PWM_MAX_FOR_CENTER_ALIGNED_PWM-1)*(PWM_MAX_FOR_CENTER_ALIGNED_PWM-1)
        < PWM_SUPREMUM*PWM_SUPREMUM/3, "" 
    );

    #ifdef PHASE_OPPOSITION
        // TODO : Replace the following code by a code that use the center-aligned
        // mode of the STM32FXXX . We can improve theneed to be replaced by using the mod
        if( phase_pwm_u==0 ){
            lock = true;
            sd_pin_low = U_SD_PIN;
            in_pin_low = U_IN_PIN;

            sd_pin_1 = V_SD_PIN;
            in_pin_1 = V_IN_PIN;
            pwm_1 = (phase_pwm_v*(PWM_MAX-PWM_SHIFT))/PWM_MAX_FOR_CENTER_ALIGNED_PWM;

            sd_pin_2 = W_SD_PIN;
            in_pin_2 = W_IN_PIN;
            pwm_2 = (phase_pwm_w*(PWM_MAX-PWM_SHIFT))/PWM_MAX_FOR_CENTER_ALIGNED_PWM;
            lock = false;
        } else if( phase_pwm_v==0 ){
            lock = true;
            sd_pin_low = V_SD_PIN;
            in_pin_low = V_IN_PIN;

            sd_pin_1 = U_SD_PIN;
            in_pin_1 = U_IN_PIN;
            pwm_1 = (phase_pwm_u*(PWM_MAX-PWM_SHIFT))/PWM_MAX_FOR_CENTER_ALIGNED_PWM;

            sd_pin_2 = W_SD_PIN;
            in_pin_2 = W_IN_PIN;
            pwm_2 = (phase_pwm_w*(PWM_MAX-PWM_SHIFT))/PWM_MAX_FOR_CENTER_ALIGNED_PWM;
            lock = false;
        } else if( phase_pwm_w==0 ){
            lock = true;
            sd_pin_low = W_SD_PIN;
            in_pin_low = W_IN_PIN;
            
            sd_pin_1 = U_SD_PIN;
            in_pin_1 = U_IN_PIN;
            pwm_1 = (phase_pwm_u*(PWM_MAX-PWM_SHIFT))/PWM_MAX_FOR_CENTER_ALIGNED_PWM;

            sd_pin_2 = V_SD_PIN;
            in_pin_2 = V_IN_PIN;
            pwm_2 = (phase_pwm_v*(PWM_MAX-PWM_SHIFT))/PWM_MAX_FOR_CENTER_ALIGNED_PWM;
            lock = false;
        } else {
            security_set_error(SECURITY_NO_PHASE_IS_ON_THE_MASS);
            motor_on=false;
            disable_all_motors();
        }
    #else
        if( phase_pwm_u==0 ){
            apply_pwm(U_SD_PIN, U_IN_PIN, phase_pwm_u+PWM_SHIFT);
            apply_pwm(V_SD_PIN, V_IN_PIN, PWM_SHIFT+(phase_pwm_v*(PWM_MAX-PWM_SHIFT))/PWM_MAX_FOR_CENTER_ALIGNED_PWM);
            apply_pwm(W_SD_PIN, W_IN_PIN, PWM_SHIFT+(phase_pwm_w*(PWM_MAX-PWM_SHIFT))/PWM_MAX_FOR_CENTER_ALIGNED_PWM);
        } else if( phase_pwm_v==0 ){
            apply_pwm(V_SD_PIN, V_IN_PIN, phase_pwm_v+PWM_SHIFT);
            apply_pwm(W_SD_PIN, W_IN_PIN, PWM_SHIFT+(phase_pwm_w*(PWM_MAX-PWM_SHIFT))/PWM_MAX_FOR_CENTER_ALIGNED_PWM);
            apply_pwm(U_SD_PIN, U_IN_PIN, PWM_SHIFT+(phase_pwm_u*(PWM_MAX-PWM_SHIFT))/PWM_MAX_FOR_CENTER_ALIGNED_PWM);
        } else if( phase_pwm_w==0 ){
            apply_pwm(W_SD_PIN, W_IN_PIN, phase_pwm_w+PWM_SHIFT);
            apply_pwm(U_SD_PIN, U_IN_PIN, PWM_SHIFT+(phase_pwm_u*(PWM_MAX-PWM_SHIFT))/PWM_MAX_FOR_CENTER_ALIGNED_PWM);
            apply_pwm(V_SD_PIN, V_IN_PIN, PWM_SHIFT+(phase_pwm_v*(PWM_MAX-PWM_SHIFT))/PWM_MAX_FOR_CENTER_ALIGNED_PWM);
        } else {
            security_set_error(SECURITY_NO_PHASE_IS_ON_THE_MASS);
            disable_all_motors();
        }
    #endif
}

static void _init_timer(int number)
{
    HardwareTimer timer(number);

    // Configuring timer
    timer.pause();
    timer.setPrescaleFactor(PWM_PRESCALE_FACTOR);
    timer.setOverflow(PWM_OVERFLOW);
    timer.refresh();

    #if BOARD == GREG
    if (number == 3) {
        timer.setMode(TIMER_CH2, TIMER_OUTPUT_COMPARE);
        timer.setCompare(TIMER_CH2, 1);
        timer.attachInterrupt(TIMER_CH2, motor_irq);
    }
    #endif
    #if BOARD == CATIE
    if (number == 1) {
        timer.setMode(TIMER_CH4, TIMER_OUTPUT_COMPARE);
        timer.setCompare(TIMER_CH4, 1);
        timer.attachInterrupt(TIMER_CH4, motor_irq);
    }
    #endif
    timer.refresh();
    timer.resume();
}

void motor_foc_init()
{

    #ifdef TEST_LED_FOC
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    #endif

    // Initalizing motor pins
    pwmWrite(U_IN_PIN, 0);
    digitalWrite(U_IN_PIN, LOW);
    pwmWrite(U_IN_PIN, 0);
    digitalWrite(U_IN_PIN, LOW);
    pwmWrite(V_IN_PIN, 0);
    digitalWrite(V_IN_PIN, LOW);
    pwmWrite(W_IN_PIN, 0);
    digitalWrite(W_IN_PIN, LOW);
    pwmWrite(U_SD_PIN, 0);
    digitalWrite(U_SD_PIN, LOW);
    pwmWrite(V_SD_PIN, 0);
    digitalWrite(V_SD_PIN, LOW);
    pwmWrite(W_SD_PIN, 0);
    digitalWrite(W_SD_PIN, LOW);

    pinMode(U_SD_PIN, OUTPUT);
    pinMode(V_SD_PIN, OUTPUT);
    pinMode(W_SD_PIN, OUTPUT);

    pinMode(U_IN_PIN, PWM);
    pinMode(V_IN_PIN, PWM);
    pinMode(W_IN_PIN, PWM);
    
    // Configuring timers
    #if BOARD == GREG
    _init_timer(2);
    _init_timer(3);
    #endif


    #if BOARD == CATIE
    _init_timer(1);
    #endif
    
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

inline int32_t compute_maximal_voltage_2(int32_t vd, int32_t vq){
    return vd * vd + vq * vq;
}

void direct_quadrature_voltage_set(int vd, int vq ){
    int max_voltage_2 = compute_maximal_voltage_2(vd, vq);
    direct_voltage_c = vd;
    quadrature_voltage_c = vq;
    if( max_voltage_2 > REFERENCE_VOLTAGE * REFERENCE_VOLTAGE ){
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
static bool use_fixed_theta = false;

void set_fixed_theta(bool value){
    use_fixed_theta = value;
}

void reset_vectorial();

TERMINAL_COMMAND(go_theta, "Set theta")
{
    if (argc == 1) {
        reset_vectorial();
        float val = atof(argv[0]);
        theta_c = (int) ( val * ONE_TURN_THETA );
        direct_quadrature_voltage_set( HALF_REFERENCE_VOLTAGE, 0);
        use_fixed_theta = true;
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

// begin of automatic code - generated by motor_foc.py
static_assert(
    PWM_SUPREMUM==750, 
    "You need to regenerate the code with motor_foc.py and change PWM_SUP value" 
);

#define PHASE_PWM_U_SCALE 1
#define PHASE_PWM_V_SCALE 1
#define PHASE_PWM_W_SCALE 1
#define ALPHA_USER_PWM_SCALE 8589934592
#define ALPHA_PWM_SCALE 2147483648
#define USER_PWM_SCALE 16777216
#define PHASE_VOLTAGE_U_SCALE 8192
#define MIN_VOLTAGE_SCALE 8192
#define PHASE_VOLTAGE_V_SCALE 8192
#define PHASE_VOLTAGE_W_SCALE 8192
static int user_pwm = CONFIG_PWM*USER_PWM_SCALE;
static int phase_pwm_w = 0;
static int phase_pwm_u = 0;
static int phase_pwm_v = 0;
static int phase_voltage_u = 0;
static int phase_voltage_v = 0;
static int phase_voltage_w = 0;

// end of automatic code - generated by motor_foc.py


static int angle_origin = 0;
static int save_user_pwm = CONFIG_PWM*USER_PWM_SCALE;

/*
 * Compute vectorial command
 *
 * theta : angle of the rotor in turn
 *   VALUE : [INT_MIN/ONE_TURN_THETA, INT_MAX/ONE_TURN_THETA]
 *   SCALE : ONE_TURN_THETA
 */
inline void compute_vectorial_command(
    const int theta , 
    int & phase_voltage_u, int & phase_voltage_v, int & phase_voltage_w,
    int & phase_pwm_u, int & phase_pwm_v, int & phase_pwm_w 
){
    ///////////////////////////////////////////////////////////////////////////
    // Compute phase voltage
    ///////////////////////////////////////////////////////////////////////////
    // ( u )                ( c1 -s1 )
    // ( v ) = sqrt(2/3) .  ( c2 -s2 ) . ( vd )
    // ( w )                ( c3 -s3 )   ( vq )
    //
    // c1 = cos( theta )           s1 = cos( theta ) 
    // c2 = cos( theta - 2pi/3 )   s2 = cos( theta - 2pi/3 ) 
    // c3 = cos( theta + 2pi/3 )   s3 = cos( theta + 2pi/3 ) 
    //
    // We want to compute the pwm of the phase u, v, w. 
    //
    // Suppose now that u >= v >= w, then we will set the following pwm on each 
    // phase : 
    // 
    // pwm_u = alpha * (u-w)        (that is always >=0)
    // pwm_v = alpha * (v-w)        (that is always >=0)
    // pwm_w = 0
    //
    // we need to determine alpha;
    //
    // Determination of alpha 
    // -----------------------
    //
    // We need to determine alpha. Alpha should verify the following constraint : 
    //  - to avoid ripple, pwm_u and pwm_v should not overlap, that means
    //    pwm_u + pwm_v <= 1
    //  - if possible, the system can use the maximal power avalaible, that means,
    //    we need to reach pwm_u + pwm_v = 1 when needed.
    //
    // pwm_u + pwm_v = alpha * ( u + v - 2*w ) = alpha * (u + v + w - 3*w) 
    //               = - 3 * alpha * w
    // ( since u + v + w = 0 ).
    // this formula says that, when pwm_w = 0, then, w < 0.
    //
    // We need to determine the maximal value of the absolute value of w : 
    // w = k * (c3 * direct_voltage_c - s3 * quadrature_voltage_c)
    // 
    // We will set a = direct_voltage_c and b = quadrature_voltage_c and we have
    // w = k*(c3 * a - s3 * b)
    // 
    // w will be maximal when
    // derivate(w) = -k*(s3 * a + c3 * b) = 0
    // so
    // s3 * a^2/b + c3 * a = 0      and      s3 * b + c3 * b^2/a = 0
    // and so 
    // w = -k*(a^2/b + b)*s3        and      w = k*(a + b^2/a)*c3
    // and so
    // 1 = s3^2 + c3^2 = w^2 / ( k^2*(a^2 + b^2) )
    // We deduce that
    //
    // w_max = k * sqrt( a^2 + b^2 )
    // This value is reached when arctan(-b/3) = theta_max, since 
    // s3 * a + c3 * b = 0.
    // 
    // We deduce that : 
    // pwm_u + pwm_v <= 3 * alpha * k * sqrt( a^2 + b^2 )
    // that limit is reached ans we want to reach that limit, so
    // 1 = 3 * alpha * k * sqrt( a^2 + b^2 )
    //
    // We deduce finally that 
    // alpha = 1/( 3 * k * V_ref )
    //
    // where V_ref is the maximal voltage in such a way that
    // V_ref = max( sqrt(a^2 + b^2) ) for any control (a,b) computed by the 
    // asservissement layer.
    //
    //
    // We check now that max( pwm_u ) < 1 
    // ----------------------------------
    //
    // max( pwm_u ) = alpha * max( u-v )
    //
    // We need now to evaluate the maximum value
    // of the difference between two phases voltages when theta is going from 
    // 0 to 2 pi.
    // 
    // u - v = k * ( (c1-c2) * a - (s1-s2) * b );
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
    // 3 = (a^2 + b^2) * J^2 / (a^2 + b^2)^2
    // 
    // J = sqrt(3) * sqrt( a^2 + b^2 ) 
    //
    // So the maximal difference voltage is equal to 
    // max( u - v ) = k * sqrt(3) * sqrt( a^2 + b^2 )
    //
    // We deduce finally that
    // max( pwm_u ) = alpha * k * sqrt(3) * sqrt( a^2 + b^2 )
    //
    // When the control (a,b) reach the maximum value V_ref = max( sqrt(a^2+b^2) )
    // then 
    // max( pwm_u ) = sqrt(3)/3 = 0.58
    // 
    // The maximal pwm is lesser that 1 :) and the maximal pwm is biger that 0.5,
    // that is compatible with the fact that pmw_u + pwm_v = 1 for some angular
    // position of the rotor (and that means that when pwm_u or pwm_v is maximal
    // then pwm_u + pwm_v is strictly lesser than 1). 
    // 
    //
    // CONCLUSION :
    // ------------ 
    //
    // When u,v >= w, then, the phase pwms are equals to 
    //
    // pwm_u = ( u-w )/( 3 * k * V_ref )
    // pwm_v = ( v-w )/( 3 * k * V_ref )
    // pwm_w = 0
    //
    // where the control layer computes   
    // a = direct_voltage_c        and       b = quadrature_voltage_c.
    //
    // with the constraint that
    // sqrt(a^2 + b^2) <= V_ref,
    //
    // with
    // u = k*(c1*a -s1*b),   v = k*(c1*a -s1*b)   and     w= k*(c1*a -s1*b)
    //
    // with k = sqrt(2/3).
    //
    //
    // by using the direct and quadrature voltage we obtain 
    //
    // pwm_u = ( (c1-c3) * a - (s1-s3) * b )/( 3 * V_ref )
    // pwm_v = ( (c2-c3) * a - (s2-s3) * b )/( 3 * V_ref )
    // pwm_w = 0
    //
    // pwm_u and pwm_v should be in phase opposition ! 
    // (by using the center-aligned feature of stm32F)
    //
    ///////////////////////////////////////////////////////////////////////////

    // VALUE : [0, NB_POSITIVE_MAGNETS]
    // SCALE : ONE_TURN_THETA
    int theta_park = NB_POSITIVE_MAGNETS * mod( theta, ONE_TURN_THETA ); //D/

    // VALUE : [0, NB_POSITIVE_MAGNETS]
    // SCALE : ONE_TURN_THETA
    int16_t t1 = theta_park; //D/
    // VALUE : [-1/3, NB_POSITIVE_MAGNETS-1/3]
    // SCALE : ONE_TURN_THETA
    int16_t t2 = theta_park - ONE_TURN_THETA/3; //D/
    // VALUE : [+1/3, NB_POSITIVE_MAGNETS+1/3]
    // SCALE : ONE_TURN_THETA
    int16_t t3 = theta_park + ONE_TURN_THETA/3; //D/

    // VALUE : [0, 1]
    // SCALE : SIN_INPUT_RESOLUTION
    static_assert( SIN_INPUT_RESOLUTION <=  ONE_TURN_THETA , "");
    t1 = mod(t1, ONE_TURN_THETA)/(ONE_TURN_THETA/SIN_INPUT_RESOLUTION);
    t2 = mod(t2, ONE_TURN_THETA)/(ONE_TURN_THETA/SIN_INPUT_RESOLUTION);
    t3 = mod(t3, ONE_TURN_THETA)/(ONE_TURN_THETA/SIN_INPUT_RESOLUTION);
    
    // VALUE : [-1, 1]
    // SCALE : SIN_OUPUT_MAX
    const int c1 = discrete_cos(t1);
    const int s1 = discrete_sin(t1);
    const int c2 = discrete_cos(t2);
    const int s2 = discrete_sin(t2);
    const int c3 = discrete_cos(t3);
    const int s3 = discrete_sin(t3);

    int alpha_user_pwm;
    int alpha_pwm;
    int min_voltage;

    // begin of automatic code - generated by motor_foc.py
    alpha_pwm = (5235889);
    alpha_user_pwm = ((alpha_pwm/128)*(user_pwm/32768));
    phase_voltage_u = (((c1/1)*(direct_voltage_c/1))/1 + (-((s1/1)*(quadrature_voltage_c/1)))/1);
    phase_voltage_v = (((c2/1)*(direct_voltage_c/1))/1 + (-((s2/1)*(quadrature_voltage_c/1)))/1);
    phase_voltage_w = (((c3/1)*(direct_voltage_c/1))/1 + (-((s3/1)*(quadrature_voltage_c/1)))/1);
    min_voltage = ( (( (phase_voltage_u*1 < phase_voltage_v*1) ? phase_voltage_u*1 : phase_voltage_v*1 )*1 < phase_voltage_w*1) ? ( (phase_voltage_u*1 < phase_voltage_v*1) ? phase_voltage_u*1 : phase_voltage_v*1 )*1 : phase_voltage_w*1 );
    phase_pwm_u = (((alpha_user_pwm/65536)*((phase_voltage_u/1 + (-min_voltage)/1)/256))/4194304);
    phase_pwm_v = (((alpha_user_pwm/65536)*((phase_voltage_v/1 + (-min_voltage)/1)/256))/4194304);
    phase_pwm_w = (((alpha_user_pwm/65536)*((phase_voltage_w/1 + (-min_voltage)/1)/256))/4194304);
    // end of automatic code - generated by motor_foc.py
}

/*
 * Apply control command on motors.
 *
 * theta : angle of the rotor in turn
 *   VALUE : [INT_MIN/ONE_TURN_THETA, INT_MAX/ONE_TURN_THETA]
 *   SCALE : ONE_TURN_THETA
 */
inline void control_motor_with_vectorial( const int theta ){
    compute_vectorial_command(
        theta ,
        phase_voltage_u, phase_voltage_v, phase_voltage_w,
        phase_pwm_u, phase_pwm_v, phase_pwm_w 
    );
            
    ///////////////////////////////////////////////////////////////////////////
    // Set PWM 
    ///////////////////////////////////////////////////////////////////////////
    if (!motor_on) {
        disable_all_motors();
    } else {
        set_pwm(phase_pwm_u, phase_pwm_v, phase_pwm_w);
    }
}


inline int rotor_angle(){
    return encoder_position() - angle_origin;
}

int nb_pass = 0;

#define RESOLUTION 1024 // Should be a power of two lesser thant ONE_TURN_THETA 
static_assert( RESOLUTION < ONE_TURN_THETA, "");

static int tare_cnt;
static int all_angle[RESOLUTION];
static int precision = 4;
static int nb_turn = 0;

TERMINAL_PARAMETER_FLOAT(REF, "PID P", 0.0);


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

            if( CONFIG_PWM > 50){
              direct_quadrature_voltage_set(REFERENCE_VOLTAGE/2, 0);
            }else{
              direct_quadrature_voltage_set(REFERENCE_VOLTAGE, 0);
            }

            //direct_quadrature_voltage_set(REF, 0);
            save_user_pwm = user_pwm;
            user_pwm = CONFIG_PWM*USER_PWM_SCALE;
            tare_is_set = true;
            tare_state = DEFINE_ORIGIN;
            break;
        case DEFINE_ORIGIN:
            theta = 0;
            if( millis() - last_tare_time > 1000 ){
                angle_origin = encoder_position();
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
            //terminal_io()->println("Tare is done.");
            user_pwm = save_user_pwm;
            direct_quadrature_voltage_set(0, 0);
            tare_is_set = false;
            break;
        default:;
    }
    return theta;
}

    
int filter(int angle){
    if(tare_is_set) return angle;
    return all_angle[mod(angle, ONE_TURN_THETA) / (ONE_TURN_THETA/RESOLUTION)];
}
int default_update_theta(int angle){
    return angle;
}

static int (* update_theta)(int angle) = default_update_theta;

void register_update_theta( int (* fct)(int) ){
    update_theta = fct;
};

TERMINAL_PARAMETER_INT(deph, "dephasage", 0);

void motor_foc_tick()
{
    // TODO : RETIRER motor_ticking : INUTILE
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
    
    // VALUE : [INT_MIN/ONE_TURN_THETA, INT_MAX/ONE_TURN_THETA]
    // SCALE :  ONE_TURN_THETA 
    static int theta_s = 0;
    static int speed_s = 0;
    if( tare_is_set ){
        theta_s = tare_process();
    }else{
        #define SCALED_FREQ ((ENCODER_SPEED_SCALE/THETA_OUT_SCALE)*MOTOR_UPDATE_FREQUENCE)
        if( nb_motor_update_by_servo_update == 0 ){
            speed_s = encoder_to_speed();
            // TODO : IMPROVE encoder_to_speed() !
            //if( abs(speed_s)<(ENCODER_SPEED_SCALE*5/10) ){
            //  speed_s /= 2;
            //}
            // TODO
            const int dephasage_for_delay = 0;
            //const int dephasage_for_delay = (
            //   deph*(speed_s/(ENCODER_SPEED_SCALE/THETA_OUT_SCALE))
            //)/1000000;
            theta_s = dephasage_for_delay + rotor_angle() + (speed_s/(2*SCALED_FREQ));
        }else{
            theta_s += nb_motor_update_by_servo_update * (speed_s/SCALED_FREQ);
        }
    }
    if( tare_state == TARE_IS_DONE || tare_is_set ){
        if( ! tare_is_set and use_fixed_theta ){
            theta_c = update_theta(theta_s);
            #ifdef FULL_TARE_PROCESS
            control_motor_with_vectorial(filter(theta_c));
            #else
            control_motor_with_vectorial(theta_c);
            #endif
        }else{
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
    use_fixed_theta = false;
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
    terminal_io()->print("user pwm : ");
    terminal_io()->println(((float)user_pwm)/USER_PWM_SCALE);
    terminal_io()->print("theta : ");
    terminal_io()->println(rotor_angle());
    terminal_io()->print("phase_voltage_u : ");
    terminal_io()->println(phase_voltage_u);
    terminal_io()->print("phase_voltage_v : ");
    terminal_io()->println(phase_voltage_v);
    terminal_io()->print("phase_voltage_w : ");
    terminal_io()->println(phase_voltage_w);
    terminal_io()->print("phase_pwm_u : ");
    terminal_io()->println(((float)phase_pwm_u)/PHASE_PWM_U_SCALE);
    terminal_io()->print("phase_pwm_v : ");
    terminal_io()->println(((float)phase_pwm_v)/PHASE_PWM_V_SCALE);
    terminal_io()->print("phase_pwm_w : ");
    terminal_io()->println(((float)phase_pwm_w)/PHASE_PWM_W_SCALE);

    terminal_io()->print("motor on : ");
    terminal_io()->println(motor_on);
}


TERMINAL_COMMAND(tare, "Tare origin")
{
    tare_is_set = false;
    start_to_tare_motor();
}

void launch_tare_motor_foc(){
    tare_is_set = false;
    start_to_tare_motor();
}

TERMINAL_COMMAND(user_pwm, "Force the phase to use a given pwm")
{
    if (argc == 1) {
        // begin of automatic code - generated by motor_foc.py
        float user_pwm_input = atof(argv[0]);
        if(user_pwm_input < -100) user_pwm_input = -100;
        if(user_pwm_input > 100) user_pwm_input = 100;
        user_pwm = (int) (user_pwm_input * USER_PWM_SCALE);
        // begin of automatic code - generated by motor_foc.py
    } else {
        if(argc==0){
            terminal_io()->println(((double)user_pwm)/USER_PWM_SCALE);
            return;
        }
        terminal_io()->println("Usage: user_pwm [ -100.0 - 100 ]");
    }
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

#ifdef PHASE_OPPOSITION 
    terminal_io()->print("pwm1 : ");
    terminal_io()->println(pwm_1);
    terminal_io()->print("pwm2 : ");
    terminal_io()->println(pwm_2);
#endif

    terminal_io()->print("user pwm : ");
    terminal_io()->println(user_pwm);

}

bool motor_is_tared(){
    return tare_state == TARE_IS_DONE and ! tare_is_set;
}

bool motor_foc_is_on(){
    return motor_on;
}

void enable_motor_foc(bool value){
    enable_foc = value;
}

TERMINAL_COMMAND(enable_foc, "Enable foc"){
    if (argc == 1) {
        int val = atoi(argv[0]);
        if( val == 0 ){
            enable_motor_foc(false);
        }else{
            enable_motor_foc(true);
        }
    }
}


TERMINAL_COMMAND(FORCEREFLASH, "Force the phase to use a given pwm")
{
  terminal_io()->println("FORCE REFLASH");
}

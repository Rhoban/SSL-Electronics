#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include "hardware.h"
#include "motor.h"
#include "current.h"
#include "servo.h"
#include "encoder.h"
#include "security.h"


// Interrupt flag
static bool servo_flag = false;
float curr;
// Enable and target
static bool servo_enable = false;
static float servo_target = 0;
static int16_t servo_prior_pwm = 0;
static float servo_limited_target = 0;

// Speed estimation
static float servo_speed = 0;
static volatile float servo_public_speed = 0;
static volatile int16_t servo_public_pwm = 0;

// Number of values stored in the speed ring buffer
#define SPEED_RB        (((SPEED_DT)/SERVO_DT)+1)
static int encoder_rb[SPEED_RB] = {0};
static int encoder_pos = 0;
////////////////////////////////////////////////////////////////////////////////:
#define VMAX 20
#define NBR_COEF 4
#define ART_LIM  2500
#define PLOT 2
#define PLOT_CURR 0
#define STEP 1
#define STEP1 1.5
#define STEP2 2.5
#define STEP3 0
#define RAMP 0

#define ASSERV_FPI 1
#define ASSERV_FPI2 0
#define BO 0
#define ASSERV_TEST 0

fifo::fifo(){

}

fifo::fifo(int _length){
  data = (double*)malloc(_length*sizeof(double));
  length = _length;
  init();
}

fifo::~fifo(){
  free(data);
}

void fifo::pop(){
  for(int i=length-1; i>0; i--){
    data[i] = data[i-1];
  }
}

void fifo::top(double _newValue){
  pop();
  data[0] = _newValue;
}

int fifo::get_Length(){
  return length;
}

double fifo::get_Value(int _pos){
  if((_pos > length - 1) | (_pos < 0)){
    terminal_io()->println("Fifo : Index out of range");
    return 0;
  }
  else
    return data[_pos];
}

void fifo::init(){
  for(int i=0; i < length; i++){
    data[i] = 0;
  }
}
#if ASSERV_FPI == 1
static double coef_err[NBR_COEF] = {0.0512, -0.0502, 0, 0};//t  t-1   t-2  t-3
static double coef_cmd[NBR_COEF] = {1, 0, 0, 0}; //t-1 t-2 t-3 t-4
#endif

#if ASSERV_FPI2 == 1
static double coef_err[NBR_COEF] = {0.0524, -0.0514, 0, 0};//t  t-1   t-2  t-3
static double coef_cmd[NBR_COEF] = {1, 0, 0, 0}; //t-1 t-2 t-3 t-4
#endif

#if ASSERV_TEST == 1
static double coef_err[NBR_COEF] = {0.0254, -0.0246, 0, 0};//t  t-1   t-2  t-3
static double coef_cmd[NBR_COEF] = {1, 0, 0, 0}; //t-1 t-2 t-3 t-4
#endif

#if BO == 1
static double coef_err[NBR_COEF] = {1, 0, 0, 0};//t  t-1   t-2  t-3
static double coef_cmd[NBR_COEF] = {0, 0, 0, 0}; //t-1 t-2 t-3 t-4
#endif

fifo error(NBR_COEF);
fifo cmd(NBR_COEF);
static int cpt = 0;

/////////////////////////////////////////////////////////////////



TERMINAL_PARAMETER_BOOL(sdb, "Speed debug", false)
static int sdb_t = 0;

int timer = 0;
int aim = 0;

static void servo_irq()
{
    // XXX: This should probably be moved in encoder
    encoder_read();
}

void servo_set_flag()
{
    servo_flag = true;
}

void servo_init()
{
}

static int servo_pwm = 0;
static int servo_pwm_limited = 0;
static float servo_acc = 0;
static float servo_last_error = 0;
TERMINAL_PARAMETER_FLOAT(kp, "PID P", 10.0);
TERMINAL_PARAMETER_FLOAT(ki, "PID I", 0.5);
TERMINAL_PARAMETER_FLOAT(kd, "PID D", 0.5);

float servo_lut(float target, float current)
{
    // XXX: This is a simple LUT based on empirical measured
    // on motor behavior
    float sign = target > 0 ? 1 : -1;
    float boost = 0;

    if (fabs(target) > 0.1) {
        if (fabs(current) < 0.1) {
            // boost = 100;
        }
        return 16*target + sign*(60 + boost);
        // return 15*target + sign*(40 + boost);
    } else {
        return 0;
    }
}

#define MAX_SPEED 50.0  // turn / s
#define MAX_SPEED_STR "50.0"

static float max_speed = MAX_SPEED/5.0;
static float theta = 0.0; // Angular position
static float speed_consign = 0.0; // Angular position
bool origin_is_set = false;
static float max_theta = 0.0;
static float min_theta = 0.0;
        
static float derivative_position_error = 0.0;
static float theta_error = 0.0;
static float integral_position_error = 0.0;
static float angle_origin = 0.0;

TERMINAL_PARAMETER_FLOAT(pos_kp, "position PID P", 5.0);
TERMINAL_PARAMETER_FLOAT(pos_ki, "position PID I", 0.001);
TERMINAL_PARAMETER_FLOAT(pos_kd, "position PID d", 0.1);


void compute_rotor_position(){
    theta = ( encoder_to_turn() - angle_origin );
}


void define_origin(){
    angle_origin = encoder_to_turn();
    theta = 0.0;
    origin_is_set = true;
    compute_rotor_position();
}

#define DT_MIN_FOR_POSTION_ASSERV 0.001
static float dt = 0;
static float time = 0;

void update_dt_for_position_asserv(){
    float new_time = micros();
    dt = (new_time - time)/1000000.0;
    if( dt <= DT_MIN_FOR_POSTION_ASSERV ) return; //dt = 0.00001;
    time = new_time;
}

TERMINAL_PARAMETER_BOOL(dbg_pos, "", false);

static float theta_consign = 0.0;
bool position_asserv = false;

static int last_time = 0;

void display_msg(){
    if (((millis() - last_time) > 500) && dbg_pos) {
        last_time = millis();
        terminal_io()->print("angle ");
        terminal_io()->print(theta);
        terminal_io()->print("Speed: ");
        terminal_io()->println(servo_public_speed);
        terminal_io()->println();
    }
}



void servo_tick()
{
    if (security_get_error() != SECURITY_NO_ERROR) {
        error.init();
        cmd.init();
        motor_set(false, 0);
    } else {
        if (servo_flag) {
            servo_flag = false;

#ifdef ENCODER_NONE
            if (servo_enable) {
                // No servoing, the value is just 0-1 of pwm max
                if (fabs(servo_target) > 0.1) {
                    motor_set(true, servo_target*PWM_MAX);
                } else {
                    motor_set(false, 0);
                }
            }
#else
            /*// Updating limited target
            float maxAcc = ACC_MAX/1000.0;
            float diff = servo_limited_target - servo_target;
            if (fabs(diff) < maxAcc) {
                servo_limited_target = servo_target;
            } else {
                if (diff > 0) {
                    servo_limited_target -= maxAcc;
                } else {
                    servo_limited_target += maxAcc;
                }
            }*/

            // XXX: Removing servo limit
              servo_limited_target = servo_target;

            // Storing current value
            int current_value = encoder_value();
            encoder_rb[encoder_pos] = current_value;
            encoder_pos++;
            if (encoder_pos >= SPEED_RB) {
                encoder_pos = 0;
            }
            int past_value = encoder_rb[encoder_pos];

            // Updating current speed estimation [pulse per SPEED_DT]
            // XXX: Is there a problem when we overflowed?
            int speed_pulse = current_value - past_value;

            // Converting this into a speed [turn/s]
            // XXX: The discount was not tuned properly
            servo_speed = 0.95*servo_speed +  0.05*(1000.0/(double)SPEED_DT)*speed_pulse/(double)ENCODER_CPR;

            if (sdb) {
                //terminal_io()->println(sdb_t);
                servo_enable = true;
                #if RAMP == 1
                  timer++;
                  sdb_t += 1;
                  aim += 1;
                  motor_set(true, -aim);


                  if(aim >= 2500){
                    sdb = false;
                    //motor_set(false, 0);
                    servo_set = 5;
                  }
                #endif

                #if STEP == 1
                 sdb_t +=1;
                 if ((sdb_t >= 0)&(sdb_t < 1000)) {
                   //aim = STEP1;
                   //motor_set(true, aim);
                   servo_target = STEP1;
                 }
                 if (sdb_t == 5000){
                   //aim = STEP2;
                   //motor_set(true, aim);
                   servo_target = STEP2;
                 }
                 if(sdb_t >= 12000){
                   //sdb = false;
                   //motor_set(false, 0);
                   servo_target = STEP3;
                 }
                 if(sdb_t >= 12500){
                   sdb = false;
                   servo_target = 0;
                   //motor_set(false, 0);
                 }
               #endif

            } else {
                sdb_t = 0;
            }

            if (servo_enable) {
                /*float error = (servo_speed - servo_limited_target);

                // Applying prior LUT
                servo_pwm = -servo_prior_pwm;

                // Limiting the P impact
                float j = kp*error;
                if (j > 800) j = 800;
                if (j < -800) j = -800;

                servo_pwm += j
                          + round(ki * servo_acc)
                          + (error - servo_last_error) * kd;

                servo_acc += error;
                servo_last_error = error;

                // Limiting output PWM
                if (servo_pwm < -3000) servo_pwm = -3000;
                if (servo_pwm > 3000) servo_pwm = 3000;

                // Limiting accumulator
                if (servo_acc < -(3000/ki)) servo_acc = -(3000/ki);
                if (servo_acc > (3000/ki)) servo_acc = (3000/ki);

                // Limiting PWM variation
                if (abs(servo_pwm_limited - servo_pwm) > 25) {
                    if (servo_pwm_limited < servo_pwm) servo_pwm_limited += 25;
                    else servo_pwm_limited -= 25;
                } else {
                    servo_pwm_limited = servo_pwm;sr
                }
                motor_set(servo_pwm_limited);
            }*/

                double servo_filt_target = servo_target*2*3.1416;
                double curr_error =  servo_filt_target - (servo_speed)*2*3.1416;

                #if BO == 1
                  curr_error = servo_target;
                  #endif


                //motor_set(true, curr_error*0.3162);

                error.top(curr_error);

                double new_cmd = 0;//insérer combinaison linéaire;
                double new_cmd_V = 0;


                for(int i = 0; i < NBR_COEF; i++){
                  new_cmd_V += coef_cmd[i]*cmd.get_Value(i) + coef_err[i]*error.get_Value(i);
                  #if PLOT == 0
                  terminal_io()->println(i);
                  terminal_io()->print("cmd = ");
                  terminal_io()->println(cmd.get_Value(i));
                  terminal_io()->print("err = ");
                  terminal_io()->println(error.get_Value(i));
                  #endif
                }

                new_cmd_V = (new_cmd_V >= 18)? 18: new_cmd_V;
                new_cmd_V = (new_cmd_V <= -18)? -18 : new_cmd_V;
                new_cmd = new_cmd_V*3000/VMAX;
                cmd.top(new_cmd_V);

                #if PLOT == 0
                terminal_io()->print("new_cmd = ");
                terminal_io()->println(new_cmd);
                terminal_io()->print("new_cmd_V = ");
                terminal_io()->println(new_cmd_V);
                terminal_io()->println();
                #endif

                #if PLOT == 1
                //terminal_io()->println(cpt);
                terminal_io()->print(servo_speed);
                terminal_io()->print(" ");
                terminal_io()->print(servo_target);
                terminal_io()->print(" ");
                terminal_io()->print(new_cmd_V);
                terminal_io()->print(" ");
                #endif

                #if PLOT_CURR == 1

                terminal_io()->print(current_amps());

                #endif

                // terminal_io()->print(servo_speed);
                // terminal_io()->print(" ");
                // terminal_io()->println(servo_target);

                cpt++;
                motor_set(true, -new_cmd);

          }
#endif
        }
    }

    servo_public_pwm = servo_pwm_limited;
    servo_public_speed = servo_speed;


    // Position asservissment
    compute_rotor_position();
    if( position_asserv ){
        update_dt_for_position_asserv();
        if( dt <= DT_MIN_FOR_POSTION_ASSERV ) return;
        
        derivative_position_error =  theta_error;
        theta_error = theta_consign - theta;
        derivative_position_error = (theta_error - derivative_position_error)/dt;
        integral_position_error += ( dt * theta_error );

        speed_consign = (
            pos_kp * theta_error + 
            pos_ki * integral_position_error +
            pos_kd * derivative_position_error
        );
        if( fabs(speed_consign) > max_speed ){
            speed_consign = ( (speed_consign >= 0 )? +1 : -1 ) * max_speed; 
        }
        servo_set(true, speed_consign);

        //Security tests 
        if( theta < min_theta or theta > max_theta ){
            security_set_error(SECURITY_POSOTION_OUT_OF_LIMITS);
        }
        if( servo_public_speed > MAX_SPEED ){
            security_set_error(SECURITY_POSOTION_OUT_OF_LIMITS);
        }

    }
    display_msg();
}

TERMINAL_COMMAND(dbg, "Dbg servo")
{
    terminal_io()->println("PWM: ");
    terminal_io()->println(servo_pwm);
    terminal_io()->println("Error: ");
    terminal_io()->println(servo_last_error);
    terminal_io()->println("Acc: ");
    terminal_io()->println(servo_acc);
}

void position_set(float consign)
{
    theta_consign = consign;
    servo_set(true, 0.0);
    position_asserv = true;
}

void servo_set(bool enable, float target, int16_t pwm)
{
    servo_enable = enable;
    servo_target = target;
    servo_prior_pwm = pwm;

    if (!servo_enable) {
        servo_pwm = 0;
        servo_prior_pwm = 0;
        servo_pwm_limited = 0;
        servo_acc = 0;
        servo_last_error = 0;
        servo_limited_target = 0;
        current_resample();
        motor_set(false, 0);
        security_set_error(SECURITY_NO_ERROR);
    }
}

void servo_set_pid(float kp_, float ki_, float kd_)
{
    kp = kp_;
    ki = ki_;
    kd = kd_;
}

void set_position_pid(float kp_, float ki_, float kd_)
{
    pos_kp = kp_;
    pos_ki = ki_;
    pos_kd = kd_;
}

float minimal_angle(){
    return min_theta;
}

float maximal_angle(){
    return max_theta;
}

float servo_get_speed()
{
    return servo_public_speed;
}

TERMINAL_COMMAND(speed, "Speed estimation")
{
    terminal_io()->println(servo_get_speed()*100);
}

void reset_asserv(){
    servo_set(false, 0.0);
    theta_consign = 0.0;
    position_asserv = false;
    speed_consign = 0.0;
}


TERMINAL_COMMAND(em, "Emergency")
{
    servo_set(false, 0);
    reset_asserv();
    sdb = false;
}

TERMINAL_COMMAND(st, "Stop")
{
    servo_set(false, 0);
    motor_set(true, 0);
    sdb = false;
}

TERMINAL_COMMAND(step, "Step")
{
  if (argc > 0) {
    if(servo_enable){
      servo_target = atof(argv[0]); // cmd en tour par secondes
    }
  }
  else{
    terminal_io()->println("Usage: step [turn/s]");
  }
    //do nothing, use sdb=1 to avoid the servo impact
}
TERMINAL_COMMAND(clean, "Clean fifo of asserv")
{
  servo_set(false, 0);
  motor_set(true, 0);
  sdb = false;
  error.init();
  cmd.init();
}

TERMINAL_COMMAND(set, "Set target speed")
{
    if (argc > 0) {
        servo_set(true, atof(argv[0]));
        error.init();
        cmd.init();
    } else {
        terminal_io()->println("Usage: set [turn/s]");
    }
}

TERMINAL_COMMAND(set_origin, "Set position")
{
    reset_asserv();
    define_origin();
    min_theta = 0.0;
    max_theta = 0.0;
}

TERMINAL_COMMAND(limits, "Print limits")
{
    terminal_io()->print("min angle : " );
    terminal_io()->println(min_theta);
    terminal_io()->print("max angle : " );
    terminal_io()->println(max_theta);
    terminal_io()->print("max speed : " );
    terminal_io()->println(max_speed);
    terminal_io()->println();
}

TERMINAL_COMMAND(set_min_angle, "Set minimum angle")
{
    reset_asserv();
    if( !origin_is_set ){
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
    min_theta = theta;
}

TERMINAL_COMMAND(set_max_speed, "Set maximum speed")
{
    char* endptr = NULL;
    float value = strtod(argv[0], &endptr);
    if( value == 0.0 and endptr == argv[0] ){
        terminal_io()->print("Invalid parameter.");
        terminal_io()->println();
        return;
    }
    if( value < 0 ){
        terminal_io()->print("Maximal speed should be positive.");
        terminal_io()->println();
        return;
    }
    if( value > MAX_SPEED ){
        terminal_io()->print("Maximal speed should be lesser than "MAX_SPEED_STR);
        terminal_io()->println();
        return;
    }
    max_speed = value;
}

TERMINAL_COMMAND(set_max_angle, "Set maximum angle")
{
    reset_asserv();
    if( !origin_is_set ){
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
    max_theta = theta;
}

TERMINAL_COMMAND(set_position, "Set position")
{
    if( !origin_is_set ){
        terminal_io()->print("First define the origin by typing :" );
        terminal_io()->println();
        terminal_io()->println();
        terminal_io()->print("    set_origin");
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
    if( consign < minimal_angle() ){
        terminal_io()->print("Angle is too small." );
        terminal_io()->println();
        return;
    }
    if( consign > maximal_angle() ){
        terminal_io()->print("Angle is too big." );
        terminal_io()->println();
        return;
    }

    if (argc > 0) {
        position_set(consign);
        error.init();
        cmd.init();
    } else {
        terminal_io()->println("Usage: set [turn/s]");
    }
}


int servo_get_pwm()
{
    return servo_public_pwm;
}

TERMINAL_COMMAND(servo, "Servo status")
{
    terminal_io()->print("Target angle : ");
    terminal_io()->println(theta_consign);
    terminal_io()->print("Target speed: ");
    terminal_io()->println(servo_target);
    terminal_io()->print("Prior PWM: ");
    terminal_io()->println(servo_prior_pwm);
    terminal_io()->print("Target speed: ");
    terminal_io()->println(servo_target);
    terminal_io()->print("Angle: ");
    terminal_io()->println(theta);
    terminal_io()->print("Speed: ");
    terminal_io()->println(servo_public_speed);
    terminal_io()->print("PWM: ");
    terminal_io()->println(servo_public_pwm);
}

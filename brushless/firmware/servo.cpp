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
#define NBR_COEF 5
#define ART_LIM  2500


#define RAMP  0    //ramp enable with sdb=1 command
#define STEP  1    //step enable with sdb=1 command
#define STEP1 1    //define first step value (tr/s)
#define STEP2 3
#define STEP3 5


#define ASSERV_FPI 0    //Fast PI regulator
#define ASSERV_PI  1
#define FF_95      1    //Feedforward with wx = 95 rad/s
#define PRINT      0
////////////////////////////////////////////////////////////////////////////////:

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
////////////////////////////////////////////////////////////////////////////////:

#if ASSERV_FPI
// PI correct
static double coef_err[NBR_COEF] = {0.0512, -0.0502, 0, 0, 0};//t  t-1   t-2  t-3
static double coef_cmd[NBR_COEF] = {1, 0, 0, 0, 0}; //t-1 t-2 t-3 t-4
#endif

#if ASSERV_PI
//PI
static double coef_err[NBR_COEF] = {0.05241, -0.05137, 0, 0, 0}; //t  t-1   t-2  t-3
static double coef_cmd[NBR_COEF] = {1, 0, 0, 0, 0}; //t-1 t-2 t-3 t-4
#endif

#if FF_95
//***********************************************************************//
//FeedForward et Prefiltre modèle charge wx = 95 rad/s
/*
     ne diverge pas, glissement léger
*/
static double coef_ff_cns[NBR_COEF] = {0.0016, 0.0030, -0.0086, 0.0027, 0.0013};
static double coef_ff_cmd[NBR_COEF] = {3.6375, -4.9618, 3.0081, -0.6839, 0};

static double coef_f_cns[NBR_COEF] = {0.000000637119, 0.000015555, 0.000037067, 0.000013704, 0.00000049459};
static double coef_f_cns_f[NBR_COEF] = {3.6375, -4.9618, 3.0081, -0.6839, 0};
//***********************************************************************//
#endif

////////////////////////////////////////////////////////////////////////////////:

fifo fifo_error(NBR_COEF);
fifo fifo_command(NBR_COEF);
fifo fifo_consigne(NBR_COEF);
fifo fifo_filtred_consigne(NBR_COEF);
fifo fifo_command_ff(NBR_COEF);
fifo fifo_command_pid(NBR_COEF);
static int cpt = 0;

/////////////////////////////////////////////////////////////////



TERMINAL_PARAMETER_BOOL(sdb, "Speed debug", false)
static int sdb_t = 0;

int timer = 0;
int aim = 0;

static void servo_irq()
{
    if (encoder_read()) {
        servo_flag = true;
    }
}

static void init_timer()
{
    HardwareTimer timer(4);

    // Configuring timer
    timer.pause();
    timer.setPrescaleFactor(9);
    timer.setOverflow(1000); // 8Khz

    timer.setChannel4Mode(TIMER_OUTPUT_COMPARE);
    timer.setCompare(TIMER_CH4, 1);
    timer.attachCompare4Interrupt(servo_irq);

    timer.refresh();
    timer.resume();
}

void servo_init()
{
    init_timer();
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

void servo_tick()
{
    if (security_get_error() != SECURITY_NO_ERROR) {
        fifo_error.init();
        fifo_command.init();
        fifo_consigne.init();
        fifo_filtred_consigne.init();
        fifo_command_ff.init();
        fifo_command_pid.init();
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
                //terminal_io()->println(encoder_value());
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
                 if ((sdb_t >= 0)&(sdb_t < 2000)) {
                   //aim = STEP1;
                   //motor_set(true, aim);
                   servo_target = STEP1;
                 }
                 if (sdb_t == 4000){
                   //aim = STEP2;
                   //motor_set(true, aim);
                   servo_target = STEP2;
                 }
                 if(sdb_t >= 6000){
                   sdb = false;
                   //motor_set(false, 0);
                   servo_target = STEP3;
                 }
                 if(sdb_t >= 8000){
                   sdb = false;
                   servo_enable = false;
                   //motor_set(false, 0);
                 }
               #endif

            } else {
                sdb_t = 0;
            }

            if (servo_enable) {

                double current_command = 0;
                double current_command_v = 0;

                double current_filtred_consigne = 0;
                double current_command_ff_v = 0;
                double current_command_pid_v = 0;

                double servo_filt_target = servo_target*2*3.14159265359;
                fifo_consigne.top(servo_filt_target);


                //Filtrage de consigne

                for(int j = 0; j < NBR_COEF; j++){
                  current_filtred_consigne += coef_f_cns_f[j]*fifo_filtred_consigne.get_Value(j) + coef_f_cns[j]*fifo_consigne.get_Value(j);
                }
                fifo_filtred_consigne.top(current_filtred_consigne);


                double current_error = servo_filt_target - (servo_speed)*2*3.14159265359;
                fifo_error.top(current_error);

                //FeedForward et Regulation

                for(int i = 0; i < NBR_COEF; i++){
                  current_command_ff_v += coef_ff_cmd[i]*fifo_command_ff.get_Value(i) + coef_ff_cns[i]*fifo_consigne.get_Value(i);
                  current_command_pid_v += coef_cmd[i]*fifo_command_pid.get_Value(i) + coef_err[i]*fifo_error.get_Value(i);
                }

                fifo_command_ff.top(current_command_ff_v);
                fifo_command_pid.top(current_command_pid_v);

                current_command_v = current_command_pid_v + current_command_ff_v;

                current_command_v = (current_command_v >=  18) ?  18 : current_command_v;
                current_command_v = (current_command_v <= -18) ? -18 : current_command_v;
                current_command   = current_command_v*3000/VMAX;
                fifo_command.top(current_command_v);


                #if PRINT
                terminal_io()->print(servo_speed);              //1
                terminal_io()->print(" ");
                terminal_io()->print(servo_target);             //2
                terminal_io()->print(" ");
                terminal_io()->print(current_command_ff_v);     //3
                terminal_io()->print(" ");
                terminal_io()->print(current_command_pid_v);    //4
                terminal_io()->print(" ");
                terminal_io()->print(current_filtred_consigne); //5
                terminal_io()->print(" ");
                terminal_io()->print(current_error);            //6
                terminal_io()->print(" ");
                terminal_io()->println(current_command_v);      //7
                #endif

                cpt++;
                motor_set(true, -current_command);

          }
#endif
        }
    }

if(curr){

}


    servo_public_pwm = servo_pwm_limited;
    servo_public_speed = servo_speed;


}

TERMINAL_COMMAND(dbg, "Dbg servo")
{
    terminal_io()->println("PWM: ");
    terminal_io()->println(servo_pwm);
    terminal_io()->println(": ");
    terminal_io()->println(servo_last_error);
    terminal_io()->println("Acc: ");
    terminal_io()->println(servo_acc);
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

float servo_get_speed()
{
    return servo_public_speed;
}

TERMINAL_COMMAND(speed, "Speed estimation")
{
    terminal_io()->println(servo_get_speed()*100);
}

TERMINAL_COMMAND(em, "Emergency")
{
    servo_set(false, 0);
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
  fifo_error.init();
  fifo_command.init();
  fifo_consigne.init();
  fifo_filtred_consigne.init();
  fifo_command_ff.init();
  fifo_command_pid.init();
}

TERMINAL_COMMAND(set, "Set target speed")
{
    if (argc > 0) {
        fifo_error.init();
        fifo_command.init();
        fifo_consigne.init();
        fifo_filtred_consigne.init();
        fifo_command_ff.init();
        fifo_command_pid.init();
        servo_set(true, atof(argv[0]));

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
    terminal_io()->print("Target speed: ");
    terminal_io()->println(servo_target);
    terminal_io()->print("Prior PWM: ");
    terminal_io()->println(servo_prior_pwm);
    terminal_io()->print("Speed: ");
    terminal_io()->println(servo_public_speed);
    terminal_io()->print("PWM: ");
    terminal_io()->println(servo_public_pwm);
}

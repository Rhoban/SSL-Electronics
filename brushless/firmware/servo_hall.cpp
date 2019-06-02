#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include "hardware.h"
#include "motor.h"
#include "current.h"
#include "servo_hall.h"
#include "encoder.h"
#include "security.h"
#include "motor_hall.h"

class fifo{
  public:
    fifo();
    fifo(int _length);
    ~fifo();

    void top(double _newValue);
    void init();
    int get_Length();
    double get_Value(int _pos);

  private:
    int length;
    double *data;
    void pop();
};


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
static volatile int16_t servo_public_pwm = 0;

#define VMAX 20
#define VLIMIT 18
#define NBR_COEF 4

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

/////////////////////////////////////////////////////////////////

static void servo_irq()
{
    // XXX: This should probably be moved in encoder
    encoder_read();
}

void servo_hall_set_flag()
{
    servo_flag = true;
}

void servo_hall_init()
{
}

static int servo_pwm = 0;
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

void servo_hall_tick()
{
    if (security_get_error() != SECURITY_NO_ERROR) {
        error.init();
        cmd.init();
        motor_set(false, 0);
    } else {
        if (servo_flag) {
            servo_flag = false;

            servo_speed = encoder_to_float_speed();

            if (servo_enable) {
                float servo_filt_target = servo_target*2*3.1416;
                float curr_error =  servo_filt_target - (servo_speed)*2*3.1416;

                #if BO == 1
                curr_error = servo_target;
                #endif

                error.top(curr_error);

                float new_cmd = 0;//insérer combinaison linéaire;
                float new_cmd_V = 0;

                for(int i = 0; i < NBR_COEF; i++){
                  new_cmd_V += coef_cmd[i]*cmd.get_Value(i) + coef_err[i]*error.get_Value(i);
                }

                new_cmd_V = (new_cmd_V >= VLIMIT)? VLIMIT: new_cmd_V;
                new_cmd_V = (new_cmd_V <= -VLIMIT)? -VLIMIT : new_cmd_V;
                new_cmd = new_cmd_V*PWM_HALL_SUPREMUM/VMAX;
                cmd.top(new_cmd_V);

                motor_set(true, -new_cmd);
                servo_public_pwm = -new_cmd;
          }
       }
    }
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

void servo_hall_set(bool enable, float target, int16_t pwm)
{
    servo_enable = enable;
    servo_target = target;
    servo_prior_pwm = pwm;

    if (!servo_enable) {
        servo_pwm = 0;
        servo_prior_pwm = 0;
        servo_acc = 0;
        servo_last_error = 0;
        servo_limited_target = 0;
        current_resample();
        motor_set(false, 0);
        security_set_error(SECURITY_NO_ERROR);
    }
}

void servo_hall_set_pid(float kp_, float ki_, float kd_)
{
    kp = kp_;
    ki = ki_;
    kd = kd_;
}

float servo_hall_get_speed()
{
    return servo_speed;
}

TERMINAL_COMMAND(speed, "Speed estimation")
{
    terminal_io()->println(servo_hall_get_speed()*100);
}

void servo_hall_emergency(){
    servo_hall_set(false, 0);
}
void servo_hall_stop(){
    servo_hall_set(false, 0);
    motor_set(true, 0);
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
}
TERMINAL_COMMAND(clean, "Clean fifo of asserv")
{
  servo_hall_set(false, 0);
  motor_set(true, 0);
  error.init();
  cmd.init();
}

TERMINAL_COMMAND(set, "Set target speed")
{
    if (argc > 0) {
        servo_hall_set(true, atof(argv[0]));
        error.init();
        cmd.init();
    } else {
        terminal_io()->println("Usage: set [turn/s]");
    }
}

int servo_hall_get_pwm()
{
    return servo_public_pwm;
}

TERMINAL_COMMAND(servo, "Servo status")
{
    terminal_io()->print("Servo enable : ");
    terminal_io()->println(servo_enable);
    terminal_io()->print("Target speed: ");
    terminal_io()->println(servo_target);
    terminal_io()->print("speed: ");
    terminal_io()->println(servo_speed);
    terminal_io()->print("PWM: ");
    terminal_io()->println(servo_public_pwm);
}

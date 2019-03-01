#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include "hardware.h"
#include "motor.h"
#include "current.h"
#include "servo.h"
#include "encoder.h"
#include "security.h"
#include "ssl.h"

// Interrupt flag
static bool servo_flag = false;
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

void control_with_hall_sensor(){
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
    servo_public_pwm = servo_pwm_limited;
    servo_public_speed = servo_speed;
}

TERMINAL_PARAMETER_BOOL(manual_speed, "Enable manual Speed consign", MANUAL_SPEED);

static bool servo_ticking = false;

// begin of automatic code - generated by integer_calculus.py
static int speed_csg = 0*134217728;
// end of automatic code - generated by integer_calculus.py

// begin of automatic code - generated by integer_calculus.py
static int theta_c = 0;
static int k_pos_p = K_POS_P*1073741824;
static int speed_i = 0;
static int k_pos_i = K_POS_I*67108864;
static int k_speed_p = K_SPEED_P*4194304;
static int voltage_i = 0;
static int k_speed_i = K_SPEED_I*524288;
static int k_fem = KFEM*16777216;
// end of automatic code - generated by integer_calculus.py


static int output_voltage_d = 0;
// begin of automatic code - generated by integer_calculus.py
static int output_voltage_q = 0;
static int reference_voltage_q = 0;
static int voltage_q = 0;
static int voltage_p = 0;
static int limited_speed_error = 0;
static int speed_error = 0;
static int speed_c = 0;
static int speed_p = 0;
static int limited_theta_error = 0;
static int theta_error = 0;
static int speed_i__acc_sum = 0;
static int speed_load = 0;
static int voltage_i__acc_sum = 0;
static int voltage_load = 0;
static int electromagnetic_force = 0;
// end of automatic code - generated by integer_calculus.py

static int max_output_voltage = 0;
static int speed=0;
static int theta=0;

void control_with_vectorial_command(){

    if( servo_ticking ){
        return;
    }
    servo_ticking = true;

    if( ! get_serv_flag() ){
        servo_ticking = false;
        return;
    }

    if( motor_is_tared() ){
        theta = rotor_angle();
        _Static_assert( SPEED_NOMRALISATION==1048576, "");
        speed = encoder_to_speed();

        #ifdef STOP_OUTSIDE_LIMITS 
        if( theta < min_theta or theta > max_theta ){
            motor_set( false, 0 );
        }
        #endif

        // Compute velocity consign 
        if(!manual_speed){
            // begin of automatic code - generated by integer_calculus.py
            theta_error = ((-theta)*8 + theta_c/2);
            limited_theta_error = ( (theta_error > 655360) ? 655360 : ( (theta_error < -655360) ? -655360 : theta_error ) );
            speed_p = ((limited_theta_error/16)*(k_pos_p/65536));
            speed_load = ((((limited_theta_error/16)*(k_pos_i/32768))/32768)*((2684354)/64));
            speed_i__acc_sum = (speed_i/1 + speed_load/128);
            speed_i = (( (speed_i__acc_sum > 1207959552) ? 1207959552 : ( (speed_i__acc_sum < -1207959552) ? -1207959552 : speed_i__acc_sum ) )*1);
            speed_c = ( ((speed_p/2 + speed_i/2) > 603979776) ? 603979776 : ( ((speed_p/2 + speed_i/2) < -603979776) ? -603979776 : (speed_p/2 + speed_i/2) ) );
            // end of automatic code - generated by integer_calculus.py
        }else{
            // begin of automatic code - generated by integer_calculus.py
            speed_c = (speed_csg/2);
            // end of automatic code - generated by integer_calculus.py
        }

        // begin of automatic code - generated by integer_calculus.py
        speed_error = ((-speed)*8 + speed_c/8);
        limited_speed_error = ( (speed_error > 33554432) ? 33554432 : ( (speed_error < -33554432) ? -33554432 : speed_error ) );
        voltage_p = ((limited_speed_error/1024)*(k_speed_p/32768));
        voltage_load = ((((limited_speed_error/1024)*(k_speed_i/32768))/32768)*((2684354)/64));
        voltage_i__acc_sum = (voltage_i/1 + voltage_load/128);
        voltage_i = (( (voltage_i__acc_sum > 1073741824) ? 1073741824 : ( (voltage_i__acc_sum < -1073741824) ? -1073741824 : voltage_i__acc_sum ) )*1);
        electromagnetic_force = ((speed/4096)*(k_fem/32768));
        voltage_q = ((electromagnetic_force/1 + voltage_i/8)/2 + voltage_p/16);
        reference_voltage_q = ( (voltage_q > 67108864) ? 67108864 : ( (voltage_q < -67108864) ? -67108864 : voltage_q ) );
        output_voltage_q = (reference_voltage_q/65536);
        // end of automatic code - generated by integer_calculus.py

        if( max_output_voltage<output_voltage_q){
            max_output_voltage = output_voltage_q;
        }
        output_voltage_d = 0;
        direct_quadrature_voltage_set(output_voltage_d, output_voltage_q);
    }
    
    reset_serv_flag();
    servo_ticking = false;
}

enum ServoMode {
    SERVO_HALL,
    SERVO_VECTORIAL
};

const ServoMode servo_mode = SERVO_VECTORIAL;

void servo_tick()
{
    if (security_get_error() != SECURITY_NO_ERROR) {
        error.init();
        cmd.init();
        motor_set(false, 0);
    } else {
        if( servo_mode == SERVO_HALL ){
            control_with_hall_sensor();
            reset_serv_flag();
        }else{
            control_with_vectorial_command();
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

TERMINAL_COMMAND(kpp, "Set P for position PID")
{
    if (argc == 1) {
        // begin of automatic code - generated by integer_calculus.py
        float k_pos_p_input = atof(argv[0]);
        if(k_pos_p_input < 0.0) k_pos_p_input = 0.0;
        if(k_pos_p_input > 1.8) k_pos_p_input = 1.8;
        k_pos_p = (int) (k_pos_p_input * 1073741824);
        // end of automatic code - generated by integer_calculus.py
    } else {
        if(argc==0){
            terminal_io()->println(((double)k_pos_p)/1073741824);
            return;
        }
        terminal_io()->println("Usage: kpp [0.0 - 1.8]");
    }
}
TERMINAL_COMMAND(kpi, "Set I for position PID")
{
    if (argc == 1) {
        // begin of automatic code - generated by integer_calculus.py
        float k_pos_i_input = atof(argv[0]);
        if(k_pos_i_input < 0.0) k_pos_i_input = 0.0;
        if(k_pos_i_input > 18.0) k_pos_i_input = 18.0;
        k_pos_i = (int) (k_pos_i_input * 67108864);
        // end of automatic code - generated by integer_calculus.py
    } else {
        if(argc==0){
            terminal_io()->println(((double)k_pos_i)/67108864);
            return;
        }
        terminal_io()->println("Usage: kpi [0.0 - 18.0]");
    }
}
TERMINAL_COMMAND(ksp, "Set P for speed PID")
{
    if (argc == 1) {
        // begin of automatic code - generated by integer_calculus.py
        float k_speed_p_input = atof(argv[0]);
        if(k_speed_p_input < 0.0) k_speed_p_input = 0.0;
        if(k_speed_p_input > 256.0) k_speed_p_input = 256.0;
        k_speed_p = (int) (k_speed_p_input * 4194304);
        // end of automatic code - generated by integer_calculus.py
    } else {
        if(argc==0){
            terminal_io()->println(((double)k_speed_p)/4194304);
            return;
        }
        terminal_io()->println("Usage: ksp [ 0.0 - 256.0 ]");
    }
}
TERMINAL_COMMAND(ksi, "Set I for speed PID")
{
    if (argc == 1) {
        // begin of automatic code - generated by integer_calculus.py
        float k_speed_i_input = atof(argv[0]);
        if(k_speed_i_input < 0.0) k_speed_i_input = 0.0;
        if(k_speed_i_input > 2560.0) k_speed_i_input = 2560.0;
        k_speed_i = (int) (k_speed_i_input * 524288);
        // end of automatic code - generated by integer_calculus.py
    } else {
        if(argc==0){
            terminal_io()->println(((double)k_speed_i)/524288);
            return;
        }
        terminal_io()->println("Usage: ksi [ 0.0 - 2560.0 ]");
    }
}
TERMINAL_COMMAND(kem, "Set I for speed PID")
{
    if (argc == 1) {
        // begin of automatic code - generated by integer_calculus.py
        float k_fem_input = atof(argv[0]);
        if(k_fem_input < 0.0) k_fem_input = 0.0;
        if(k_fem_input > 113.777) k_fem_input = 113.777;
        k_fem = (int) (k_fem_input * 16777216);
        // end of automatic code - generated by integer_calculus.py
    } else {
        if(argc==0){
            terminal_io()->println(((double)k_fem)/16777216);
            return;
        }
        terminal_io()->println("Usage: kem [ 0.0 - 113.777 ]");
    }
}

TERMINAL_COMMAND(theta_c, "Set angle consign")
{
    if (argc == 1) {
        // begin of automatic code - generated by integer_calculus.py
        float theta_c_input = atof(argv[0]);
        if(theta_c_input < -6000.0) theta_c_input = -6000.0;
        if(theta_c_input > 6000.0) theta_c_input = 6000.0;
        theta_c = (int) (theta_c_input * 262144);
        // end of automatic code - generated by integer_calculus.py
    } else {
        if(argc==0){
            terminal_io()->println(((double)theta_c)/262144);
            return;
        }
        terminal_io()->println("Usage: theta_c [ -6000.0 - 6000.0 ]");
    }
}

TERMINAL_COMMAND(info, "Info")
{
//    terminal_io()->print("theta : ");
//    terminal_io()->println(theta);
//    terminal_io()->print("speed : ");
//    terminal_io()->println(speed);
    terminal_io()->println(speed_c);
//    terminal_io()->println(speed_error);
//    terminal_io()->print("em : ");
//    terminal_io()->println(electromagnetic_force);
//    terminal_io()->print("out_q : ");
//    terminal_io()->println(output_voltage_q);
//    terminal_io()->print("max_out_q : ");
//    terminal_io()->println(max_output_voltage);
//    terminal_io()->print("speed_error : ");
//    terminal_io()->println(speed_error);
//    terminal_io()->print("limited_speed_error : ");
//    terminal_io()->println(limited_speed_error);
//    terminal_io()->print("voltage_p : ");
//    terminal_io()->println(voltage_p);
//      terminal_io()->println(voltage_load);
//      terminal_io()->println(voltage_i__acc_sum);
//      terminal_io()->println(voltage_i);
//terminal_io()->println(theta_error);
//terminal_io()->println(limited_theta_error);
//terminal_io()->println(speed_p);
//terminal_io()->println(speed_load);
//terminal_io()->println(speed_i__acc_sum);
//terminal_io()->println(speed_i);
//terminal_io()->println(speed_c);

}

TERMINAL_COMMAND(speed_csg, "Set speed consign")
{
    if (argc == 1) {
        // begin of automatic code - generated by integer_calculus.py
        float speed_csg_input = atof(argv[0]);
        if(speed_csg_input < -9.0) speed_csg_input = -9.0;
        if(speed_csg_input > 9.0) speed_csg_input = 9.0;
        speed_csg = (int) (speed_csg_input * 134217728);
        // end of automatic code - generated by integer_calculus.py
    } else {
        if(argc==0){
            terminal_io()->println((double)speed_csg/134217728);
            return;
        }
        terminal_io()->println("Usage: speed_csg [ -9.0 - 9.0 ]");
    }
}

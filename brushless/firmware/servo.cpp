
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

TERMINAL_PARAMETER_BOOL(sdb, "Speed debug", false)
static int sdb_t = 0;

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
        motor_set(0);
    } else {
        if (servo_flag) {
            servo_flag = false;

#ifdef ENCODER_NONE
            if (servo_enable) {
                // No servoing, the value is just 0-1 of pwm max
                if (fabs(servo_target) > 0.1) {
                    motor_set((servo_target/20.0)*PWM_MAX);
                } else {
                    motor_set(0);
                }
            }
#else
            // Updating limited target
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
            }

            // XXX: Removing servo limit
            //  servo_limited_target = servo_target;

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
                terminal_io()->println(encoder_value());
                // sdb_t += 1;
                // if (sdb_t == 500) {
                //     servo_set(true, 2);
                // }
                // if (sdb_t == 2000) {
                //     // servo_set(true, 2);
                // }
                // if (sdb_t == 2000) {
                //     servo_set(true, 0);
                // }
                // if (sdb_t == 5000) {
                //     sdb = false;
                // }
                // terminal_io()->print(servo_get_speed()*100);
                // terminal_io()->print(" ");
                // terminal_io()->print(servo_limited_target*100);
                // terminal_io()->println();
            } else {
                sdb_t = 0;
            }

            if (servo_enable) {
                float error = (servo_speed - servo_limited_target);

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
                    servo_pwm_limited = servo_pwm;
                }
                motor_set(servo_pwm_limited);
            }
#endif
        }
    }

    servo_public_pwm = servo_pwm_limited;
    servo_public_speed = servo_speed;
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
        motor_set(0);
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

TERMINAL_COMMAND(set, "Set target speed")
{
    if (argc > 0) {
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

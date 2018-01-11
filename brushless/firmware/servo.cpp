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
static float servo_limited_target = 0;

// Speed estimation
static float servo_speed = 0;

// Number of values stored in the speed ring buffer
#define SPEED_RB        (((SPEED_DT)/SERVO_DT)+1)
static int encoder_rb[SPEED_RB] = {0};
static int encoder_pos = 0;

static void servo_irq()
{
    encoder_read();
    servo_flag = true;
}

static void init_timer()
{
    HardwareTimer timer(4);

    // Configuring timer
    timer.pause();
    timer.setPrescaleFactor(72);
    timer.setOverflow(1000); // 1Khz

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
static float servo_acc = 0;
static float servo_last_error = 0;
TERMINAL_PARAMETER_INT(kp, "PID P", 500);
TERMINAL_PARAMETER_INT(ki, "PID I", 5);
TERMINAL_PARAMETER_INT(kd, "PID D", 1);

void servo_tick()
{
    if (security_get_error() != SECURITY_NO_ERROR) {
        motor_set(0);
    } else {
        if (servo_flag) {
            servo_flag = false;

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

            // Storing current value
            int current_value = encoder_value();
            encoder_rb[encoder_pos] = current_value;
            encoder_pos++;
            if (encoder_pos >= SPEED_RB) {
                encoder_pos = 0;
            }
            int past_value = encoder_rb[encoder_pos];

            // Updating current speed estimation [pulse per SPEED_DT]
            int speed_pulse = current_value - past_value;

            // Converting this into a speed [turn/s]
            servo_speed = (1000/(float)SPEED_DT)*speed_pulse/(float)ENCODER_CPR;

            if (servo_enable) {
                float error = (servo_speed - servo_limited_target);

                servo_pwm = kp * error + servo_acc + (error - servo_last_error) * kd;

                servo_acc += ki * error;
                servo_last_error = error;

                if (servo_pwm < -3000) servo_pwm = -3000;
                if (servo_pwm > 3000) servo_pwm = 3000;
                if (servo_acc < -3000) servo_acc = -3000;
                if (servo_acc > 3000) servo_acc = 3000;

                motor_set(servo_pwm);
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

void servo_set(bool enable, float target)
{
    servo_enable = enable;
    servo_target = target;

    if (!servo_enable) {
        servo_pwm = 0;
        servo_acc = 0;
        servo_last_error = 0;
        servo_limited_target = 0;
        current_resample();
        motor_set(0);
        security_set_error(SECURITY_NO_ERROR);
    }
}

float servo_get_speed()
{
    return servo_speed;
}

TERMINAL_COMMAND(speed, "Speed estimation")
{
    terminal_io()->println(servo_get_speed());
}

TERMINAL_COMMAND(em, "Emergency")
{
    servo_set(false, 0);
}

TERMINAL_COMMAND(set, "Set target speed")
{
    if (argc > 0) {
        servo_set(true, atof(argv[0]));
    } else {
        terminal_io()->println("Usage: set [turn/s]");
    }
}

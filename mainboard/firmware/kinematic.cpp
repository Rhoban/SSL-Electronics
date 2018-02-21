#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include <math.h>
#include <watchdog.h>
#include "drivers.h"
#include "kinematic.h"

static float target_x = 0, target_y = 0, target_t = 0;
static float front_left = 0, front_right = 0, rear_left = 0, rear_right = 0;
static bool enabled = false;
static int enabled_since = 0;

#define DEG2RAD(deg) (deg*M_PI/180.0)
#define WHEEL_RADIUS (0.06/2.0)
#define ROBOT_RADIUS (0.17/2.0)
#define ANGLE_REAR   DEG2RAD(45)
#define ANGLE_FRONT  DEG2RAD(120)

#define FRONT_LEFT_X     -sin(ANGLE_FRONT)
#define FRONT_LEFT_Y     cos(ANGLE_FRONT)
#define FRONT_RIGHT_X    -sin(-ANGLE_FRONT)
#define FRONT_RIGHT_Y    cos(-ANGLE_FRONT)
#define REAR_LEFT_X      -sin(ANGLE_REAR)
#define REAR_LEFT_Y      cos(ANGLE_REAR)
#define REAR_RIGHT_X     -sin(-ANGLE_REAR)
#define REAR_RIGHT_Y     cos(-ANGLE_REAR)

void kinematic_compute(float x, float y, float t,
    float *frontLeft, float *frontRight, float *rearLeft, float *rearRight)
{
    *frontLeft = (FRONT_LEFT_X*x + FRONT_LEFT_Y*y +
        ROBOT_RADIUS*t)/(2*M_PI*WHEEL_RADIUS);

    *frontRight = (FRONT_RIGHT_X*x + FRONT_RIGHT_Y*y +
        ROBOT_RADIUS*t)/(2*M_PI*WHEEL_RADIUS);

    *rearLeft = (REAR_LEFT_X*x + REAR_LEFT_Y*y +
        ROBOT_RADIUS*t)/(2*M_PI*WHEEL_RADIUS);

    *rearRight = (REAR_RIGHT_X*x + REAR_RIGHT_Y*y +
        ROBOT_RADIUS*t)/(2*M_PI*WHEEL_RADIUS);
}

void kinematic_set(float x, float y, float t)
{
    target_x = x;
    target_y = y;
    target_t = t;
    enabled = true;
    enabled_since = millis();
}

void kinematic_stop()
{
    target_x = 0;
    target_y = 0;
    target_t = 0;
    enabled = false;
}

#define MAX_ACCELERATION    (10*0.01)

void kinematic_tick()
{
    static int last_tick = millis();
    int elapsed = millis() - last_tick;

    if (elapsed >= 10) { // Every 10ms
        last_tick += elapsed;

        if (enabled) {
            if (millis() - enabled_since > 100) {
                kinematic_stop();
            } else {
                float new_front_left, new_front_right, new_rear_left, new_rear_right;
                kinematic_compute(target_x, target_y, target_t,
                    &new_front_left, &new_front_right, &new_rear_left, &new_rear_right);

                float delta1 = fabs(new_front_left - front_left);
                float delta2 = fabs(new_front_right - front_right);
                float delta3 = fabs(new_rear_left - rear_left);
                float delta4 = fabs(new_rear_right - rear_right);
                if (delta2 > delta1) delta1 = delta2;
                if (delta3 > delta1) delta1 = delta3;
                if (delta4 > delta1) delta1 = delta4;

                if (delta1 > MAX_ACCELERATION) {
                    float alpha = MAX_ACCELERATION/delta1;
                    front_left = front_left + alpha*(new_front_left - front_left);
                    front_right = front_right + alpha*(new_front_right - front_right);
                    rear_left = rear_left + alpha*(new_rear_left - rear_left);
                    rear_right = rear_right + alpha*(new_rear_right - rear_right);
                } else {
                    front_left = new_front_left;
                    front_right = new_front_right;
                    rear_left = new_rear_left;
                    rear_right = new_rear_right;
                }

                // terminal_io()->print(front_left);
                // terminal_io()->print(" ");
                // terminal_io()->print(rear_left);
                // terminal_io()->print(" ");
                // terminal_io()->print(rear_right);
                // terminal_io()->print(" ");
                // terminal_io()->print(front_right);
                // terminal_io()->println();

                drivers_set_safe(0, true, front_left);
                drivers_set_safe(1, true, rear_left);
                drivers_set_safe(2, true, rear_right);
                drivers_set_safe(3, true, front_right);
            }
        } else {
            front_left = 0;
            front_right = 0;
            rear_left = 0;
            rear_right = 0;
        }
    }
}

TERMINAL_COMMAND(kin, "Kinematic")
{
    if (argc >= 3) {
        while (!SerialUSB.available()) {
            kinematic_set(atof(argv[0]), atof(argv[1]), atof(argv[2]));
            kinematic_tick();
            watchdog_feed();
            delay(5);
        }
    }
}
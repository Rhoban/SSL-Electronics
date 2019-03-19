#ifndef _MOTOR_HYBRID_H
#define _MOTOR_HYBRID_H

#include "ssl.h"

#define MIN_LOW_NORMAL_SPEED_HYSTERESIS 0.6
#define MAX_LOW_NORMAL_SPEED_HYSTERESIS 0.8
#define MIN_NORMAL_HIGH_SPEED_HYSTERESIS 9.0
#define MAX_NORMAL_HIGH_SPEED_HYSTERESIS 10.0


void servo_hybrid_init();
void servo_hybrid_tick();
void servo_hybrid_set(bool enable, float targetSpeed, int16_t pwm=0);
float servo_hybrid_get_speed();
int servo_hybrid_get_pwm();
void servo_hybrid_set_pid(float kp, float ki, float kd);
void servo_hybrid_set_speed_consign( float speed );
void servo_hybrid_set_flag();
void servo_hybrid_emergency();
void servo_hybrid_stop();

enum HybridState {
    NO_STATE,
    LOW_SPEED,
    NORMAL_SPEED,
    HIGH_SPEED
};

static inline const char *hyb_state_str(HybridState s){
    switch(s){
        case NO_STATE:
            return "NO_STATE";
        case LOW_SPEED:
            return "LOW_SPEED";
        case NORMAL_SPEED:
            return "NORMAL_SPEED";
        case HIGH_SPEED:
            return "HIGH_SPEED";
        default :
            return "?";
    }
}


#endif

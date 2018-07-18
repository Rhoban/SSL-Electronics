#ifndef _BL_SERVO_H
#define _BL_SERVO_H

#include <stdint.h>

/**
 * Initializing servo system
 */
void servo_init();

/**
 * Ticks servo
 */
void servo_tick();
void servo_set_flag();

/**
 * Sets the servo value, enables or not the motor control, target speed is
 * in [turn/s]
 */
void servo_set(bool enable, float targetSpeed, int16_t pwm=0);

/**
 * Sets the PID parameters
 */
void servo_set_pid(float kp, float ki, float kd);

/**
 * Current speed [turn/s]
 */
float servo_get_speed();

/**
 * Current PWM output [-3000 to 3000]
 */
int servo_get_pwm();


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

#endif

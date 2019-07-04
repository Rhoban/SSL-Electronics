#ifndef _SERVO_H

void servo_set(bool enable, float targetSpeed, int pwm);
float servo_get_speed();
int servo_get_pwm();
void servo_set_pid(float kp, float ki, float kd);
void servo_set_flag();
void servo_stop();
void servo_set_speed_consign( float speed );

#endif

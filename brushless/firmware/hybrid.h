#ifndef _HYBRID_H
#define _HYBRID_H

typedef enum {
  FOC,
  HALL
} HybridMod;


float hybrid_get_speed();

int hybrid_get_pwm();

void hybrid_process_packet( bool enable, float targetSpeed, int pwm);

void motor_set(bool enable, int value);

void hybrid_tick( );

void switch_to_hall();
void switch_to_foc();

#endif

#ifndef _KINEMATIC_H
#define _KINEMATIC_H

void kinematic_compute(float x, float y, float t,
    float *w1, float *w2, float *w3, float *w4);

void kinematic_stop();

void kinematic_set(float x, float y, float t);

void kinematic_tick();

#endif

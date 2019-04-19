#include "sin_lut.h"

#include "sin_samples.h"

inline int16_t discrete_sin_positive(int16_t x){
    if (x <= SIN_INPUT_RESOLUTION/4) {
        return sin_samples[x];
    } else if (x <= SIN_INPUT_RESOLUTION/2) {
        return sin_samples[SIN_INPUT_RESOLUTION/2 - x];
    } else if (x <= SIN_INPUT_RESOLUTION/2+SIN_INPUT_RESOLUTION/4){
        return - sin_samples[x - SIN_INPUT_RESOLUTION/2];
    } else {
        return - sin_samples[SIN_INPUT_RESOLUTION - x];
    }
}

int16_t discrete_sin(int16_t x)
{
    x = x % SIN_INPUT_RESOLUTION;
    if( x>=0 ){
        return discrete_sin_positive(x);
    }else{
        return - discrete_sin_positive(-x);
    }
}

int16_t discrete_cos(int16_t x){
    return discrete_sin( x + SIN_INPUT_RESOLUTION/4 );
}

float sin_t(float turn){
    if( turn < 0 ){
        return - discrete_sin( (int16_t) (-turn*SIN_INPUT_RESOLUTION) )/(1.0*SIN_INPUT_RESOLUTION);
    }else{
        return discrete_sin( (int16_t) (turn*SIN_INPUT_RESOLUTION) )/(1.0*SIN_INPUT_RESOLUTION);
    }
}

float cos_t(float turn){
    return sin_t( turn + .25 );
}

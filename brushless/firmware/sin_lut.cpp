#include "sin_lut.h"

#include "sin_samples.h"

uint16_t sin_lut(uint16_t x)
{
    if (x < 0 || x >= 8192) {
        return 0;
    }

    if (x <= 2048) {
        return samples[x];
    } else if (x <= 4096) {
        return samples[4096 - x];
    } else if (x <= 6144) {
        return 16384 - samples[x - 4096];
    } else {
        return 16384 - samples[8192 - x];
    }
}

int16_t discrete_sin(int16_t x)
{
    x = x % 8192;
    if( x<0 ){
        x = - x;
    }
    if (x <= 2048) {
        return sin_samples[x];
    } else if (x <= 4096) {
        return sin_samples[4096 - x];
    } else if (x <= 6144){
        return - sin_samples[x - 4096];
    } else {
        return - sin_samples[ 8192 - x];
    }
}

int16_t discrete_cos(int16_t x){
    return discrete_sin( x + 4096 );
}

float sin_t(float turn){
    if( turn < 0 ){
        return - discrete_sin( (int16_t) (-turn*8192) )/8192.0;
    }else{
        return discrete_sin( (int16_t) (turn*8192) )/8192.0;
    }
}

float cos_t(float turn){
    return sin_t( turn + .25 );
}

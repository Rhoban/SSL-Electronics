#include "sin_lut.h"

#include "sin_samples.h"

uint16_t sin_lut(uint16_t x)
{
    if (x < 0 || x >= 8192) {
        return 0;
    }

    if (x < 2048) {
        return samples[x];
    } else if (x < 4096) {
        return samples[4095 - x];
    } else if (x < 6144) {
        return 16384 - samples[x - 4096];
    } else {
        return 16384 - samples[8192 - x];
    }
}


float sin_t(float turn){
    const int t1 = turn*8191;
    if(t1 <= 0 ){
        return - sin_lut( -t1 )/16384.0;
    }else{
        return sin_lut( t1 )/16384.0;
    }
}

float cos_t(float turn){
    return sin_t( turn + .25 );
}

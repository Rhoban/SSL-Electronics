#include "sin_lut.h"
#include <math.h>
#include <assert.h>
#include <stdio.h>

int main(){
  for( int i=-2*SIN_INPUT_RESOLUTION-1; i<2*SIN_INPUT_RESOLUTION+1; i++ ){
    int16_t res = discrete_sin(i);
    int16_t expected = (
        (int)( 
        SIN_OUPUT_MAX * sin((2.0*M_PI*i)/SIN_INPUT_RESOLUTION)
      )
    );
    assert( res == expected );
  }
  for( int i=-8*SIN_INPUT_RESOLUTION-1; i<8*SIN_INPUT_RESOLUTION+1; i++ ){
    int16_t res = discrete_cos(i);
    int16_t expected = (
        (int)( 
        SIN_OUPUT_MAX * cos((2.0*M_PI*i)/SIN_INPUT_RESOLUTION)
      )
    );
    assert( res == expected );
  }
}

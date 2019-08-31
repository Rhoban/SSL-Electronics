/*
    This file is part of SSL-Electronics.

    Copyright 2019 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

    SSL-Electronics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SSL-Electronics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with SSL-Electronics.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <sin_table.h>
#include <assert.h>
#include <math.h>
#include <tools.h>

void test_sin_cos_table(){
  int i = 0;
  const int N = 1024;
  const int M = 16;
  //const float error = 1.0/1024;
  const float error = 2.0/SIN_RESOLUTION;
  for( i = -M*N; i<=M*N; i++){
      //DEBUG----");  
      assert(
        fabs( sin_table( ((2*M_PI)*i)/N ) - sin( ((2*M_PI)*i)/N ) ) < error
      );
      assert(
        fabs( cos_table( ((2*M_PI)*i)/N ) - cos( ((2*M_PI)*i)/N ) ) < error
      );
  }
  {
    float v = M_PI_2;
    assert(
      fabs( sin_table( v ) - sin( v ) ) < error
    );
    assert(
      fabs( cos_table( v ) - cos( v ) ) < error
    );
  }
  {
    float v = -M_PI_2;
    assert(
      fabs( sin_table( v ) - sin( v ) ) < error
    );
    assert(
      fabs( cos_table( v ) - cos( v ) ) < error
    );
  }
}

void test_mod_2_pi(){
  int i = 0;
  const int N = 1024;
  const float error = 1.0/(128*N);
  for(i = 0; i<N; i++){
      float val = (2*M_PI)*i/N;
      assert(
        fabs(mod_2_pi(val) - val) < error
      );
      assert(
        fabs(mod_2_pi((2*M_PI)+val) - val) < error
      );
      assert(
        fabs(mod_2_pi(2*(2*M_PI)+val) - val) < error
      );
      assert(
        fabs(mod_2_pi(-(2*M_PI)+val) - val) < error
      );
      assert(
        fabs(mod_2_pi(-2*(2*M_PI)+val) - val) < error
      );
  }
}

int main(){
  test_sin_cos_table();
  test_mod_2_pi();
}

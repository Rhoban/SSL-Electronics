/*
 * Copyright (C) 2019  Adrien Boussicault <adrien.boussicault@labri.fr>
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include <filter.h>
#include <assert.h>
#include <math.h>

void test_shift_butterworth_3_1256(){
  float sample = 0.0;
  butterworth_3_data_t data_1;
  butterworth_3_data_t data_2;
  init_butterworth_3_pulsation_1256_rad_s(&data_1, 0.1);
  init_butterworth_3_pulsation_1256_rad_s(&data_2, 2.0);

  float t=0;

  float te = 1/8000.0;  
  float f = 200;

  for( int i =0; i<4000*8000/200; i++ ){
    t+= te;
    sample = sin( f*t );
    update_butterworth_3_pulsation_1256_rad_s(sample, &data_1);
    update_butterworth_3_pulsation_1256_rad_s(sample, &data_2);
    assert(
      fabs( get_filtered_data(&data_1)-get_filtered_data(&data_1)) < 0.0000001
    );
  }
}

#define SAMPLE_FREQUENCE 8000
void test_pulsation( float w, float module, float delay ){
  float sample = 0.0;
  butterworth_3_data_t data;
  init_butterworth_3_pulsation_1256_rad_s(&data,2.0);

  float t=0;

  float max=0;
  float fe = SAMPLE_FREQUENCE;
  for( int i =0; i< 1000*2*M_PI*fe/w; i++ ){
    t+= (1/fe);
    sample = sin( w*t );
    update_butterworth_3_pulsation_1256_rad_s(sample, &data);
    if(t>delay){
      max=fmax( fabs(get_filtered_data(&data)), max);
    }
  }
  // DEBUG( "max : " << max );
  assert( max < module );
}

void test_module_butterworth_3_1256(){
  float delay = 65.0/SAMPLE_FREQUENCE;
  test_pulsation( 500.0, 0.9981, delay );
  test_pulsation( 1000.0, 0.8930188775722774, delay );
  test_pulsation( 1256.6370614359173, 0.707106781186548, delay );
  test_pulsation( 2000.0, 0.2407541002219644, delay );
  test_pulsation( 3000.0, 0.07329865693978056, delay );
  test_pulsation( 4000.0, 0.030991382868835193, delay );
}

void test_null_butterworth_3_1256(){
  float sample = 0.0;
  butterworth_3_data_t data;
  init_butterworth_3_pulsation_1256_rad_s(&data, 0.5);
  assert( 0.0 == get_filtered_data(&data) );

  update_butterworth_3_pulsation_1256_rad_s(sample, &data);
  assert( 0.0 == get_filtered_data(&data) );

  update_butterworth_3_pulsation_1256_rad_s(sample, &data);
  assert( 0.0 == get_filtered_data(&data) );
}

void test_butterworth_3_1256(){
  test_shift_butterworth_3_1256();
  test_null_butterworth_3_1256();
  test_module_butterworth_3_1256();
}

int main(){
  test_butterworth_3_1256();
}

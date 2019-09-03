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
#include <frequence_definitions.h>

//
// butterworth_3_pulsation_1256
// Those coeficients were  computed with the script :
// butterworth_coeeficients.py
//
// If you want to compute new coefficients, modify the frequence `fe` of that 
// script, and then run the script.
//
// To run this script install sagmath, matplotlib and scipt. 
// Then, start sage in console : sage
// On the sage console, just type : attach("butterworth_coeeficients.py")
//
#if ENCODER_FREQ == 8000
  #define D0 2414.790782274951f
  #define D1 -6488.057607935079f
  #define D2 +5845.602032624118f
  #define D3 -1764.3352069639898f
#elif ENCODER_FREQ == 9000
  #define D0 +3378.9197587333133
  #define D1 -9195.466129670838
  #define D2 +8380.764542167903
  #define D3 -2556.218171230377
#elif ENCODER_FREQ == 9600
  #define D0 +4065.2074593339344
  #define D1 -11133.615353259444
  #define D2 +10205.839324811657
  #define D3 -3129.4314308861494
#elif ENCODER_FREQ == 10000
  #define D0 +4570.878710980003
  #define D1 -12566.100342043117 
  #define D2 +11558.88850561974
  #define D3 -3555.666874556626
#else
  _Static_assert(
    false,
    "You need to compute new coefficients for the butterworth filter."
  );
#endif


void reset_filter(butterworth_3_data_t * data){
  data->theta_shift = 0.0;
  data->theta_out[0] = 0.0;
  data->theta_out[1] = 0.0;
  data->theta_out[2] = 0.0;
  data->theta_out[3] = 0.0;
  data->theta_in[0] = 0.0;
  data->theta_in[1] = 0.0;
  data->theta_in[2] = 0.0;
  data->theta_in[3] = 0.0;
}

void init_butterworth_3_pulsation_1256_rad_s(
  butterworth_3_data_t * data, float limit
){
  reset_filter(data);
  data->frame_limit = limit;
}

float get_filtered_data(const butterworth_3_data_t * data){
  return data->theta_out[0] + data->theta_shift;
}

float get_shift(const butterworth_3_data_t * data){
  return data->theta_shift;
}

void update_butterworth_3_pulsation_1256_rad_s(
  float sample, butterworth_3_data_t * data
){
    const float d[4] = {D0, -D1/D0, -D2/D0, -D3/D0};
    const float n[4] = {1.0f/D0, 3.0f/D0, 3.0f/D0, 1.0f/D0};

    data->theta_in[3] = data->theta_in[2];
    data->theta_in[2] = data->theta_in[1];
    data->theta_in[1] = data->theta_in[0];
    data->theta_in[0] = sample - data->theta_shift;

    data->theta_out[3] = data->theta_out[2];
    data->theta_out[2] = data->theta_out[1];
    data->theta_out[1] = data->theta_out[0];
    float num = (
        n[0] * data->theta_in[0] +
        n[1] * data->theta_in[1] +
        n[2] * data->theta_in[2] +
        n[3] * data->theta_in[3]
    );

    float den = (
        d[1] * data->theta_out[1] +
        d[2] * data->theta_out[2] +
        d[3] * data->theta_out[3]
    );
    data->theta_out[0] = num + den;

    if( data->theta_out[0] >= data->frame_limit ){
        data->theta_in[3] -= data->frame_limit;
        data->theta_in[2] -= data->frame_limit;
        data->theta_in[1] -= data->frame_limit;
        data->theta_in[0] -= data->frame_limit;
        data->theta_out[3] -= data->frame_limit;
        data->theta_out[2] -= data->frame_limit;
        data->theta_out[1] -= data->frame_limit;
        data->theta_out[0] -= data->frame_limit;
        data->theta_shift += data->frame_limit;
    }
    if( data->theta_out[0] <= -data->frame_limit ){
        data->theta_in[3] += data->frame_limit;
        data->theta_in[2] += data->frame_limit;
        data->theta_in[1] += data->frame_limit;
        data->theta_in[0] += data->frame_limit;
        data->theta_out[3] += data->frame_limit;
        data->theta_out[2] += data->frame_limit;
        data->theta_out[1] += data->frame_limit;
        data->theta_out[0] += data->frame_limit;
        data->theta_shift -= data->frame_limit;
    }
}

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

void init_butterworth_3_pulsation_1256_rad_s(
  butterworth_3_data_t * data, float limit
){
  data->theta_shift = 0.0;
  data->theta_out[0] = 0.0;
  data->theta_out[1] = 0.0;
  data->theta_out[2] = 0.0;
  data->theta_out[3] = 0.0;
  data->theta_in[0] = 0.0;
  data->theta_in[1] = 0.0;
  data->theta_in[2] = 0.0;
  data->theta_in[3] = 0.0;
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
    const float d_0 = 2414.790782274951f;
    const float d[4] = {
        d_0, +6488.057607935079f/d_0, -5845.602032624118f/d_0,
        +1764.3352069639898f/d_0
    };
    const float n[4] = {1.0f/d_0, 3.0f/d_0, 3.0f/d_0, 1.0f/d_0};

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

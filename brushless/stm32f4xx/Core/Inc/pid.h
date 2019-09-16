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

#pragma once

/**
 * The Proportional and Integral constants of control.
 */ 
typedef struct {
  float p; /**< The proportional constant of the PI control. */
  float i; /**< The integral constant of the PI control. */
  float limit; /**< The output limit of the PI control. */
  float i_limit; /**< The limit for the integral value of the PI control. */
} Pi_constants;

/**
 * The structure containing the proportional error, the integral error 
 * and the output computed by a PI control.
 */
typedef struct {
  float e; /**< The error */
  float p; /**< The proportional error of the PI control. */
  float i; /**< The integral error of the PI control. */
  float output; /**< The ouput of the PI control */
} Pi;

/**
 * This function fills the structure of the PI constnats of the PID.
 * 
 * @param k The adress of PI constants sttructure.
 * @param kp The proportional constant.
 * @param ki The integral constant.
 * @param limit The output limit of the control.
 * @param i_limit The integral limit of the control.
 */
inline void init_pi_k(
  Pi_constants* k, const float kp, const float ki,
  const float limit, const float i_limit
){
  k->p = kp;
  k->i = ki;
  k->limit = limit;
  k->i_limit = i_limit;
}

/**
 * This function initializes the structure that contains the computed errors and
 * outuput of a PI Control.
 */
inline void init_pi(Pi* pi){
  pi->p = 0;
  pi->i = 0;
  pi->output = 0;
}

/**
 * This function sets to 0 all the errors of the PI control.
 */
inline void reset_pi(Pi* pi){
  pi->i = 0.0;
}

/**
 * This function initializes the structure that contains the computed errors and
 * outuput of a PI Control.
 */
void init_pi(Pi* pi);

inline void limit_value(float * value, const float limit){
  if( limit <= 0.0 ) return;
  if(*value > limit){
    *value = limit;
  }else if( *value < -limit ){
    *value = -limit;
  }
}

/**
 * This function updates the errors and the ouput of the PI control using the 
 * following formula : 
 *
 * p_error = k->p * error
 * i_error = limit_value(
 *   i_error + k->i * error * dt,
 *   k->i_limit
 * ) 
 * 
 * output = limit_value( p_error + i_error, k->limit ) 
 */
inline void update_pi(
  const Pi_constants* k, Pi* pi, const float error, const float dt
){
  pi->e = error;
  pi->p = k->p * error;
  if( k->i > 0 ){
    pi->i += ( error * k->i * dt ); 
    limit_value(&(pi->i), k->i_limit);
  }else{
    pi->i = 0;
  }
  pi->output = pi->p + pi->i;
  limit_value(&(pi->output), k->limit);
}

/**
 * This function returns the ouput of the PI control.
 */
inline float get_output(Pi* pi){
  return pi->output;
}

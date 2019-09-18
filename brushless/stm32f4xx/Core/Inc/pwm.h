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


// 
// PWN and generated Update event 
//
// with Center-Aligned counter (TIM1) 
// ----------------------------------
//
// CNT           :  |0 |1 |2 |3 |2 |1 |0
// Up/Down Event :  |        |        | 
// CA period :      |                 |
//
// Reload value (TIMxARR) = half_period
// CA perdiod = 2 * half_period
//
// with a Counter-up (TIM2 and TIM3)
// ---------------------------------
//
// CNT           :  |0 |1 |2 |0 |1 |2 |0
// Update Event  :  |        |        | 
//
// Reload value (TIMxARR) = period-1
//
// with a Counter-down 
// -------------------
//
// CNT           :  |2 |1 |0 |2 |1 |0 |2
// Update Event  :  |        |        | 
//
// Reload value (TIMxARR) = period-1
#define CA_RELOAD(period) (period/2)
#define CU_RELOAD(period) (period-1)
#define CD_RELOAD(period) (period-1)

// 
// Duty-cycle of an upcounting pwm
// --------------------------------
//
// if  TIMx_CNT < TIMx_CCRx
//   PWM = High
// else
//   PWM = LOW
//
// example : TIMx_CCRx = 3
//           TIMxARRx = 6
// |0 1 2 3 4 5 6| 0 1 2 3 4 5 6|
//  H H H L L L L  H H H L L L L
//
// reload = period - 1
// duty = TIMx_CCRx / period 
//      = TIMx_CCRx / ( TIMxARR+1 )
//      
// 
// Duty-cycle of a downcounting pwm
// --------------------------------
//
// if  TIMx_CNT > TIMx_CCRx
//   PWM = LOW
// else
//   PWM = HIG
//
// example : TIMx_CCRx = 3
//           TIMxARRx = 6
// |6 5 4 3 2 1 0| 6 5 4 3 2 1 0|
//  L L L H H H H  L L L H H H H
//
// (0% of PWM is impossible)
//
// reload = period - 1
// duty = (TIMx_CCRx+1) / period 
//      = (TIMx_CCRx+1) / ( TIMxARR+1 )
//      
// Duty-cycle of a center-aligned pwm
// ----------------------------------
//
// if counter is up 
//   if  TIMx_CNT < TIMx_CCRx
//     PWM = High
//   else
//     PWM = LOW
// else
//   if TIMx_CNT > TIMx_CCRx
//     PWM = LOW
//   else
//     PWM = HIG
//
// CCRx = 2  and  ARR = 8
// |0 1 2 3 4 5 6 7 |8 7 6 5 4 3 2 1 |0
// |H H L L L L L L  L L L L L L H H |H
//
// Reload value (TIMxARR) = half_period
// CA perdiod = 2 * half_period
//
// duty = (2*TIMx_CCRx)/(2*half_period)
//      = TIMx_CCRx/half_period

/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Define to prevent recursive inclusion
#ifndef DEFINES_H
#define DEFINES_H

#include "stm32f1xx_hal.h"
#include "config.h"

//#define PHASE_CURR_mA_CNT 50 //mA per bit
#define DC_VOLT_uV_CNT 14431 //uV per bit

#define DELAY_TIM_FREQUENCY_US 1000000

#define MILLI_R (R * 1000)
#define MILLI_PSI (PSI * 1000)
#define MILLI_V (V * 1000)

#define NO 0
#define YES 1
#define ABS(a) (((a) < 0) ? -(a) : (a))
#define LIMIT(x, lowhigh) (((x) > (lowhigh)) ? (lowhigh) : (((x) < (-lowhigh)) ? (-lowhigh) : (x)))
#define SAT(x, lowhigh) (((x) > (lowhigh)) ? (1.0f) : (((x) < (-lowhigh)) ? (-1.0f) : (0.0f)))
#define SAT2(x, low, high) (((x) > (high)) ? (1.0f) : (((x) < (low)) ? (-1.0f) : (0.0f)))
#define STEP(from, to, step) (((from) < (to)) ? (MIN((from) + (step), (to))) : (MAX((from) - (step), (to))))
#define DEG(a) ((a)*M_PI / 180.0f)
#define RAD(a) ((a)*180.0f / M_PI)
#define SIGN(a) (((a) < 0) ? (-1) : (((a) > 0) ? (1) : (0)))
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define IN_RANGE(x, low, high) (((x) >= (low)) && ((x) <= (high)))
#define SCALE(value, high, max) MIN(MAX(((max) - (value)) / ((max) - (high)), 0.0f), 1.0f)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN3(a, b, c) MIN(a, MIN(b, c))
#define MAX3(a, b, c) MAX(a, MAX(b, c))
#define ARRAY_LEN(x) (uint32_t)(sizeof(x) / sizeof(*(x)))
#define MAP(x, in_min, in_max, out_min, out_max) (((((x) - (in_min)) * ((out_max) - (out_min))) / ((in_max) - (in_min))) + (out_min))

/*
typedef struct {
  uint16_t curr_a;
  uint16_t curr_b;
  uint16_t curr_c;
  uint16_t volt_a;
  uint16_t volt_b;
  uint16_t volt_c;
  uint16_t ntc;
  uint16_t vbat;
*/

/*
typedef struct {
  uint16_t rlA;
  uint16_t rlB;
  uint16_t rlC;
  uint16_t voltPhA;
  uint16_t voltPhB;
  uint16_t voltPhC;
  uint16_t temp;
  uint16_t batt1;
} adc_buf_t;
*/

typedef struct {
 // XIAOMI firmware
  uint16_t curr_a;
  uint16_t curr_b;
  uint16_t curr_c;
  uint16_t vbat;
  uint16_t temp;
  uint16_t volt_a;
  uint16_t volt_b;
  uint16_t volt_c;
} adc_buf_t;


typedef struct {
  int32_t curr_a;
  int32_t curr_b;
  int32_t curr_c;

  int16_t curr_a_cnt;
  int16_t curr_b_cnt;
  int16_t curr_c_cnt;

  int32_t curr_dc;
  int16_t curr_dc_raw;
} analog_t;

extern analog_t analog;


//#define PHASE_CURR_mA_CNT 50 //mA per bit
#define DC_VOLT_uV_CNT 14431 //uV per bit


#endif // DEFINES_H


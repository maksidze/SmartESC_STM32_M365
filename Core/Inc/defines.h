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

#if KX
#define LEFT_HALL_U_PIN GPIO_PIN_5
#define LEFT_HALL_V_PIN GPIO_PIN_6
#define LEFT_HALL_W_PIN GPIO_PIN_7

#define LEFT_HALL_U_PORT GPIOB
#define LEFT_HALL_V_PORT GPIOB
#define LEFT_HALL_W_PORT GPIOB
#else
#define LEFT_HALL_U_PIN HALL_A_Pin
#define LEFT_HALL_V_PIN HALL_B_Pin
#define LEFT_HALL_W_PIN HALL_C_Pin

#define LEFT_HALL_U_PORT HALL_A_GPIO_Port
#define LEFT_HALL_V_PORT HALL_B_GPIO_Port
#define LEFT_HALL_W_PORT HALL_C_GPIO_Port
#endif

#if KX
#define LEFT_TIM TIM8
#define LEFT_TIM_U CCR1
#define LEFT_TIM_UH_PIN GPIO_PIN_6
#define LEFT_TIM_UH_PORT GPIOC
#define LEFT_TIM_UL_PIN GPIO_PIN_7
#define LEFT_TIM_UL_PORT GPIOA
#define LEFT_TIM_V CCR2
#define LEFT_TIM_VH_PIN GPIO_PIN_7
#define LEFT_TIM_VH_PORT GPIOC
#define LEFT_TIM_VL_PIN GPIO_PIN_0
#define LEFT_TIM_VL_PORT GPIOB
#define LEFT_TIM_W CCR3
#define LEFT_TIM_WH_PIN GPIO_PIN_8
#define LEFT_TIM_WH_PORT GPIOC
#define LEFT_TIM_WL_PIN GPIO_PIN_1
#define LEFT_TIM_WL_PORT GPIOB
#else
#define LEFT_TIM TIM1
#define LEFT_TIM_U CCR1
//#define LEFT_TIM_UH_PIN GPIO_PIN_6
//#define LEFT_TIM_UH_PORT GPIOC
//#define LEFT_TIM_UL_PIN GPIO_PIN_7
//#define LEFT_TIM_UL_PORT GPIOA
#define LEFT_TIM_V CCR2
//#define LEFT_TIM_VH_PIN GPIO_PIN_7
//#define LEFT_TIM_VH_PORT GPIOC
//#define LEFT_TIM_VL_PIN GPIO_PIN_0
//#define LEFT_TIM_VL_PORT GPIOB
#define LEFT_TIM_W CCR3
//#define LEFT_TIM_WH_PIN GPIO_PIN_8
//#define LEFT_TIM_WH_PORT GPIOC
//#define LEFT_TIM_WL_PIN GPIO_PIN_1
//#define LEFT_TIM_WL_PORT GPIOB
#endif



#define PHASE_CURR_mA_CNT 50 //mA per bit
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
  uint16_t volt_a;
  uint16_t volt_b;
  uint16_t volt_c;
  uint16_t temp;
  uint16_t vbat;
} adc_buf_t;

typedef struct {
  int32_t curr_a;
  int32_t curr_b;
  int32_t curr_c;

  int16_t curr_a_cnt;
  int16_t curr_b_cnt;
  int16_t curr_c_cnt;

  int32_t curr_dc;
} analog_t;

extern analog_t analog;


#define PHASE_CURR_mA_CNT 50 //mA per bit
#define DC_VOLT_uV_CNT 14431 //uV per bit


// Define I2C, Nunchuk, PPM, PWM functions
void PPM_Init(void);
void PPM_ISR_Callback(void);
void PWM_Init(void);
void PWM_ISR_CH1_Callback(void);
void PWM_ISR_CH2_Callback(void);

// Sideboard definitions
#define LED1_SET            (0x01)
#define LED2_SET            (0x02)
#define LED3_SET            (0x04)
#define LED4_SET            (0x08)
#define LED5_SET            (0x10)
#define SENSOR1_SET         (0x01)
#define SENSOR2_SET         (0x02)
#define SENSOR_MPU          (0x04)


#endif // DEFINES_H


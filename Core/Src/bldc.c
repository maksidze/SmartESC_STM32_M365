/*
 * This file implements FOC motor control.
 * This control method offers superior performanace
 * compared to previous cummutation method. The new method features:
 * ► reduced noise and vibrations
 * ► smooth torque output
 * ► improved motor efficiency -> lower energy consumption
 *
 * Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
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

#include "stm32f1xx_hal.h"
#include "defines.h"
#include "config.h"
#include "util.h"
#include "main.h"

// Matlab includes and defines - from auto-code generation
// ###############################################################################
#include "BLDC_controller.h"           /* Model's header file */
#include "rtwtypes.h"

extern RT_MODEL *const rtM_Left;

extern DW rtDW_Left; /* Observable states */
extern ExtU rtU_Left; /* External inputs */
extern ExtY rtY_Left; /* External outputs */

// ###############################################################################

#if KX
static int16_t pwm_margin = 110;        /* This margin allows to always have a window in the PWM signal for proper Phase currents measurement */
                                        /* official firmware value */
#else
static int16_t pwm_margin = 10; // Xiaomi firmware value
#endif

analog_t analog;

extern uint8_t ctrlModReq;
static int16_t curDC_max = (I_DC_MAX * A2BIT_CONV);
//int16_t curL_phaA = 0, curL_phaB = 0, curL_DC = 0;
//int16_t curR_phaB = 0, curR_phaC = 0, curR_DC = 0;

volatile int pwml = 0;
volatile int pwmr = 0;

extern volatile adc_buf_t adc_buffer;

uint8_t enable = 0;        // initially motors are disabled for SAFETY
static uint8_t enableFin = 0;

static const uint16_t pwm_res = 64000000 / 2 / PWM_FREQ; // = 2000

static uint16_t offsetcount = 0;
static int offset_curr_a = 2000;
static int offset_curr_b = 2000;
static int offset_curr_c = 2000;
static int offset_volt_a = 0;
static int offset_volt_b = 0;
static int offset_volt_c = 0;

uint8_T errCodeLeft;
int16_T motSpeedLeft;

int16_t voltageTimer = 0;
int16_t batVoltage = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE;
static int32_t batVoltageFixdt = (400 * BAT_CELLS * BAT_CALIB_ADC)
		/ BAT_CALIB_REAL_VOLTAGE << 16; // Fixed-point filter output initialized at 400 V*100/cell = 4 V/cell converted to fixed-point

// =================================
// DMA interrupt frequency =~ 16 kHz
// =================================
void DMA1_Channel1_IRQHandler(void) {

	DMA1->IFCR = DMA_IFCR_CTCIF1;
	// HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);
	// HAL_GPIO_TogglePin(LED_PORT, LED_PIN);

#define ENABLE_LOOP 1
#if ENABLE_LOOP

	if (offsetcount < 1000) {  // calibrate ADC offsets
		offsetcount++;
		offset_curr_a = (adc_buffer.curr_a + offset_curr_a) / 2;
		offset_curr_b = (adc_buffer.curr_b + offset_curr_b) / 2;
		offset_curr_c = (adc_buffer.curr_c + offset_curr_c) / 2;
		offset_volt_a = (adc_buffer.volt_a + offset_volt_a) / 2;
		offset_volt_b = (adc_buffer.volt_b + offset_volt_b) / 2;
		offset_volt_c = (adc_buffer.volt_c + offset_volt_c) / 2;
		return;
	}

	// Get motor currents
	analog.curr_a_cnt = (offset_curr_a - adc_buffer.curr_a);
	analog.curr_b_cnt = (offset_curr_b - adc_buffer.curr_b);
	analog.curr_c_cnt = (offset_curr_c - adc_buffer.curr_c);
	analog.curr_a = analog.curr_a_cnt * PHASE_CURR_mA_CNT;
	analog.curr_b = analog.curr_b_cnt * PHASE_CURR_mA_CNT;
	analog.curr_c = analog.curr_c_cnt * PHASE_CURR_mA_CNT;

	// compute DC current
	static int32_t filter_buffer;
	filtLowPass32(
			(analog.curr_a_cnt + analog.curr_b_cnt + analog.curr_c_cnt) / 3, 20,
			&filter_buffer);
	analog.curr_dc = ((filter_buffer >> 16) * (PHASE_CURR_mA_CNT * 173) / 100);

	// compute DC voltage
	voltageTimer++;
	if (voltageTimer % 1000 == 0) { // Filter battery voltage at a slower sampling rate
		filtLowPass32(adc_buffer.vbat, BAT_FILT_COEF, &batVoltageFixdt);
		batVoltage = (int16_t) (batVoltageFixdt >> 16); // convert fixed-point to integer
	}

	// Disable PWM when current limit is reached (current chopping)
	// This is the Level 2 of current protection. The Level 1 should kick in first given by I_MOT_MAX
	if (ABS(analog.curr_dc) > curDC_max || enable == 0) {
		LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
	} else {
		LEFT_TIM->BDTR |= TIM_BDTR_MOE;
	}

	// ############################### MOTOR CONTROL ###############################

	int ul, vl, wl;
	static boolean_T OverrunFlag = false;

	/* Check for overrun */
	if (OverrunFlag) {
		return;
	}
	OverrunFlag = true;

	/* Make sure to stop BOTH motors in case of an error */
	enableFin = enable && !rtY_Left.z_errCode;

	// ========================= LEFT MOTOR ============================
	// Get hall sensors values
	uint8_t hall_ul = !(LEFT_HALL_U_PORT->IDR & LEFT_HALL_U_PIN);
	uint8_t hall_vl = !(LEFT_HALL_V_PORT->IDR & LEFT_HALL_V_PIN);
	uint8_t hall_wl = !(LEFT_HALL_W_PORT->IDR & LEFT_HALL_W_PIN);

	/* Set motor inputs here */
	rtU_Left.b_motEna = enableFin;
	rtU_Left.z_ctrlModReq = ctrlModReq;
	rtU_Left.r_inpTgt = pwml;
	rtU_Left.b_hallA = hall_ul;
	rtU_Left.b_hallB = hall_vl;
	rtU_Left.b_hallC = hall_wl;
	rtU_Left.i_phaAB = analog.curr_a_cnt;
	rtU_Left.i_phaBC = analog.curr_b_cnt;
	rtU_Left.i_DCLink = analog.curr_dc;
	// rtU_Left.a_mechAngle   = ...; // Angle input in DEGREES [0,360] in fixdt(1,16,4) data type. If `angle` is float use `= (int16_t)floor(angle * 16.0F)` If `angle` is integer use `= (int16_t)(angle << 4)`

	/* Step the controller */
#ifdef MOTOR_LEFT_ENA
	BLDC_controller_step(rtM_Left);
#endif

	/* Get motor outputs here */
	ul = rtY_Left.DC_phaA;
	vl = rtY_Left.DC_phaB;
	wl = rtY_Left.DC_phaC;
	errCodeLeft = rtY_Left.z_errCode;
	motSpeedLeft = rtY_Left.n_mot;
	// motAngleLeft = rtY_Left.a_elecAngle;

	/* Apply commands */
	LEFT_TIM->LEFT_TIM_U = (uint16_t) CLAMP(ul + pwm_res / 2, pwm_margin,
			pwm_res - pwm_margin);
	LEFT_TIM->LEFT_TIM_V = (uint16_t) CLAMP(vl + pwm_res / 2, pwm_margin,
			pwm_res - pwm_margin);
	LEFT_TIM->LEFT_TIM_W = (uint16_t) CLAMP(wl + pwm_res / 2, pwm_margin,
			pwm_res - pwm_margin);

	// =================================================================

	/* Indicate task complete */
	OverrunFlag = false;

#endif

	// ###############################################################################

}

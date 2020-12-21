/**
 * This file is part of the hoverboard-firmware-hack project.
 *
 * Copyright (C) 2020-2021 Emanuel FERU <aerdronix@gmail.com>
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
#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>


// Rx Structure USART
typedef struct {
	uint16_t start;
	int16_t steer;
	int16_t speed;
	uint16_t checksum;
} SerialFromDisplayToEsc;

// Tx Struture USART
typedef struct {
	uint16_t start;
	int16_t cmd1;
	int16_t cmd2;
	int16_t currDC;
	int16_t speedMeas;
	int16_t batVoltage;
	int16_t boardTemp;
	int16_t currPhA;
	int16_t speedMotor;
	uint16_t checksum;
} SerialFromEscToDisplay;

// Initialization Functions
void Input_Lim_Init(void);
void Input_Init(void);
void UART_DisableRxErrors(UART_HandleTypeDef *huart);

// General Functions
void calcAvgSpeed(void);
int addDeadBand(int16_t u, int16_t type, int16_t deadBand, int16_t in_min,
		int16_t in_mid, int16_t in_max, int16_t out_min, int16_t out_max);
int checkInputType(int16_t min, int16_t mid, int16_t max);
void adcCalibLim(void);
void updateCurSpdLim(void);
void saveConfig(void);
void standstillHold(void);
void electricBrake(uint16_t speedBlend, uint8_t reverseDir);
void cruiseControl(uint8_t button);

// Poweroff Functions
void poweroff(void);
void poweroffPressCheck(void);

// Read Functions
void readInput(void);
void readCommand(void);
void usart3_rx_check(void);
void usart_process_command(SerialFromDisplayToEsc *command_in,
		SerialFromDisplayToEsc *command_out, uint8_t usart_idx);

// Filtering Functions
void filtLowPass32(int32_t u, uint16_t coef, int32_t *y);
void rateLimiter16(int16_t u, int16_t rate, int16_t *y);
void mixerFcn(int16_t rtu_speed, int16_t rtu_steer, int16_t *rty_speedL);

#endif


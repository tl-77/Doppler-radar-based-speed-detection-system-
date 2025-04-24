/*
 * comparator.c
 *
 *  Created on: Dec 13, 2024
 *      Author: yitin
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "lcd16x2_v2.h" // Custom library for LCD control
#include "comparator.h" // Comparator-related header file
#include "lcd.h"  // LCD display helper functions

int ticks;                // Variable to store the number of half-cycles detected
float period;             // Variable to store the calculated period of the input signal
float freqComp;           // Variable to store the calculated frequency of the input signal
float speedC = 0.0;       // Variable to store the calculated speed (derived from frequency)
uint8_t comp_val;         // Variable to store comparator output
float total_period = 0.065535; // Fixed time period for measurement (65.535 ms ~ 65535us)
float Vref = 2.3; //reference voltage


/* UART Buffer for Debugging */
int uart_buf_len2;
char uart_buf2[50];

/*
 * HAL_COMP_TriggerCallback
 *
 * This function is triggered every time the comparator detects a crossing (e.g., rising or falling edge).
 * It increases the tick count, which corresponds to the number of half-cycles of the input signal.
 *
 * @param hcomp: Pointer to the comparator handle
 */
void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp){
	ticks++; // Increment the tick count (half-cycles passed)
}

/*
 * HAL_TIM_PeriodElapsedCallback
 *
 * This function is triggered at fixed time intervals (timer overflow interrupt).
 * It calculates the period and frequency of the input signal using the tick count and the fixed time period.
 * The frequency is then converted into speed.
 *
 * @param htim16: Pointer to the timer handle (TIM16)
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim16){ //to calculate the period and frequency at a fix time
	if(htim16->Instance == TIM16){  // Verify if the interrupt is from TIM16
		if(ticks){ // Process only if ticks are recorded
			// Calculate the period by dividing the total fixed time with the number of half-cycles
			period = total_period/((double)ticks);
			// Calculate the frequency (frequency = 1 / period)
			freqComp = 1.0/period;
			// Convert frequency to speed using the helper function
			speedC =frequencyToSpeed(freqComp);
			// Reset the tick count for the next measurement cycle
			ticks=0;
		}
	}
}


void compMethod(COMP_HandleTypeDef *hcomp1,TIM_HandleTypeDef *htim16,ADC_HandleTypeDef *hadc2,UART_HandleTypeDef *huart2, DAC_HandleTypeDef *hdac1,UART_HandleTypeDef *huart3){
	// Calculate the DAC reference voltage level based on the desired voltage (Vref)
	int VrefLevel = (int)(Vref * 4095 / 3.3); // Convert reference voltage to a DAC-compatible value

	// Start the DAC (Digital-to-Analog Converter)
	HAL_DAC_Start(hdac1, DAC_CHANNEL_1);

	// Set the calculated reference voltage level in the DAC for conversion
	HAL_DAC_SetValue(hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, VrefLevel);

	// Start the base timer with interrupt for periodic operations
	HAL_TIM_Base_Start_IT(htim16);

	// Start the comparator with interrupt to detect threshold crossings
	HAL_COMP_Start_IT(hcomp1);

	// Prepare a UART message containing the comparator frequency
	uart_buf_len2 = snprintf(uart_buf2, sizeof(uart_buf2), "passing comp freq%.2f", freqComp);

	// Transmit the prepared message via UART to provide feedback or debugging information
	HAL_UART_Transmit(huart2, (uint8_t*)uart_buf2, uart_buf_len2, 100);

	// Update the LCD display with the speed and comparator frequency values
	displayLCD(speedC, freqComp, huart2, huart3);
}

/*
 * comparator.c
 *
 *  Created on: Dec 13, 2024
 *      Author: yitin
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "lcd16x2_v2.h"
#include "comparator.h"
#include "lcd.h"

int ticks; //to store the number of half a cycle passed
float period; //period of the input signal
float freqComp; //frequency of the input signal
float speedC =0.0;
uint8_t comp_val; //to store the comparator output
float total_period = 0.065535; //total time from 0 to 65535 is the fix time (which is 65535us)

int uart_buf_len2;
char uart_buf2[50];


void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp){ //trigger every half a cycle of the input voltage
	ticks++; //increase the number of half a cycle passed
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim16){ //to calculate the period and frequency at a fix time
	if(htim16->Instance == TIM16){
		if(ticks){ //when there is cycle recorded
			period = total_period/((double)ticks); //divide total time with number of cycle passed
			freqComp = 1.0/period; //frequency of input
			speedC =frequencyToSpeed(freqComp);
			ticks=0; //reset the  number of half a cycle is recorded to 0
		}
	}
}


void compMethod(COMP_HandleTypeDef *hcomp1,TIM_HandleTypeDef *htim16,ADC_HandleTypeDef *hadc2,UART_HandleTypeDef *huart2){
	HAL_TIM_Base_Start_IT(htim16);
	HAL_COMP_Start_IT(hcomp1);
	uart_buf_len2 = snprintf(uart_buf2, sizeof(uart_buf2), "passing comp freq%.2f",freqComp);
	HAL_UART_Transmit(huart2, (uint8_t*)uart_buf2, uart_buf_len2, 100);

	//processButton(speedC,freqComp,hadc2);
	displayLCD(speedC,freqComp);
}

/*
 * adc.c
 *
 *  Created on: Dec 13, 2024
 *      Author: yitin
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "adc.h"
#include "lcd.h"
#include "FFT.h"
#include "arm_math.h"
int flag=0;

float fwave=10525000000.0;
struct ADC_param ADC_val={0};
struct FFT_res FFT_val = {0};
float speed =0.0;
float freqADC;

int uart_buf_len1;
char uart_buf1[50];

void ADCmethod(ADC_HandleTypeDef *hadc1,ADC_HandleTypeDef *hadc2,UART_HandleTypeDef *huart2){
	ADC_val.bit=12;
	ADC_val.prescaler=64;
	ADC_val.sampling_time=247.5;
	ADC_val.speed=64000000;					//clock speed on .ioc file
	ADC_val.adc_buf_len=2048;				//buffer length
	uint32_t adc_buf[ADC_val.adc_buf_len];	//buffer array
	ADC_val.adc_buf=adc_buf;
	HAL_ADC_Start_DMA(hadc1, adc_buf,ADC_val.adc_buf_len);

	if (flag == 1) {
		start_FFT(&flag, &ADC_val, &FFT_val);

		flag = 0;
		HAL_ADC_Start_DMA(hadc1, ADC_val.adc_buf, ADC_val.adc_buf_len);
	}
	get_velocity(FFT_val.fdominant, fwave, &speed);
	freqADC=FFT_val.fdominant;
	uart_buf_len1 = snprintf(uart_buf1, sizeof(uart_buf1), "passing adc freq%.2f",freqADC);
	HAL_UART_Transmit(huart2, (uint8_t*)uart_buf1, uart_buf_len1, 100);

	//processButton(speed,freqADC,hadc2);
	displayLCD(speed,freqADC);

}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1) {
	flag=1;
	HAL_ADC_Stop_DMA(hadc1);
}


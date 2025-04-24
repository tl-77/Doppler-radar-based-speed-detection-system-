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
int flag=0; // Flag for triggering FFT processing

float fwave=10525000000.0;

float speed =0.0;
float freqADC;

int uart_buf_len1;
char uart_buf1[50];


// Callback function for ADC conversion complete interrupt
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1) {
	flag=1;
	HAL_ADC_Stop_DMA(hadc1);
}


// Main function to handle ADC processing and velocity calculation
void ADCmethod(ADC_HandleTypeDef *hadc1,ADC_HandleTypeDef *hadc2,UART_HandleTypeDef *huart2,UART_HandleTypeDef *huart3){
	// Structure to hold ADC parameters
	struct ADC_param ADC_val={0};
	// Structure to store FFT results
	struct FFT_res FFT_val = {0};

	ADC_val.bit=12;
	ADC_val.prescaler=64;
	ADC_val.sampling_time=247.5;
	ADC_val.speed=64000000;					//clock speed on .ioc file
	ADC_val.adc_buf_len=2048;				//buffer length
	uint32_t adc_buf[ADC_val.adc_buf_len];	//buffer array
	ADC_val.adc_buf=adc_buf;

	// Start the ADC in DMA mode to continuously transfer data to the adc_buf array
	HAL_ADC_Start_DMA(hadc1, (uint32_t*) adc_buf,ADC_val.adc_buf_len);

	// Check if the flag is set, indicating data is ready for processing
	if (flag == 1) {
		// Perform FFT (Fast Fourier Transform) to analyze the frequency components of the ADC data
		start_FFT(&flag, &ADC_val, &FFT_val);
		// Prepare a UART message with the dominant frequency from the FFT results
		uart_buf_len1 = snprintf(uart_buf1, sizeof(uart_buf1), "adc freq%.2f",FFT_val.fdominant);
		// Reset the flag after processing
		flag = 0;

		// Calculate the velocity based on the dominant frequency and wave properties
		get_velocity(FFT_val.fdominant, fwave, &speed);

		// Store the dominant frequency from the FFT results for further use
		freqADC=FFT_val.fdominant;

		// Prepare a UART message to transmit the calculated frequency
		uart_buf_len1 = snprintf(uart_buf1, sizeof(uart_buf1), "passing adc freq%.2f",freqADC);

		// Transmit the frequency data via UART for debugging or monitoring
		HAL_UART_Transmit(huart2, (uint8_t*)uart_buf1, uart_buf_len1, 100);

		// Update the LCD display with the calculated speed and frequency values
		displayLCD(speed,freqADC,huart2,huart3);
	}
}


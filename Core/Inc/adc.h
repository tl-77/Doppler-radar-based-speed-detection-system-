/*
 * adc.h
 *
 *  Created on: Dec 13, 2024
 *      Author: yitin
 */

#ifndef SRC_ADC_H_
#define SRC_ADC_H_
#include "main.h"

void ADCmethod(ADC_HandleTypeDef *hadc1,ADC_HandleTypeDef *hadc2,UART_HandleTypeDef *huart2,UART_HandleTypeDef *huart3);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1) ;

#endif /* SRC_ADC_H_ */

/*
 * comparator.h
 *
 *  Created on: Dec 13, 2024
 *      Author: yitin
 */

#ifndef SRC_COMPARATOR_H_
#define SRC_COMPARATOR_H_

#include "main.h"

void compMethod(COMP_HandleTypeDef *hcomp1,TIM_HandleTypeDef *htim16,ADC_HandleTypeDef *hadc2,UART_HandleTypeDef *huart2, DAC_HandleTypeDef *hdac1,UART_HandleTypeDef *huart3);
void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim16);


#endif /* SRC_COMPARATOR_H_ */

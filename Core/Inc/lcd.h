/*
 * lcd.h
 *
 *  Created on: Dec 13, 2024
 *      Author: yitin
 */

#ifndef LCD_H_
#define LCD_H_

#include "main.h"

void LCD_Init();
int processButton(ADC_HandleTypeDef *hadc2);
void displayLCD(float speed,float freq,UART_HandleTypeDef *huart2,UART_HandleTypeDef *huart3);
void RS485_EnableTransmit(void) ;
void RS485_EnableReceive(void);
uint8_t ConvertSpeedToBCD(uint8_t speed) ;
void ConvertToBinary(uint8_t num, char* binaryStr);
void RS485_SendSpeed(uint8_t speed,UART_HandleTypeDef *huart2,UART_HandleTypeDef *huart3);
void display7seg(float speed,UART_HandleTypeDef *huart2,UART_HandleTypeDef *huart3);

float frequencyToSpeed (float frequency);
float msToKmh (float speed);
float msToMph(float speed);

#endif /* LCD_H_ */

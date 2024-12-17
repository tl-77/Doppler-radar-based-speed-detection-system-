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
void displayLCD(float speed,float freq);
float frequencyToSpeed (float frequency);
float msToKmh (float speed);
float msToMph(float speed);

#endif /* LCD_H_ */

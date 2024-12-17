/*
 * lcd.c
 *
 *  Created on: Dec 13, 2024
 *      Author: yitin
 */
#include "main.h"
#include "lcd16x2_v2.h"
#include <stdio.h>
#include <string.h>
#include "lcd.h"


#define MS 0
#define KMH 1
#define MPH 2

static uint16_t unit =MS;
uint16_t adc_value;
static char string [20];
char line[20];
uint32_t lastUpdateTime = 0;
static uint8_t displayMode = 0;
//static float speed =0.0;
//float freq; //frequency of the input signal


void LCD_Init(){
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef pinconf;
    pinconf.Pin = GPIO_PIN_0;  // Correct GPIO pin assignment
    pinconf.Mode = GPIO_MODE_OUTPUT_PP;
    pinconf.Pull = GPIO_PULLDOWN;
    pinconf.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &pinconf);

    HAL_NVIC_SetPriority(EXTI0_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

int processButton(ADC_HandleTypeDef *hadc2) {
     static uint32_t lastButtonPressTime = 0;
     //adc_value = 0;

     // Read ADC2 value
     HAL_ADC_Start(hadc2);
     if (HAL_ADC_PollForConversion(hadc2, 10) == HAL_OK) {
         adc_value = HAL_ADC_GetValue(hadc2);
     }
     HAL_ADC_Stop(hadc2);

     // Debounce: Ensure at least 100ms between button presses
     uint32_t currentTime = HAL_GetTick();
     if (currentTime - lastButtonPressTime < 100) {
         return 3; // Ignore this press
     }

     // Determine button action
     if (adc_value > 0 && adc_value < 900) {
         // Up button: Switch to FFT frequency
         displayMode = 0;
         return 0;

     } else if (adc_value >= 900 && adc_value < 2000) {
         // Down button: Switch to comparator frequency
         displayMode = 1;
         return 1;

         // Cycle through units (MS -> KMH -> MPH -> MS)

     } else if (adc_value >= 2000 && adc_value < 3000) {
         // Left button: Switch to Speed display or cycle through units

         unit = (unit + 1) % 3;                   // Switch to Speed display mode
         return 2;
     }
     lastButtonPressTime = currentTime;
     return 3;
}


// Update display based on the current mode
void displayLCD(float speed, float freq){
	  switch (displayMode) {
	  	  case 0:
	  		snprintf(line, sizeof(line), "ADC: %.2fHz", freq);

	         break;

	      case 1:
	    	  snprintf(line, sizeof(line), "Comp: %.2fHz", freq);
	    	  //speed=frequencyToSpeed(freq);

	          break;

	  }
	  snprintf(string, sizeof(string), "Speed:%.2f %s",
	  	    (unit == MS ? speed : (unit == KMH ? msToKmh(speed) : msToMph(speed))),
	  	    (unit == MS ? "M/S" : (unit == KMH ? "KM/H" : "MPH")));

	  lcd16x2_1stLine();
	  lcd16x2_clear();
	  lcd16x2_printf(line);
	  lcd16x2_2ndLine();
	  lcd16x2_printf(string);


	  // Handle button presses to update displayMode
	  //processButton();

	  HAL_Delay(1000); // Adjust delay as needed
	  lcd16x2_clear();
}


float frequencyToSpeed (float frequency){
 	float speed = (frequency*0.01419);
 	return speed;
}

float msToKmh (float speed){
	return (speed*3.6);
}

float msToMph(float speed){
	return (speed*2.237);
}

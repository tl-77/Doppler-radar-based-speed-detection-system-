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


#define MS 0	// Constant for meters per second
#define KMH 1	// Constant for kilometers per hour
#define MPH 2	// Constant for miles per hour


static uint16_t unit = MS;   // Default unit is meters per second
uint16_t adc_value;           // Variable to store ADC value
static char string[20];       // String to store formatted speed display
char line[20];                // String to store line data for the LCD
uint32_t lastUpdateTime = 0;  // Variable for timing updates
static uint8_t displayMode = 0; // Variable to track the current display mode

// LCD initialization function
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

// Process button press and read ADC values
int processButton(ADC_HandleTypeDef *hadc2) {
     static uint32_t lastButtonPressTime = 0;
     //adc_value = 0;

     // Start ADC conversion
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

     // Determine button action based on ADC value
     if (adc_value > 0 && adc_value < 900) {
         // Up button: Switch to FFT frequency
         displayMode = 0;
         return 0;

     } else if (adc_value >= 900 && adc_value < 2000) {
         // Down button: Switch to comparator frequency
         displayMode = 1;
         return 1;

     } else if (adc_value >= 2000 && adc_value < 3000) {
         // Left button: Switch to Speed display or cycle through units
    	 // Cycle through units (MS -> KMH -> MPH -> MS)
         unit = (unit + 1) % 3;
         return 2;
     }
     lastButtonPressTime = currentTime;
     return 3;
}


// Update display based on the current mode
void displayLCD(float speed, float freq,UART_HandleTypeDef *huart2,UART_HandleTypeDef *huart3){
	  switch (displayMode) {
	  	  case 0:
	  		  // Display FFT frequency
	  		snprintf(line, sizeof(line), "ADC: %.2fHz", freq);
	         break;

	      case 1:
	    	  // Display comparator frequency
	    	  snprintf(line, sizeof(line), "Comp: %.2fHz", freq);
	          break;

	  }
	  // Format the speed string with appropriate units (M/S, KM/H, MPH)
	  snprintf(string, sizeof(string), "Speed:%.2f %s",
	  	    (unit == MS ? speed : (unit == KMH ? msToKmh(speed) : msToMph(speed))),
	  	    (unit == MS ? "M/S" : (unit == KMH ? "KM/H" : "MPH")));

	  lcd16x2_clear();

	  // Display on LCD
	  lcd16x2_1stLine();
	  lcd16x2_printf(line); // Display frequency info
	  lcd16x2_2ndLine();
	  lcd16x2_printf(string); // Display speed and unit info

	  // Send the data to 7 segment display via huart3
	  display7seg(speed,huart2,huart3);


	  HAL_Delay(1000); // Wait for 1 second before clearing the display

}

// Function to enable the driver (for transmission)
void RS485_EnableTransmit(void) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);  // Set DE pin high to enable transmission
}

// Function to disable the driver (for receiving)
void RS485_EnableReceive(void) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);  // Set DE pin low to enable receiving
}

// Function to convert a speed value into a single byte in BCD format
uint8_t ConvertSpeedToBCD(uint8_t speed) {
	// The BCD format places the tens digit in the upper nibble
	// and the ones digit in the lower nibble
    return ((speed / 10) << 4) | (speed % 10);  // Tens in upper nibble, ones in lower nibble
}

// Function to convert an 8-bit integer into a binary string (for debugging purposes)
void ConvertToBinary(uint8_t num, char* binaryStr) {
	// Loop through each bit of the number
	for (int i = 7; i >= 0; i--) {
		// Check if the current bit is 1 or 0
	    binaryStr[7 - i] = (num & (1 << i)) ? '1' : '0';
	}
    binaryStr[8] = '\0';  // Null-terminate the string
}

// Function to send a speed value in BCD format via RS485
void RS485_SendSpeed(uint8_t speed,UART_HandleTypeDef *huart2,UART_HandleTypeDef *huart3) {
    uint8_t bcd = ConvertSpeedToBCD(speed);  // Convert speed to BCD format

    // Create a binary string representation of the BCD value for debugging
    char binaryStr[9];
    ConvertToBinary(bcd, binaryStr);

    RS485_EnableTransmit();  // Enable transmission mode for RS485
    HAL_UART_Transmit(huart3, &bcd,1, HAL_MAX_DELAY);// Send BCD byte
    RS485_EnableReceive();  // Switch back to receive mode after transmitting

    // Prepare a debug message with the speed, BCD, and binary representation
    char buffer[50];
    sprintf(buffer, "Speed: %d -> BCD: 0x%X | Binary: %s\r\n", speed, bcd, binaryStr);
    // Send the debug message to Putty via a different UART
    HAL_UART_Transmit(huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

// Update the 7-segment display board
void display7seg(float speed,UART_HandleTypeDef *huart2,UART_HandleTypeDef *huart3){
    uint8_t data = (uint8_t) (unit == MS ? speed : (unit == KMH ? msToKmh(speed) : msToMph(speed)));
    RS485_SendSpeed(data,huart2,huart3);
}

// Function to convert frequency to speed in m/s unit
float frequencyToSpeed (float frequency){
 	float speed = (frequency*0.01419);
 	return speed;
}

// Function to convert speed from m/s to km/h
float msToKmh (float speed){
	return (speed*3.6);
}

// Function to convert speed from m/s to mph
float msToMph(float speed){
	return (speed*2.237);
}

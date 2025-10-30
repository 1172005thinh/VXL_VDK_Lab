/*
 * stm32f103c6.c
 *
 *  Created on: Sep 24, 2025
 *      Author: HungThinh
 */

 #include "main.h"
 #include "stm32f1xx_hal.h"
 #include "stm32f103c6.h"
 #include "soft_timer.h"

 const PINMAP pinMap[] = {
	 //PA1 to PA12
   {_1_GPIO_Port,  _1_Pin},   	// indexPin = 1
   {_2_GPIO_Port,  _2_Pin},  	// indexPin = 2
   {_3_GPIO_Port,  _3_Pin},  	// indexPin = 3
   {_4_GPIO_Port,  _4_Pin},   	// indexPin = 4
   {_5_GPIO_Port,  _5_Pin},   	// indexPin = 5
   {_6_GPIO_Port,  _6_Pin},   	// indexPin = 6
   {_7_GPIO_Port,  _7_Pin},   	// indexPin = 7
   {_8_GPIO_Port,  _8_Pin},   	// indexPin = 8
   {_9_GPIO_Port,  _9_Pin},   	// indexPin = 9
   {_10_GPIO_Port, _10_Pin},  	// indexPin = 10
   {_11_GPIO_Port, _11_Pin},  	// indexPin = 11
   {_12_GPIO_Port, _12_Pin},  	// indexPin = 12
   //PA13 to PA14 are reserved
   //PA15 for LED-SYS
   {_13_GPIO_Port, _13_Pin},  	// indexPin = 13
	 //PB0 to PB15
   {_14_GPIO_Port, _14_Pin},  	// indexPin = 14
   {_15_GPIO_Port, _15_Pin},  	// indexPin = 15
   {_16_GPIO_Port, _16_Pin},  	// indexPin = 16
   {_17_GPIO_Port, _17_Pin},  	// indexPin = 17
   {_18_GPIO_Port, _18_Pin},  	// indexPin = 18
   {_19_GPIO_Port, _19_Pin},  	// indexPin = 19
   {_20_GPIO_Port, _20_Pin},  	// indexPin = 20
   {_21_GPIO_Port, _21_Pin},  	// indexPin = 21
   {_22_GPIO_Port, _22_Pin},  	// indexPin = 22
   {_23_GPIO_Port, _23_Pin},  	// indexPin = 23
   {_24_GPIO_Port, _24_Pin},  	// indexPin = 24
   {_25_GPIO_Port, _25_Pin},  	// indexPin = 25
   {_26_GPIO_Port, _26_Pin},  	// indexPin = 26
   {_27_GPIO_Port, _27_Pin},  	// indexPin = 27
   {_28_GPIO_Port, _28_Pin},  	// indexPin = 28
   {_29_GPIO_Port, _29_Pin},  	// indexPin = 29
 };

 PINMAP LED_SYS = PA15;

 PINMAP segMent_1[7] = {
   PB0,
   PB1,
   PB2,
   PB3,
   PB4,
   PB5,
   PB6
 };

 PINMAP segMent_2[7] = {
   PB7,
   PB8,
   PB9,
   PB10,
   PB11,
   PB12,
   PB13
 };

 uint8_t segMapAnode[12] = {
   //abcdefg
   0b0000001, //0
   0b1001111, //1
   0b0010010, //2
   0b0000110, //3
   0b1001100, //4
   0b0100100, //5
   0b0100000, //6
   0b0001111, //7
   0b0000000, //8
   0b0000100, //9
   0b1111111, //10 - OFF
   0b1001111  //E - ERROR
 };

 void delay(double sec) {
   HAL_Delay(sec * 1000);
 }

 void togglePin(PINMAP pin, bool state) {
   //safety check
   if (pin.port == NULL) {
     return;
   }

   HAL_GPIO_WritePin(pin.port, pin.pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
 }

 void toggleSeg(PINMAP segment[7], uint8_t map[12], uint8_t number) {
   //safety check
   if (number < 0 || number > 10) {
     number = 11; //E - ERROR
   }
   
   for (int i = 0; i < 7; i++) {
     uint8_t state = (map[number] >> (6 - i)) & 0x01;
     togglePin(segment[i], state);
   }
 }

 void blinkPin(PINMAP pin) {
   HAL_GPIO_TogglePin(pin.port, pin.pin);
 }

 void blinkLED_SYS() {
   HAL_GPIO_TogglePin(LED_SYS.port, LED_SYS.pin);
 }

 void blinkLED_SYS_TIM(int duration) {
   if (timerLED_SYS_flag == 1) {
     setTimerLED_SYS(duration);
     blinkLED_SYS();
   }
 }
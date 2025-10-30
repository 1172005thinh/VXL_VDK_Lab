/*
 * stm32f103c6.h
 *
 *  Created on: Sep 24, 2025
 *      Author: HungThinh
 */

 #ifndef INC_STM32F103C6_H_
 #define INC_STM32F103C6_H_

 #include "main.h"
 #include <stdbool.h>

 typedef struct {
   GPIO_TypeDef *port;
	 uint16_t pin;
 } PINMAP;
 
 #define PA1   pinMap[0]
 #define PA2   pinMap[1]
 #define PA3   pinMap[2]
 #define PA4   pinMap[3]
 #define PA5   pinMap[4]
 #define PA6   pinMap[5]
 #define PA7   pinMap[6]
 #define PA8   pinMap[7]
 #define PA9   pinMap[8]
 #define PA10  pinMap[9]
 #define PA11  pinMap[10]
 #define PA12  pinMap[11]
 #define PA15  pinMap[12] //LED-SYS
 #define PB0   pinMap[13]
 #define PB1   pinMap[14]
 #define PB2   pinMap[15]
 #define PB3   pinMap[16]
 #define PB4   pinMap[17]
 #define PB5   pinMap[18]
 #define PB6   pinMap[19]
 #define PB7   pinMap[20]
 #define PB8   pinMap[21]
 #define PB9   pinMap[22]
 #define PB10  pinMap[23]
 #define PB11  pinMap[24]
 #define PB12  pinMap[25]
 #define PB13  pinMap[26]
 #define PB14  pinMap[27]
 #define PB15  pinMap[28]

 extern const PINMAP pinMap[];
 extern PINMAP LED_SYS;
 extern PINMAP segMent_1[7];
 extern PINMAP segMent_2[7];
 extern uint8_t segMapAnode[12];

 void delay(double sec);
 void togglePin(PINMAP indexPin, bool state);
 void toggleSeg(PINMAP segment[7], uint8_t map[12], uint8_t number);
 void blinkPin(PINMAP pin);
 void blinkLED_SYS();
 void blinkLED_SYS_TIM(int duration);

 #endif /* INC_STM32F103C6_H_ */

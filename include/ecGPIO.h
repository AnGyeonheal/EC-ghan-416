/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : Gyeonheal An
Created          : 05-03-2021
Modified         : 10-03-2023
Language/ver     : C++ in Keil uVision

Description      : GPIO.h
/----------------------------------------------------------------*/


#include "stm32f411xe.h"
#include "ecRCC.h"

#ifndef __ECGPIO_H
#define __ECGPIO_H

#define INPUT  0x00
#define OUTPUT 0x01
#define AF     0x02
#define ANALOG 0x03

#define HIGH 1
#define LOW  0

#define EC_NONE 0
#define EC_PU 1
#define EC_PD 2

#define EC_PUSH_PULL 0
#define EC_OPEN_DRAIN 1

#define EC_LOW 0
#define EC_MEDIUM 1
#define EC_FAST 2
#define EC_HIGH 3

#define LED_PIN 	5
#define BUTTON_PIN 13

// ------------------- LAB3P_2 Definition -------------------
#define pin_a   9
#define pin_b   6
#define pin_c   7
#define pin_d   6
#define pin_e   7
#define pin_f   9
#define pin_g   8
#define pin_dp  10

typedef struct{
    unsigned int next[2];
    unsigned int out[7];
} State_P1;

extern State_P1 FSM[10];  // {{next_state}, {a,b,c,d,e,f,g}}  FSM[state].next[Button_input]   FSM[state].out[0], FSM[state].out[1]

// -------------------  LAB3P_3 Definition -------------------
#define pin_D   7
#define pin_C   6
#define pin_B   7
#define pin_A   9

#define S0 0
#define S1 1
#define S2 2
#define S3 3
#define S4 4
#define S5 5
#define S6 6
#define S7 7
#define S8 8
#define S9 9

typedef struct{
    unsigned int next[2];
    unsigned int out[4];
} State_P2;

extern State_P2 FSM_D[10];

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
	 
void GPIO_init(GPIO_TypeDef *Port, int pin, unsigned int mode);
void GPIO_write(GPIO_TypeDef *Port, int pin, unsigned int Output);
int  GPIO_read(GPIO_TypeDef *Port, int pin);
void GPIO_mode(GPIO_TypeDef* Port, int pin, unsigned int mode);
void GPIO_ospeed(GPIO_TypeDef* Port, int pin, unsigned int speed);
void GPIO_otype(GPIO_TypeDef* Port, int pin, unsigned int type);
void GPIO_pupd(GPIO_TypeDef *Port, int pin, unsigned int pupd);
void seven_segment_init(void);
void seven_segment_decode(uint8_t state);
void sevensegment_display_init(void);
void sevensegment_display(uint8_t  state);
void GPIO(GPIO_TypeDef *Port, int pin, unsigned int mode, unsigned int speed, unsigned int type, unsigned int pupd);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : Gyeonheal An
Created          : 05-03-2021
Modified         : 10-14-2023
Language/ver     : C++ in Keil uVision

Description      : LAB_EXTI.c
/----------------------------------------------------------------*/

#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"
#include "ecEXTI.h"
#include "ecSysTick.h""

#define LED_PIN	5
#define BUTTON_PIN 13
#define MCU_CLK_PLL 84000000
#define MCU_CLK_HSI 16000000

unsigned char state = S0;
unsigned  char next_state = S0;
unsigned int input = 1;
unsigned int delay = 0;

void setup(void);
void EXTI15_10_IRQHandler(void);
void sevensegment_switch(void);
// Initialiization 


int main(void) {
	setup();
	while (1) {
        delay ++;
    }
}

void setup(void)
{
	RCC_PLL_init();
	SysTick_init();
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);
	// Priority Highest(0) External Interrupt 
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
	sevensegment_display_init();
}

//EXTI for Pin 13
void EXTI15_10_IRQHandler(void) {

    if (is_pending_EXTI(BUTTON_PIN) == 1) {
        while(1){
            delay++;
            if(delay > 500000) break;
        }
        sevensegment_switch();
        clear_pending_EXTI(BUTTON_PIN);
        delay = 0;
    }
}

void sevensegment_switch(void){
	
	input = 0;
	next_state = FSM[state].next[input];
	state = next_state;

	sevensegment_display(state);
	input = 1;

}
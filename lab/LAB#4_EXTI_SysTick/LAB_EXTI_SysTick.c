/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : Gyeonheal An
Created          : 05-03-2021
Modified         : 10-14-2023
Language/ver     : C++ in Keil uVision

Description      : LAB_EXTI_SysTick.c
/----------------------------------------------------------------*/

#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"
#include "ecEXTI.h"
#include "ecSysTick.h"

#define LED_PIN	5
#define BUTTON_PIN 13
#define MCU_CLK_PLL 84000000
#define MCU_CLK_HSI 16000000

unsigned char state = S0;
unsigned  char next_state = S0;
unsigned int input = 1;
int count = 0;

void setup(void);
void EXTI15_10_IRQHandler(void);
// Initialiization 


int main(void) {
    // Initialiization --------------------------------------------------------
    setup();

    // Inifinite Loop ----------------------------------------------------------
    while(1){
        sevensegment_display(count);
        delay_ms(1000);
        count++;
        if (count >9) count =0;
        SysTick_reset();
    }
}

void setup(void)
{
    RCC_PLL_init();
    SysTick_init();
    sevensegment_display_init();
    GPIO_init(GPIOC, BUTTON_PIN, INPUT);
    GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);
    EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);

}

void EXTI15_10_IRQHandler(void) {
    if (is_pending_EXTI(BUTTON_PIN) == 1) {
        count = 9;
        clear_pending_EXTI(BUTTON_PIN);
    }
}
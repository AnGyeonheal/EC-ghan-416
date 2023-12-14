/**
******************************************************************************
* @author   2023-10-31 by GH An
* @brief   Embedded Controller:  LAB - Stepper Motor
*
******************************************************************************
*/

#include "ecSTM32F411.h"

#define A 8
#define B 4
#define NA 5
#define NB 3

int RPM = 2;
int dir = 0;
void setup(void);

int main(){
    setup();
    Stepper_step(200, dir, FULL);
    while(1){
    }
}

void EXTI15_10_IRQHandler(void) {
    if (is_pending_EXTI(BUTTON_PIN)) {
        if(dir == 0) dir = 1;
        else if(dir == 1) dir = 0;
        clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
    }
}

void setup(void){
    RCC_PLL_init();
    SysTick_init();

    GPIO_init(GPIOC, BUTTON_PIN, INPUT);           // GPIOC pin13 initialization
    EXTI_init(GPIOC, BUTTON_PIN, FALL,15);           // External Interrupt Setting

    Stepper_init(GPIOA, A, GPIOB, B, GPIOB, NA, GPIOB, NB);
    Stepper_setSpeed(RPM);
}
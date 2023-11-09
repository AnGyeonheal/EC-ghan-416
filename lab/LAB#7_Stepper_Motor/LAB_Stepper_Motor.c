#include "ecSTM32F411.h"

#define A 10
#define B 4
#define NA 5
#define NB 3

int RPM = 29;

void setup(void);

void setup(void){
    RCC_PLL_init();
    SysTick_init();

    GPIO_init(GPIOC, BUTTON_PIN, INPUT);           // GPIOC pin13 initialization
    EXTI_init(GPIOC, BUTTON_PIN, FALL,15);           // External Interrupt Setting

    Stepper_init(GPIOB, A, GPIOB, B, GPIOB, NA, GPIOB, NB);
    Stepper_setSpeed(RPM);
}

int main(){
    setup();
    Stepper_step(2048*1, 0, HALF);
    while(1){
    }
}

void EXTI15_10_IRQHandler(void) {
    if (is_pending_EXTI(BUTTON_PIN)) {
        Stepper_stop();
        clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
    }
}


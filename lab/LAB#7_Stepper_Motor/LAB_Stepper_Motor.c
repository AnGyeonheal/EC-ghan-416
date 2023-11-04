#include "ecSTM32F411.h"

#define A 10
#define B 4
#define NA 5
#define NB 3

int RPM = 2;

void setup(void);

void setup(void){
    RCC_PLL_init();
    SysTick_init();
    Stepper_init(GPIOB, A, GPIOB, B, GPIOB, NA, GPIOB, NB);
    Stepper_setSpeed(RPM);
}

int main(){
    setup();
    while(1){
        Stepper_step(2048 * 10, 1, FULL);
    }
}


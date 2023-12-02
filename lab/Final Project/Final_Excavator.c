#include "ecSTM32F411.h"

PinName_t PWM_PIN = PA_0;
#define DIR_PIN1 3
PinName_t PWM_PIN2 = PA_1;
#define DIR_PIN2 2

void setup(void);

void TIM3_IRQHandler(void);

uint32_t count = 0;
float period = 1;
float duty = 0.5;
int dir1 = 0;
int dir2 = 0;
static int stop = 0;


int main(void) {
    // Initialization --------------------------------------------------
    setup();

    // Infinite Loop ---------------------------------------------------
    while (1) {
        GPIO_write(GPIOC, DIR_PIN1, dir1);
        PWM_duty(PWM_PIN, duty);
        GPIO_write(GPIOC, DIR_PIN2, dir2);
        PWM_duty(PWM_PIN2, 1);
    }
}

void TIM3_IRQHandler(void) {
    if (is_UIF(TIM3)) {            // Check UIF(update interrupt flag)
        count++;
        if(count > 2000 && count < 4000){
            dir1 = 1;
        }
        else if(count > 4000){
            dir1 = 0;
            count = 0;
        }
    }
    clear_UIF(TIM3);            // Clear UI flag by writing 0
}

// Initialiization
void setup(void) {
    RCC_PLL_init();

    // SYSTICK
    SysTick_init();

    // DIR SETUP
    GPIO_init(GPIOC, DIR_PIN1, OUTPUT);
    GPIO_otype(GPIOC, DIR_PIN1, EC_PUSH_PULL);
    GPIO_init(GPIOC, DIR_PIN2, OUTPUT);
    GPIO_otype(GPIOC, DIR_PIN2, EC_PUSH_PULL);
    // TIM
    TIM_UI_init(TIM3, 1);

    // PWM
    PWM_init(PWM_PIN);
    PWM_period_ms(PWM_PIN, period);
    PWM_init(PWM_PIN2);
    PWM_period_ms(PWM_PIN2, period);
}
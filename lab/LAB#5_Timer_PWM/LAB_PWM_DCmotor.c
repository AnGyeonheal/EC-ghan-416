/**
******************************************************************************
* @author   2023-10-31 by GH An
* @brief   Embedded Controller:  LAB - PWM_DC Motor
*
******************************************************************************
*/

#include "stm32f411xe.h"
#include "math.h"

// #include "ecSTM32F411.h"
#include "ecPinNames.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecTIM.h"
#include "ecPWM.h"
#include "ecEXTI.h"
#include "ecSysTick.h"

// Definition Button Pin & PWM Port, Pin
#define BUTTON_PIN 13
PinName_t PWM_PIN = PA_0;
#define DIR_PIN 2

void setup(void);

void TIM3_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

uint32_t count = 0;
float period = 1;
float duty = 0.25;
static int stop = 0;


int main(void) {
    // Initialization --------------------------------------------------
    setup();

    // Infinite Loop ---------------------------------------------------
    while (1) {
        GPIO_write(GPIOC, DIR_PIN, 0);
        if(stop == 0){
            PWM_duty(PWM_PIN, duty);
        }
        else if(stop == 1){
            PWM_duty(PWM_PIN, 0);
        }
    }
}

void TIM3_IRQHandler(void) {
    if (is_UIF(TIM3)) {            // Check UIF(update interrupt flag)
        count++;
        if(count > 2000 && count < 4000){
            duty = 0.75;
        }
        else if(count > 4000){
            duty = 0.25;
            count = 0;
        }
    }
    clear_UIF(TIM3);            // Clear UI flag by writing 0
}

void EXTI15_10_IRQHandler(void) {
    if (is_pending_EXTI(BUTTON_PIN)){
        stop ^= 1;
        clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
    }
}

// Initialiization
void setup(void) {
    RCC_PLL_init();

    // SYSTICK
    SysTick_init();

    // Status LED

    GPIO(GPIOA, LED_PIN, OUTPUT, EC_MEDIUM, EC_PUSH_PULL, EC_NONE);

    // Seven Segment
    seven_segment_init();

    //BUTTON
    GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
    GPIO_pupd(GPIOC, BUTTON_PIN, EC_PD);

    //EXTI
    EXTI_init(GPIOC, BUTTON_PIN, RISE, 15);

    // DIR SETUP
    GPIO_init(GPIOC, DIR_PIN, OUTPUT);
    GPIO_otype(GPIOC, DIR_PIN, EC_PUSH_PULL);

    // TIM
    TIM_UI_init(TIM3, 1);

    // PWM
    PWM_init(PWM_PIN);
    PWM_period_ms(PWM_PIN, period);


}
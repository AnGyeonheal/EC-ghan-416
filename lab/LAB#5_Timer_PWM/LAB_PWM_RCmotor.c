/**
******************************************************************************
* @author   2023-10-31 by GH An
* @brief   Embedded Controller:  LAB - PWM_RC Motor
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


// Definition Button Pin & PWM Port, Pin
#define BUTTON_PIN 13
PinName_t PWM_PIN = PA_1;

void setup(void);

void TIM3_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

uint32_t count = 0;
float period = 20;
float n = 0;
float duty = 0;
int dir = 0;											// dir = 0 , 1

int main(void) {
    // Initialization --------------------------------------------------
    setup();

    // Infinite Loop ---------------------------------------------------
    while (1) {
        PWM_duty(PWM_PIN, (duty / period));
    }
}

void TIM3_IRQHandler(void) {
    if (is_UIF(TIM3)) {            // Check UIF(update interrupt flag)
        count++;
        if(count > 500){
            if(dir == 0){
                n++;
                duty = 0.5 + (2 * n / 18);
                if(n == 18) dir = 1;
            }
            else if ( dir == 1){
                n--;
                duty = 0.5 + (2 * n / 18);
                if(n == 0) dir = 0;
            }
            count = 0;
        }
        clear_UIF(TIM3);            // Clear UI flag by writing 0
    }
}

void EXTI15_10_IRQHandler(void) {
    if (is_pending_EXTI(BUTTON_PIN)){
        duty = 0;
        dir = 0;
        n = 0;
        clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
    }
}

// Initialiization
void setup(void) {
    // CLK SETUP
    RCC_PLL_init();
    // EXTI: BUTTON SETUP
    GPIO_init(GPIOC, BUTTON_PIN, INPUT);
    GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);
    EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
    // TIMER SETUP
    TIM_UI_init(TIM3, 1);
    // PWM SETUP
    PWM_init(PWM_PIN);
    PWM_period_ms(PWM_PIN, period);
}


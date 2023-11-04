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
void LED_toggle(void);
void TIM3_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void printsevensemgent(void);

uint32_t count = 0;
double duty = 1;
unsigned int direction = 0;
unsigned int delay = 0;
unsigned int state = 0;
float vel = 0;

int main(void) {
    // Initialization --------------------------------------------------
    setup();

    // Infinite Loop ---------------------------------------------------
    while (1) {

    }
}

void TIM3_IRQHandler(void) {
    if (is_UIF(TIM3)) {            // Check UIF(update interrupt flag)

        count++;
        if(count > 500){
            if(vel == 1){
                LED_toggle();
                count = 0;
            }
            else if(vel == 0){
                GPIO_write(GPIOA, LED_PIN, 0);
            }
        }
        if(count > 1000){
            if(vel == 0.5){
                LED_toggle();
                count = 0;
            }
            else if(vel == 0){
                GPIO_write(GPIOA, LED_PIN, 0);
            }
        }

        if (state == 0) {
            direction = 1;
            duty = 1;
            vel = 0;
        } else if (state == 1) {
            direction = 1;
            duty = 0.5;
            vel = 0.5;
        } else if (state == 2) {
            direction = 1;
            duty = 0;
            vel = 1;
        } else if (state == 3) {
            direction = 1;
            duty = 1;
            vel = 0;
        } else if (state == 4) {
            direction = 0;
            duty = 0.5;
            vel = 0.5;
        } else if (state == 5) {
            direction = 0;
            duty = 1;
            vel = 1;
        }

        PWM_duty(PWM_PIN, duty);
        GPIO_write(GPIOC, DIR_PIN, direction);
        printsevensemgent();


    }
    clear_UIF(TIM3);            // Clear UI flag by writing 0
}

void EXTI15_10_IRQHandler(void) {
    if ((is_pending_EXTI(BUTTON_PIN))){
        state++;
        if(state > 5) state = 0;
        clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
    }
}

void LED_toggle(void){
    static unsigned int out = 0;
    out ^= 1UL;
    GPIO_write(GPIOA, LED_PIN, out);
}

void printsevensemgent(void){

    if(vel == 0){
        sevensegment_display(0);
    }
    else if(vel == 0.5){
        sevensegment_display(1);
    }
    else if(vel == 1){
        sevensegment_display(2);
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
    sevensegment_display_init();

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
    PWM_period_ms(PWM_PIN, 1);

}
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecTIM.h"
#include "ecEXTI.h"
#include "ecSysTick.h"

#define LED0 0
#define LED1 1
#define LED2 0
#define LED3 1

uint32_t _count = 0;
uint32_t pause = 0;
uint32_t delay = 0;

unsigned int state = S0;
unsigned  char next_state = S0;
unsigned int input = 1;
uint32_t num = 0;
static unsigned int out = 0;

void setup(void);
void Upcount(void);
void LED_toggle();
void displayp();
void Upcount_sevensegment(void);

int main(void) {
    // Initialization --------------------------------------------------
    setup();
    seven_segment_decode(S0);
    // Infinite Loop ---------------------------------------------------
    while(1){
        if(pause == 1){
            TIM_UI_disable(TIM2);
            displayp();
            LED_toggle();
            delay_ms(1000);
            SysTick_reset();
        }
        else{
            TIM_UI_enable(TIM2);
        }
    }
}

void TIM2_IRQHandler(void){
    if(is_UIF(TIM2)){			// Check UIF(update interrupt flag)
        _count++;
        if (_count > 50000) {
            Upcount();
            _count = 0;
            if(num == 15){
                Upcount_sevensegment();
            }
        }
        if(pause == 0){
            GPIO_write(GPIOA, LED_PIN, 0);
            seven_segment_decode(state);
        }
        clear_UIF(TIM2); 		// Clear UI flag by writing 0
    }
}

void Upcount(void){
    num++;

    GPIO_write(GPIOA, LED0, num>>0&1);
    GPIO_write(GPIOA, LED1, num>>1&1);
    GPIO_write(GPIOB, LED2, num>>2&1);
    GPIO_write(GPIOC, LED3, num>>3&1);

    if(num == 17) num =0;
}

void Upcount_sevensegment(void){
    input = 0;
    next_state = FSM[state].next[input];
    state = next_state;
    seven_segment_decode(state);
    input = 1;
}

void LED_toggle(){
    out ^= 1UL;
    GPIO_write(GPIOA, LED_PIN, out);
}

void displayp(){
    GPIO_write(GPIOB, pin_a, 0);     // a
    GPIO_write(GPIOA, pin_b, 0);     // b
    GPIO_write(GPIOA, pin_c, 1);     // c
    GPIO_write(GPIOB, pin_d, 1);     // d
    GPIO_write(GPIOC, pin_e, 0);     // e
    GPIO_write(GPIOA, pin_f, 0);     // f
    GPIO_write(GPIOA, pin_g, 0);     // g
}

void EXTI15_10_IRQHandler(void) {
    if (is_pending_EXTI(BUTTON_PIN)){
        pause ^= 1;
        delay = 0;
        clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
    }
}

// Initialization
void setup(void){
    RCC_PLL_init();

    // SYSTICK
    SysTick_init();

    // Status LED
    GPIO(GPIOA, LED_PIN, OUTPUT, EC_MEDIUM, EC_PUSH_PULL, EC_NONE);

    // LED 0~4

    GPIO(GPIOA, LED0, OUTPUT, EC_MEDIUM, EC_PUSH_PULL, EC_NONE);
    GPIO(GPIOA, LED1, OUTPUT, EC_MEDIUM, EC_PUSH_PULL, EC_NONE);
    GPIO(GPIOB, LED2, OUTPUT, EC_MEDIUM, EC_PUSH_PULL, EC_NONE);
    GPIO(GPIOC, LED3, OUTPUT, EC_MEDIUM, EC_PUSH_PULL, EC_NONE);

    // Seven Segment
    seven_segment_init();

    //BUTTON
    GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
    GPIO_pupd(GPIOC, BUTTON_PIN, EC_PD);

    //EXTI
    EXTI_init(GPIOC, BUTTON_PIN, RISE, 15);

    // TIM
    TIM_UI_init(TIM2, 1);
    TIM_period_us(TIM2, 10);
}

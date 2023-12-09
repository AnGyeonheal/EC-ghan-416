#include "ecSTM32F411.h"

PinName_t PWM_PIN1 = PA_0;
#define DIR_PIN1 3
PinName_t PWM_PIN2 = PA_1;
#define DIR_PIN2 2
PinName_t PWM_PIN3 = PB_10;
#define DIR_PIN3  4

void setup(void);

void TIM3_IRQHandler(void);

uint32_t count = 0;
float period = 1;
float duty1;
float duty2;
float duty3;

int dir1 = 0;
int dir2 = 0;
int dir3 = 0;
static int stop = 0;
static volatile uint8_t BT_Data = 0;

int main(void) {
    // Initialization --------------------------------------------------
    setup();

    // Infinite Loop ---------------------------------------------------
    while (1) {
        // First
        GPIO_write(GPIOC, DIR_PIN1, dir1);
        PWM_duty(PWM_PIN1, duty1);
        // Second
        GPIO_write(GPIOC, DIR_PIN2, dir2);
        PWM_duty(PWM_PIN2, duty2);
        // shovel
        GPIO_write(GPIOC, DIR_PIN3, dir3);
        PWM_duty(PWM_PIN3, duty3);
    }
}

void USART1_IRQHandler(){
    if(is_USART1_RXNE()){
        BT_Data = USART1_read();
        // First
        if(BT_Data == 'q'){
            dir1 = 0;
            duty1 = 1;
            dir2 = 0;
            duty2 = 0;
            dir3 = 0;
            duty3 = 0;
        }
        else if(BT_Data == 'a') {
            dir1 = 1;
            duty1 = 0;
            dir2 = 0;
            duty2 = 0;
            dir3 = 0;
            duty3 = 0;
        }
        // Second
        else if(BT_Data == 'w') {
            dir2 = 0;
            duty2 = 1;
            dir3 = 0;
            duty3 = 0;
            dir1 = 0;
            duty1 = 0;
        }
        else if(BT_Data == 's') {
            dir2 = 1;
            duty2 = 0;
            dir3 = 0;
            duty3 = 0;
            dir1 = 0;
            duty1 = 0;
        }
        // Shovel
        else if(BT_Data == 'e') {
            dir3 = 0;
            duty3 = 1;
            dir2 = 0;
            duty2 = 0;
            dir1 = 0;
            duty1 = 0;
        }
        else if(BT_Data == 'd') {
            dir3 = 1;
            duty3 = 0;
            dir2 = 0;
            duty2 = 0;
            dir1 = 0;
            duty1 = 0;
        }
    }
}


void TIM3_IRQHandler(void) {
    if (is_UIF(TIM3)) {            // Check UIF(update interrupt flag)

    }
    clear_UIF(TIM3);            // Clear UI flag by writing 0
}

// Initialiization
void setup(void) {
    RCC_PLL_init();

    // SYSTICK
    SysTick_init();

    // Bluetooth serial init
    UART1_init();
    UART1_baud(BAUD_9600);

    // DIR1
    GPIO_init(GPIOC, DIR_PIN1, OUTPUT);
    GPIO_otype(GPIOC, DIR_PIN1, EC_PUSH_PULL);
    // DIR2
    GPIO_init(GPIOC, DIR_PIN2, OUTPUT);
    GPIO_otype(GPIOC, DIR_PIN2, EC_PUSH_PULL);
    // DIR3
    GPIO_init(GPIOC, DIR_PIN3, OUTPUT);
    GPIO_otype(GPIOC, DIR_PIN3, EC_PUSH_PULL);

    // TIM
    TIM_UI_init(TIM3, 1);

    // PWM1
    PWM_init(PWM_PIN1);
    PWM_period_ms(PWM_PIN1, period);
    // PWM2
    PWM_init(PWM_PIN2);
    PWM_period_ms(PWM_PIN2, period);
    // PWM3
    PWM_init(PWM_PIN3);
    PWM_period_ms(PWM_PIN3, period);

}
/**
******************************************************************************
* @author   2023-10-31 by GH An
* @brief   Embedded Controller:  LAB - USART-Bluetooth
*
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecUART.h"
#include "ecPWM.h"
#include "string.h"

#define DIR_PIN1 2
#define DIR_PIN2 3
PinName_t PWM_PIN1 = PA_0;
PinName_t PWM_PIN2 = PA_1;
float period = 500;

#define MAX_BUF 	10
#define END_CHAR 	13

static volatile uint8_t PC_Data = 0;
static volatile uint8_t BT_Data = 0;
void setup(void);

void main(){
    setup();
    GPIO_write(GPIOC, DIR_PIN1, 1);
    GPIO_write(GPIOC, DIR_PIN2, 1);
    PWM_duty(PWM_PIN1, 1);
    PWM_duty(PWM_PIN2, 1);
    while(1){

    }
}
void USART1_IRQHandler(){                       // USART2 RX Interrupt : Recommended
    if(is_USART1_RXNE()){
			BT_Data = USART1_read();	
			USART1_write(&BT_Data,1);
			USART1_write("\r\n", 2);

			if(BT_Data == 's'){
				PWM_duty(PWM_PIN1, 1);
				PWM_duty(PWM_PIN2, 1);
			}
			else if(BT_Data == 'd'){
				PWM_duty(PWM_PIN1, 0.2);
				PWM_duty(PWM_PIN2, 0.5);
			}
			else if(BT_Data == 'a'){
				PWM_duty(PWM_PIN1, 0.5);
				PWM_duty(PWM_PIN2, 0.2);
			}
			else if(BT_Data == 'w'){
				PWM_duty(PWM_PIN1, 0.2);
				PWM_duty(PWM_PIN2, 0.2);
			}
			else if(BT_Data == 'L'){
				GPIO_write(GPIOA, LED_PIN, 0);
			}
			else if(BT_Data == 'H'){
				GPIO_write(GPIOA, LED_PIN, 1);
			}
    }
}


void setup(void){
    RCC_PLL_init();
	
		// LED
		GPIO(GPIOA, LED_PIN, OUTPUT, EC_MEDIUM, EC_PUSH_PULL, EC_NONE);

    // BT serial init
    UART1_init();
    UART1_baud(BAUD_9600);

    // DIR1 SETUP
    GPIO_init(GPIOC, DIR_PIN1, OUTPUT);
    GPIO_otype(GPIOC, DIR_PIN1, EC_PUSH_PULL);

    // DIR2 SETUP
    GPIO_init(GPIOC, DIR_PIN2, OUTPUT);
    GPIO_otype(GPIOC, DIR_PIN2, EC_PUSH_PULL);

    // PWM1
    PWM_init(PWM_PIN1);
    PWM_period_us(PWM_PIN1, period);

    // PWM2
    PWM_init(PWM_PIN2);
    PWM_period_us(PWM_PIN2, period);

}
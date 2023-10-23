#include "stm32f411xe.h"
#include "math.h"

// #include "ecSTM32F411.h"
#include "ecPinNames.h"
#include "ecGPIO.h"
#include "ecSysTick.h"
#include "ecRCC.h"
#include "ecTIM.h"
#include "ecPWM.h"
#include "ecEXTI.h"


// Definition Button Pin & PWM Port, Pin
#define BUTTON_PIN 13
#define PWM_PIN 1

void setup(void);
void TIM3_IRQHandlr(void);
void EXTI15_10_IRQHandler(void);
int count = 0;

int main(void) {
	// Initialization --------------------------------------------------
	setup();

	// Infinite Loop ---------------------------------------------------
   while(1){
		 for(int i=0; i<11; i++){
      PWM_duty(PWM_PIN, 0.1*i);
      delay_ms(100);
		}
			for(int i=10; i>=0; i--){
      PWM_duty(PWM_PIN, 0.1*i);
      delay_ms(100);
		}
	}
}

void TIM3_IRQHandlr(void){
		if(is_UIF(TIM2)){			// Check UIF(update interrupt flag)
			count++;
		if (count > 1000) {
			count = 0;
		}
		clear_UIF(TIM2); 		// Clear UI flag by writing 0
	}
}

void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(BUTTON_PIN) == 1) {
			count = 0;
			clear_pending_EXTI(BUTTON_PIN);
	}
}


// Initialiization 
void setup(void) {
	// CLK SETUP
	RCC_PLL_init();
	SysTick_init();
	// TIMER SETUP
	TIM_UI_init(TIM3, 1);
	TIM_UI_enable(TIM3);
	// EXTI: BUTTON SETUP
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
	// SERVO MOTOR SETUP
	GPIO_init(GPIOA, PWM_PIN, AF);
	GPIO_pupd(GPIOA, PWM_PIN, EC_PU);
	GPIO_ospeed(GPIOA, PWM_PIN, EC_FAST);
	GPIO_otype(GPIOA, PWM_PIN, EC_PUSH_PULL);
	// PWM SETUP
	PWM_init(PWM_PIN);
	PWM_period(PWM_PIN,20);
}










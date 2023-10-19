/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2021-8-12 by YKKIM  	
* @brief   Embedded Controller:  Tutorial ___
*					 - _________________________________
* 
******************************************************************************
*/
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecSysTick.h"

#define LED_PIN 	5

void setup(void);
	
int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	
	// TEMP: TIMER Register Initialiization --------------------------------------------------------		
	TIM_TypeDef *TIMx;
	TIMx = TIM2;
	
	// GPIO: ALTERNATIVE function setting
	GPIOA->AFR[0] |= 1UL << (4*LED_PIN);  // AF1 at PA5 = TIM2_CH1 (p.150)
	GPIOA->MODER &= ~(3UL << 2*LED_PIN);
    GPIOA->MODER |= 2UL << 2*LED_PIN;

    // TIMER: PWM setting
	RCC->APB1ENR |= 1 << 0;         // Enable TIMER clock
	
	TIMx->CR1 &= ~(1UL << 4UL);				    // Direction Up-count
	
	TIMx->PSC = 839UL;						    // Set Timer CLK = 100kHz : (PSC + 1) = 84MHz/100kHz --> PSC = ?
	
	TIMx->ARR = 99UL;							// Auto-reload: Upcounting (0...ARR).
												// Set Counter CLK = 1kHz : (ARR + 1) = 100kHz/1kHz --> ARR = ?
	
	TIMx->CCMR1 &= ~TIM_CCMR1_OC1M;  			// Clear ouput compare mode bits for channel 1
	TIMx->CCMR1 |= 6UL << 4UL;    				// OC1M = 110 for PWM Mode 1 output on ch1 = 
	TIMx->CCMR1	|= 1UL << 3UL;    		    // Output 1 preload enable (make CCR1 value changable)
	
	TIMx->CCR1 =  (TIMx->ARR + 1) / 2;     			// Output Compare Register for channel 1
	
	TIMx->CCER &= ~(1UL << 1UL);    			// select output polarity: active high
	TIMx->CCER |= 1UL << 0UL;					// Enable output for ch1
	
	TIMx->CR1  |= TIM_CR1_CEN;      			// Enable counter
	TIMx->BDTR |= TIM_BDTR_MOE;
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		for (uint32_t i=0;i<=10;i+=5){
            TIMx->CCR1 = i * 10;
            delay_ms(100);
		}
        for (uint32_t i=0;i<=10;i+=5){
            TIMx->CCR1 = (10-i) * 10;
            delay_ms(100);
        }
	}
}
// Initialiization 
void setup(void)
{	
	RCC_PLL_init();       // System Clock = 84MHz
	SysTick_init();       // for delay_ms()
	GPIO_init(GPIOA, LED_PIN, OUTPUT);     // GPIOA 5 ALTERNATE function
	GPIO_pupd(GPIOA, LED_PIN, EC_NONE);
	GPIO_otype(GPIOA, LED_PIN, EC_PUSH_PULL);
	GPIO_ospeed(GPIOA, LED_PIN, EC_HIGH);
}
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecTIM.h"


#define LED_PIN	5
uint32_t _count = 0;
void setup(void);
void TIM2_IRQHandler(void);
void LED_toggle(void);

int main(void) {
	// Initialization --------------------------------------------------
	setup();
	
	// Infinite Loop ---------------------------------------------------
	while(1){}
}


// Initialization
void setup(void){
	RCC_PLL_init();				// System Clock = 84MHz
	GPIO_init(GPIOA, LED_PIN, OUTPUT);	// calls RCC_GPIOA_enable()
	TIM_UI_init(TIM2, 1);			// TIM2 Update-Event Interrupt every 1 msec 
	TIM_UI_enable(TIM2);
}

void TIM2_IRQHandler(void){
	if(is_UIF(TIM2)){			// Check UIF(update interrupt flag)
		_count++;
		if (_count > 1000) {
			LED_toggle();		// LED toggle every 1 sec
			_count = 0;
		}
		clear_UIF(TIM2); 		// Clear UI flag by writing 0
	}
}
void LED_toggle(void){
	static unsigned int out = 0;
	out ^= 1UL;
	GPIO_write(GPIOA, LED_PIN, out);
}
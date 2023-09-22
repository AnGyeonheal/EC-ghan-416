#define LED_PIN 	5
#define BUTTON_PIN 13
#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"
void setup(void);
	
int main(void) { 
	// Initialiization
	setup();
	char a = HIGH;
	// Inifinite Loop 
	while(1){
		if(GPIO_read(GPIOC, BUTTON_PIN)!=0){
			a^=LOW;
		}
		GPIO_write(GPIOA, LED_PIN, a);
	}
}


// Initialiization 
void setup(void)
{
	RCC_HSI_init();	
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	GPIO_init(GPIOA, LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()
}
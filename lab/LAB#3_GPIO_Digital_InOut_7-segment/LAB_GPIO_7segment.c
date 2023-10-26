/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : Gyeonheal An
Created          : 05-03-2021
Modified         : 10-03-2023
Language/ver     : C++ in Keil uVision

Description      : LAB_GPIO_7segment
/----------------------------------------------------------------*/

#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"

unsigned char state = S0;
unsigned  char next_state = S0;
unsigned int input = 1;

void setup(void);

int main(void) {
    int delay=0;

	// Initialization --------------------------------------------------------
	setup();
    //------------------------ problem 2 ------------------------
     seven_segment_decode(S0);
     GPIO_write(GPIOB, pin_dp, HIGH);      // DP

    //------------------------ problem 3 ------------------------
//    sevensegment_display(S0);

	// Infinite Loop ----------------------------------------------------------
	while(1){

        if((GPIO_read(GPIOC, BUTTON_PIN) == 0) && (delay > 100000)){
            input = 0;
            next_state = FSM[state].next[input];
            state = next_state;
            // ---------------- problee 2 ----------------
             seven_segment_decode(state);
            // ---------------- problem 3 ----------------
//            sevensegment_display(state);

            delay = 0;
            input = 1;
        }
        delay++;
	}
}

void setup(void)
{
    RCC_HSI_init();
    GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
    GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);
    seven_segment_init();
    //sevensegment_display_init();
}
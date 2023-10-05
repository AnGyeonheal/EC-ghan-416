/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : Gyeonheal An
Created          : 05-03-2021
Modified         : 10-03-2023
Language/ver     : C++ in Keil uVision

Description      : GPIO.C
/----------------------------------------------------------------*/

#include "stm32f4xx.h"
#include "stm32f411xe.h"
#include "ecGPIO.h"

State_P1 FSM[10] = {
        {{S1, S0}, {0,0,0,0,0,0,1}},    // 0
        {{S2, S1}, {1,0,0,1,1,1,1}},    // 1
        {{S3, S2}, {0,0,1,0,0,1,0}},    // 2
        {{S4, S3}, {0,0,0,0,1,1,0}},    // 3
        {{S5, S4}, {1,0,0,1,1,0,0}},    // 4
        {{S6, S5}, {0,1,0,0,1,0,0}},    // 5
        {{S7, S6}, {1,1,0,0,0,0,0}},    // 6
        {{S8, S7}, {0,0,0,1,1,1,1}},    // 7
        {{S9, S8}, {0,0,0,0,0,0,0}},    // 8
        {{S0, S9}, {0,0,0,1,1,0,0}}     // 9

};

// Decoding Version
State_P2 FSM_D[10] = {
        {{S1, S0}, {0,0,0,0}},    // 0
        {{S2, S1}, {0,0,0,1,}},    // 1
        {{S3, S2}, {0,0,1,0,}},    // 2
        {{S4, S3}, {0,0,1,1,}},    // 3
        {{S5, S4}, {0,1,0,0,}},    // 4
        {{S6, S5}, {0,1,0,1,}},    // 5
        {{S7, S6}, {0,1,1,0,}},    // 6
        {{S8, S7}, {0,1,1,1,}},    // 7
        {{S9, S8}, {1,0,0,0,}},    // 8
        {{S0, S9}, {1,0,0,1,}}     // 9

};

void GPIO_init(GPIO_TypeDef *Port, int pin, unsigned int mode){     
	// mode  : Input(0), Output(1), AlterFunc(2), Analog(3)   
	if (Port == GPIOA)
		RCC_GPIOA_enable();
	if (Port == GPIOB)
		RCC_GPIOB_enable();
	if (Port == GPIOC)
		RCC_GPIOC_enable();
	if (Port == GPIOD)
		RCC_GPIOD_enable();

	// Make it for GPIOB, GPIOD..GPIOH

	// You can also make a more general function of
	// void RCC_GPIO_enable(GPIO_TypeDef *Port);
	GPIO_mode(Port, pin, mode);
}

// GPIO Mode          : Input(00), Output(01), AlterFunc(10), Analog(11)
void GPIO_mode(GPIO_TypeDef *Port, int pin, unsigned int mode){
   Port->MODER &= ~(3UL<<(2*pin));     
   Port->MODER |= mode <<(2*pin);    
}


// GPIO Speed          : Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
void GPIO_ospeed(GPIO_TypeDef *Port, int pin, unsigned int speed){
	Port->OSPEEDR &= ~(3UL<<(2*pin));
	Port->OSPEEDR |= speed << (2*pin);
}

// GPIO Output Type: Output push-pull (0, reset), Output open drain (1)
void GPIO_otype(GPIO_TypeDef *Port, int pin, unsigned int type){
	Port->OTYPER &= ~(1UL<<(pin));
	Port->OTYPER |= type << pin;
}

// GPIO Push-Pull    : No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
void GPIO_pupd(GPIO_TypeDef *Port, int pin, unsigned int pupd){
	Port->PUPDR &= ~(3UL<<(2*pin));
	Port->PUPDR |= pupd << (2*pin);
}


int GPIO_read(GPIO_TypeDef *Port, int pin){
	int bitVal = 0;
	bitVal = Port->IDR >> pin & 1UL;
	
	return bitVal;
}

void GPIO_write(GPIO_TypeDef *Port, int pin, unsigned int Output){
	Port->ODR &= ~(1UL << pin);
	Port->ODR |= (Output << pin);
}

void seven_segment_init(void){
    RCC_HSI_init();
    GPIO_init(GPIOA, pin_a, OUTPUT);
    GPIO_init(GPIOA, pin_b, OUTPUT);
    GPIO_init(GPIOA, pin_c, OUTPUT);
    GPIO_init(GPIOB, pin_d, OUTPUT);
    GPIO_init(GPIOC, pin_e, OUTPUT);
    GPIO_init(GPIOA, pin_f, OUTPUT);
    GPIO_init(GPIOA, pin_g, OUTPUT);
    GPIO_init(GPIOB, pin_dp, OUTPUT);

    GPIO_otype(GPIOA, pin_a, EC_PUSH_PULL);
    GPIO_otype(GPIOA, pin_b, EC_PUSH_PULL);
    GPIO_otype(GPIOA, pin_c, EC_PUSH_PULL);
    GPIO_otype(GPIOB, pin_d, EC_PUSH_PULL);
    GPIO_otype(GPIOC, pin_e, EC_PUSH_PULL);
    GPIO_otype(GPIOA, pin_f, EC_PUSH_PULL);
    GPIO_otype(GPIOA, pin_g, EC_PUSH_PULL);
    GPIO_otype(GPIOB, pin_dp, EC_PUSH_PULL);

    GPIO_pupd(GPIOA, pin_a, EC_NONE);
    GPIO_pupd(GPIOA, pin_b, EC_NONE);
    GPIO_pupd(GPIOA, pin_c, EC_NONE);
    GPIO_pupd(GPIOB, pin_d, EC_NONE);
    GPIO_pupd(GPIOC, pin_e, EC_NONE);
    GPIO_pupd(GPIOA, pin_f, EC_NONE);
    GPIO_pupd(GPIOA, pin_g, EC_NONE);
    GPIO_pupd(GPIOB, pin_dp, EC_NONE);

    GPIO_ospeed(GPIOA, pin_a, EC_MEDIUM);
    GPIO_ospeed(GPIOA, pin_b, EC_MEDIUM);
    GPIO_ospeed(GPIOA, pin_c, EC_MEDIUM);
    GPIO_ospeed(GPIOB, pin_d, EC_MEDIUM);
    GPIO_ospeed(GPIOC, pin_e, EC_MEDIUM);
    GPIO_ospeed(GPIOA, pin_f, EC_MEDIUM);
    GPIO_ospeed(GPIOA, pin_g, EC_MEDIUM);
    GPIO_ospeed(GPIOB, pin_dp, EC_MEDIUM);
}

void seven_segment_decode(uint8_t state){
    unsigned int a, b, c, d, e, f, g;

    a = FSM[state].out[0];
    b = FSM[state].out[1];
    c = FSM[state].out[2];
    d = FSM[state].out[3];
    e = FSM[state].out[4];
    f = FSM[state].out[5];
    g = FSM[state].out[6];

    GPIO_write(GPIOA, pin_a, a);     // a
    GPIO_write(GPIOA, pin_b, b);     // b
    GPIO_write(GPIOA, pin_c, c);     // c
    GPIO_write(GPIOB, pin_d, d);     // d
    GPIO_write(GPIOC, pin_e, e);     // e
    GPIO_write(GPIOA, pin_f, f);     // f
    GPIO_write(GPIOA, pin_g, g);     // g
}

void sevensegment_display_init(void){
    RCC_HSI_init();

    GPIO_init(GPIOA, pin_D, OUTPUT);
    GPIO_init(GPIOB, pin_C, OUTPUT);
    GPIO_init(GPIOC, pin_B, OUTPUT);
    GPIO_init(GPIOA, pin_A, OUTPUT);

    GPIO_otype(GPIOA, pin_D, EC_PUSH_PULL);
    GPIO_otype(GPIOB, pin_C, EC_PUSH_PULL);
    GPIO_otype(GPIOC, pin_B, EC_PUSH_PULL);
    GPIO_otype(GPIOA, pin_A, EC_PUSH_PULL);

    GPIO_pupd(GPIOA, pin_D, EC_NONE);
    GPIO_pupd(GPIOB, pin_C, EC_NONE);
    GPIO_pupd(GPIOC, pin_B, EC_NONE);
    GPIO_pupd(GPIOA, pin_A, EC_NONE);

    GPIO_ospeed(GPIOA, pin_D, EC_MEDIUM);
    GPIO_ospeed(GPIOB, pin_C, EC_MEDIUM);
    GPIO_ospeed(GPIOC, pin_B, EC_MEDIUM);
    GPIO_ospeed(GPIOA, pin_A, EC_MEDIUM);
}

void sevensegment_display(uint8_t  state){
    unsigned int D, C, B, A;

    D = FSM_D[state].out[0];
    C = FSM_D[state].out[1];
    B = FSM_D[state].out[2];
    A = FSM_D[state].out[3];

    GPIO_write(GPIOA, pin_c, D);
    GPIO_write(GPIOB, pin_d, C);
    GPIO_write(GPIOC, pin_e, B);
    GPIO_write(GPIOA, pin_f, A);

}

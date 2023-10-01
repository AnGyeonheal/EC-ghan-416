/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2022-08-12 by YKKIM  	
* @brief   Embedded Controller:  Tutorial Digital In/Out 7-segment Display
* 
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"

#define S0 0
#define S1 1
#define S2 2
#define S3 3
#define S4 4
#define S5 5
#define S6 6
#define S7 7
#define S8 8
#define S9 9

#define BUTTON_PIN  13
#define pin_a   5
#define pin_b   6
#define pin_c   7
#define pin_d   6
#define pin_e   7
#define pin_f   9
#define pin_g   8
#define pin_dp  10

unsigned char state = S0;
unsigned  char next_state = S0;
unsigned int input = 1;

void setup(void);
void seven_segment_init(void);
void seven_segment_decode(uint8_t state);

typedef struct{
    unsigned int next[2];
    unsigned int out[7];
} State_t;

State_t FSM[10] = {
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

};  // {{next_state}, {a,b,c,d,e,f,g}}  FSM[state].next[Button_input]   FSM[state].out[0], FSM[state].out[1]

int main(void) {
    int delay=0;

	// Initialization --------------------------------------------------------
	setup();
    seven_segment_decode(S0);
    GPIO_write(GPIOB, pin_dp, HIGH);      // DP
	// Infinite Loop ----------------------------------------------------------
	while(1){

        if((GPIO_read(GPIOC, BUTTON_PIN) == 0) && (delay > 100000)){
            input = 0;
            next_state = FSM[state].next[input];
            state = next_state;
            seven_segment_decode(state);
            delay = 0;
            input = 1;
        }
        delay++;
	}
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


void setup(void)
{
    RCC_HSI_init();
    GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
    GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);
    seven_segment_init();
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
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
#define v0 0.7
#define v1 0.5
#define v2 0.25
#define v3 0
#define F 1
#define B 0

float period = 500;

#define MAX_BUF 	10
#define END_CHAR 	13

static volatile uint8_t PC_Data = 0;
static volatile uint8_t BT_Data = 0;

int i=0;
char mode;
double vel[4] = {v0, v1, v2, v3};
int str1 = 0;
int str2 = 0;
int str_level = 0;
double vel1 = 0;
double vel2 = 0;
char DIR;
char VEL;
char STR;
uint8_t dir = 1;


void setup(void);
double str_angle(int str_level);
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
        if(dir == F) DIR = 'F';
        else if(dir == B) DIR = 'B';

        BT_Data = USART1_read();
        USART1_write("MOD:",4);
        USART1_write(&mode,1);
        USART1_write(" DIR:",5);
        USART1_write(&DIR,1);
        USART1_write(" STR:", 5);
        USART1_write(&STR, 2);
        USART1_write(" VEL:", 5);
        USART1_write(&VEL, 1);
        USART1_write("\r\n", 2);
            if(BT_Data == 'M') {
                mode = 'M';
            }
            if(mode == 'M') {
                GPIO_write(GPIOA, LED_PIN, 1);
                if (BT_Data == '>'){
                    i++;
                    if(i>4) i=4;
                    PWM_duty(PWM_PIN1, vel[i]);
                    PWM_duty(PWM_PIN2, vel[i]);
                }
                else if (BT_Data == '<'){
                    i--;
                    if(i<0) i=0;
                    PWM_duty(PWM_PIN1, vel[i]);
                    PWM_duty(PWM_PIN2, vel[i]);
                }
                else if (BT_Data == 'd') {
                    str_level--;
                    str_angle(str_level);
                    PWM_duty(PWM_PIN1, vel1);
                    PWM_duty(PWM_PIN2, vel2);
                } else if (BT_Data == 'a') {
                    str_level++;
                    str_angle(str_level);
                    PWM_duty(PWM_PIN1, vel1);
                    PWM_duty(PWM_PIN2, vel2);
                } else if (BT_Data == 'w') {
                    str_level = 0;
                    dir = F;
                    GPIO_write(GPIOC, DIR_PIN1, dir);
                    GPIO_write(GPIOC, DIR_PIN2, dir);
                    PWM_duty(PWM_PIN1, vel[i]);
                    PWM_duty(PWM_PIN2, vel[i]);
                }
                else if (BT_Data == 's') {
                    str_level = 0;
                    dir = B;
                    GPIO_write(GPIOC, DIR_PIN1, dir);
                    GPIO_write(GPIOC, DIR_PIN2, dir);
                    PWM_duty(PWM_PIN1, 0.5);
                    PWM_duty(PWM_PIN2, 0.5);
                }
                else if (BT_Data == 'E'){
                    dir = F;
                    GPIO_write(GPIOC, DIR_PIN1, dir);
                    GPIO_write(GPIOC, DIR_PIN2, dir);
                    PWM_duty(PWM_PIN1, 1);
                    PWM_duty(PWM_PIN2, 1);
                }
            }
        }
}

double str_angle(int str_level){
    if(str_level == -1){
        vel1 = v2;
        vel2 = v1;
        STR = '-1';
    }
    else if(str_level == -2){
        vel1 = v2;
        vel2 = v0;
        STR = '-2';
    }
    else if(str_level == -3){
        vel1 = v3;
        vel2 = v0;
        STR = '-3';
    }
    else if(str_level == 1){
        vel1 = v1;
        vel2 = v2;
        STR = '1';
    }
    else if(str_level == 2){
        vel1 = v0;
        vel2 = v2;
        STR = '2';
    }else if(str_level == 3){
        vel1 = v0;
        vel2 = v3;
        STR = '3';
    }
    else if(str_level == 0){
        vel1 = v0;
        vel2 = v0;
        STR = '00';
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
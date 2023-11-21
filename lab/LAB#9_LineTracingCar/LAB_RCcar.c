/**
******************************************************************************
* @author   2023-10-31 by GH An
* @brief   Embedded Controller:  LAB - USART-Bluetooth
*
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecSTM32F411.h"


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

//IR parameter//
uint32_t value1, value2;
int flag = 0;
PinName_t seqCHn[2] = {PB_0, PB_1};

static volatile uint8_t PC_Data = 0;
static volatile uint8_t BT_Data = 0;

int i=0;
char mode;
double vel[4] = {v0, v1, v2, v3};
int str_level = 0;
double vel1 = 1;
double vel2 = 1;
char DIR;
char VEL;
char STR;
uint8_t dir = 1;

void setup(void);
double str_angle(int str_level);
void speedUP();
void speedDOWN();
void M_right();
void M_left();
void M_straight();
void E_stop();
void M_back();
void LED_toggle();
void main(){
    setup();
    GPIO_write(GPIOC, DIR_PIN1, dir);
    GPIO_write(GPIOC, DIR_PIN2, dir);
    PWM_duty(PWM_PIN1, vel1);
    PWM_duty(PWM_PIN2, vel2);
    while(1){

			if(mode == 'A'){
				LED_toggle();
				if(value1 < 1000 && value2 < 1000){
                    vel1 = vel[3];
                    vel2 = vel[3];
				}
				else if(value1 > 1000 && value2 < 1000){
                    vel1 = vel[3];
                    vel2 = vel[2];
				}
				else if(value1 < 1000 && value2 > 1000){
                    vel1 = vel[2];
                    vel2 = vel[3];
				}
				else if(value1 > 1000 && value2 > 1000){
                    vel1 = 1;
                    vel2 = 1;
				}
                PWM_duty(PWM_PIN1, vel1);
                PWM_duty(PWM_PIN2, vel2);
			}
            if(mode == 'M'){
                GPIO_write(GPIOA, LED_PIN, 1);
                GPIO_write(GPIOC, DIR_PIN1, dir);
                GPIO_write(GPIOC, DIR_PIN2, dir);
                PWM_duty(PWM_PIN1, vel1);
                PWM_duty(PWM_PIN2, vel2);
            }
    }
}
void USART1_IRQHandler(){                       // USART2 RX Interrupt : Recommended
    if(is_USART1_RXNE()){
        BT_Data = USART1_read();
        USART1_write(&BT_Data,1);

        if(BT_Data == 'M') {
            mode = 'M';
        }
        else if(BT_Data == 'A'){
            mode = 'A';
        }
        if(mode == 'M') {
            if (BT_Data == '>'){
                speedUP();
            }
            else if (BT_Data == '<'){
                speedDOWN();
            }
            else if (BT_Data == 'd') {
                M_right();
            } else if (BT_Data == 'a') {
                M_left();
            } else if (BT_Data == 'w') {
                M_straight();
            }
            else if (BT_Data == 's') {
                M_back();
            }
            else if (BT_Data == 'E'){
                E_stop();
            }
        }
    }
}

void ADC_IRQHandler(void){
    if(is_ADC_OVR())
        clear_ADC_OVR();

    if(is_ADC_EOC()){      // after finishing sequence
        if (flag==0)
            value1 = ADC_read();
        else if (flag==1)
            value2 = ADC_read();
        flag =! flag;      // flag toggle				
    }
}

void speedUP(){
    i++;
    if(i>4) i=4;
    vel1 = vel[i];
    vel2 = vel[i];
}

void speedDOWN(){
    i--;
    if(i<0) i=0;
    vel1 = vel[i];
    vel2 = vel[i];
}

void M_right(){
    str_level--;
    str_angle(str_level);
}

void M_left(){
    str_level++;
    str_angle(str_level);
}

void M_straight(){
    str_level = 0;
    dir = F;
    GPIO_write(GPIOC, DIR_PIN1, dir);
    GPIO_write(GPIOC, DIR_PIN2, dir);
    vel1 = vel[i];
    vel2 = vel[i];
}

void M_back(){
    str_level = 0;
    dir = B;
    PWM_duty(PWM_PIN1, 0.5);
    PWM_duty(PWM_PIN2, 0.5);
}

void E_stop(){
    dir = F;
    vel1 = 1;
    vel2 = 1;
}

double str_angle(int str_level){
    if(str_level == -1){
        vel1 = v2;
        vel2 = v1;
    }
    else if(str_level == -2){
        vel1 = v2;
        vel2 = v0;
    }
    else if(str_level == -3){
        vel1 = v3;
        vel2 = v0;
    }
    else if(str_level == 1){
        vel1 = v1;
        vel2 = v2;
    }
    else if(str_level == 2){
        vel1 = v0;
        vel2 = v2;
    }else if(str_level == 3){
        vel1 = v0;
        vel2 = v3;
    }
    else if(str_level == 0){
        vel1 = v0;
        vel2 = v0;
    }
}

void LED_toggle(void){
		static unsigned int out = 0;
		if(out == 0) out = 1;
		else if(out == 1) out = 0;
		GPIO_write(GPIOA, LED_PIN, out);
}

void setup(void){
    RCC_PLL_init();
    SysTick_init();                     // SysTick Init
		UART2_init();
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

    // ADC Init
    ADC_init(PB_0);
    ADC_init(PB_1);

    // ADC channel sequence setting
    ADC_sequence(seqCHn, 2);

    // PWM1
    PWM_init(PWM_PIN1);
    PWM_period_us(PWM_PIN1, period);

    // PWM2
    PWM_init(PWM_PIN2);
    PWM_period_us(PWM_PIN2, period);

}
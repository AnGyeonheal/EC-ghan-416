/**
******************************************************************************
* @author   2023-10-31 by GH An
* @brief   Embedded Controller:  LAB - USART-Bluetooth
*
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecSTM32F411.h"
#include "math.h"
#include "stdio.h"
// Pin define
#define DIR_PIN1 2
#define DIR_PIN2 3
PinName_t PWM_PIN1 = PA_0;
PinName_t PWM_PIN2 = PA_1;
#define TRIG PA_6
#define ECHO PB_6

// Velocity, Direction define
#define EX 1
#define v0 0.7
#define v1 0.5
#define v2 0.25
#define v3 0
#define F 1
#define B 0

// PWM period define
float period = 6;

// TIM4 count define
uint32_t _count = 0;

// UltraSonic parameter define
uint32_t ovf_cnt = 0;
float distance = 0;
float timeInterval = 0;
float time1 = 0;
float time2 = 0;

// IR parameter define
uint32_t value1, value2;
int flag = 0;
PinName_t seqCHn[2] = {PB_0, PB_1};

// USART1(Bluetooth) parameter define
static volatile uint8_t PC_Data = 0;
static volatile uint8_t BT_Data = 0;

// Other parameter define
int i=0;		// speed level
char mode;		// mode = 'Manual' or 'Auto'	
double vel[4] = {v0, v1, v2, v3};	// velocity levels
int str_level = 0;	// Steer level
double vel1 = 1;	// 1st DC motor duty ratio
double vel2 = 1;	// 2nd DC motor duty ratio
uint8_t dir = 1;	// Direction

// char for printState
char DIR;		 
char VEL[2];	 
char STR[2];

// Function Defines
void setup(void);
double str_angle(int str_level);
void printState(void);
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
		// Initial State (STOP)
    GPIO_write(GPIOC, DIR_PIN1, dir);
    GPIO_write(GPIOC, DIR_PIN2, dir);
    PWM_duty(PWM_PIN1, vel1);
    PWM_duty(PWM_PIN2, vel2);
    while(1){
        if(mode == 'A'){	// Auto mode
            distance = (float) timeInterval * 340.0 / 2.0 / 10.0; 	// [mm] -> [cm]
            if(value1 < 1000 && value2 < 1000){	// Move Straight
                vel1 = 0;
                vel2 = 0;
            }
            else if(value1 > 1000 && value2 < 1000){	// Turn right
                vel1 = 0;
                vel2 = 0.54;
            }
            else if(value1 < 1000 && value2 > 1000){	// Turn left
                vel1 = 0.54;
                vel2 = 0;			
            }
            else if(value1 > 1000 && value2 > 1000){	// STOP
                vel1 = 1;
                vel2 = 1;
            }
						if(distance < 7){	// Obstacle detected
							E_stop();					
						}
						else if(distance > 3000){	// Ignoring dummy value
							continue;
						}
						if(_count >= 1){	// printing state, toggling every 1 second
							LED_toggle();
							printState();
							_count = 0;
						}
					// DC motor operate
					GPIO_write(GPIOC, DIR_PIN1, dir);
					GPIO_write(GPIOC, DIR_PIN2, dir);
					PWM_duty(PWM_PIN1, vel1);
					PWM_duty(PWM_PIN2, vel2);
        }
        if(mode == 'M'){	// Manual mode
					if((_count >= 2) &(BT_Data!='E')){	// printing state every 1 second
						printState();
						_count = 0;
					}
					GPIO_write(GPIOA, LED_PIN, 1);	// LED ON
					// DC motor operate
					GPIO_write(GPIOC, DIR_PIN1, dir);
					GPIO_write(GPIOC, DIR_PIN2, dir);
					PWM_duty(PWM_PIN1, vel1);
					PWM_duty(PWM_PIN2, vel2);
        }
    }
}
void USART1_IRQHandler(){                       // USART2 RX Interrupt : Recommended
    if(is_USART1_RXNE()){
        BT_Data = USART1_read();		// Send Data PC to Bluetooth
        if(BT_Data == 'M') {				// Manual mode
						USART1_write("Manual Mode\r\n",13);
            mode = 'M';
        }
        else if(BT_Data == 'A'){		// Auto mode
						USART1_write("Auto Mode\r\n",11);
            mode = 'A';
        }
        if(mode == 'M') {
            if (BT_Data == '>'){		// Speed Up
                speedUP();
            }
            else if (BT_Data == '<'){		// Speed Down
                speedDOWN();
            }
            else if (BT_Data == 'd') {		// Turn right
                M_right();
            } else if (BT_Data == 'a') {	// Turn left
                M_left();
            } else if (BT_Data == 'w') {	// Move straight
                M_straight();
            }
            else if (BT_Data == 's') {		// Move back
                M_back();
            }
            else if (BT_Data == 'E'){			// Emergency STOP
                E_stop();
								USART1_write("Emergency\r\n", 11);
            }
        }
    }
}
// Print the state (Manual or Auto mode)
void printState(void){
	if(mode == 'M'){	// Manual Mode
		// Changes int to char
		sprintf(VEL, "%d", i);
		sprintf(STR, "%d", str_level);
		// Print state on PC screen
		USART1_write("MOD: ", 5);
		USART1_write(&mode, 1);
		USART1_write(" DIR: ", 6);
		USART1_write(&DIR, 1);
		USART1_write(" STR: ", 6);
		USART1_write(&STR, 2);
		USART1_write(" VEL: ", 6);
		if(str_level == 0){
		USART1_write("V", 1);
		USART1_write(&VEL, 2);
		}
		USART1_write("\r\n", 2);
	}
	else if(mode == 'A'){	// Automation mode
		if(distance < 7){
			USART1_write("Obstacle Infront\r\n", 18);
		}
		else{
			if(value1 < 1000 && value2 < 1000){
				USART1_write("Straight\r\n",10);
			}
			else if(value1 > 1000 && value2 < 1000){
				USART1_write("Turn right\r\n", 13);
			}
			else if(value1 < 1000 && value2 > 1000){
				USART1_write("Turn left\r\n", 12);
			}
		}
	}
}
// IR sensor Handler
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
// TIM4 Handler (Ultra Sonic)
void TIM4_IRQHandler(void){
    if(is_UIF(TIM4)){                     // Update interrupt
        ovf_cnt++;													// overflow count
				_count++;														// count for 1sec
        clear_UIF(TIM4);  							    // clear update interrupt flag
    }
    if(is_CCIF(TIM4, 1)){ 								// TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
        time1 = TIM4->CCR1;									// Capture TimeStart
        clear_CCIF(TIM4, 1);                // clear capture/compare interrupt flag
    }
    else if(is_CCIF(TIM4, 2)){ 									// TIM4_Ch2 (IC2) Capture Flag. Falling Edge Detect
        time2 = TIM4->CCR2;									// Capture TimeEnd
        timeInterval = ((time2 - time1) + (TIM4->ARR+1) * ovf_cnt) * 0.01; 	// (10us * counter pulse -> [msec] unit) Total time of echo pulse
        ovf_cnt = 0;                        // overflow reset
        clear_CCIF(TIM4,2);								  // clear capture/compare interrupt flag
    }
}
void speedUP(){
    i++;
    if(i>=3) i=3;
    vel1 = vel[i];
    vel2 = vel[i];
}
void speedDOWN(){
    i--;
    if(i<=0) i=0;
    vel1 = vel[i];
    vel2 = vel[i];
}
void M_right(){
    str_level--;
		if(str_level<-3) str_level=-3;
    str_angle(str_level);
}
void M_left(){
    str_level++;
		if(str_level>3) str_level=3;
    str_angle(str_level);
}
void M_straight(){
    str_level = 0;
    dir = F;
    vel1 = vel[i];
    vel2 = vel[i];
		DIR = 'F';
}
void M_back(){
    str_level = 0;
    dir = B;
    vel1 = vel[1];
		vel2 = vel[1];
		DIR = 'B';
}
void E_stop(){
    dir = F;
    vel1 = EX;
    vel2 = EX;
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
			vel1 = vel[i];
			vel2 = vel[i];
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
    PWM_period_ms(PWM_PIN1, period);

    // PWM2
    PWM_init(PWM_PIN2);
    PWM_period_ms(PWM_PIN2, period);

    // PWM configuration ---------------------------------------------------------------------
    PWM_init(TRIG);			// PA_6: Ultrasonic trig pulse
    PWM_period_us(TRIG, 50000);    // PWM of 50ms period. Use period_us()
    PWM_pulsewidth_us(TRIG, 10);   // PWM pulse width of 10us

    // Input Capture configuration -----------------------------------------------------------------------
    ICAP_init(ECHO);    	// PB_6 as input caputre
    ICAP_counter_us(ECHO, 10);   	// ICAP counter step time as 10us
    ICAP_setup(ECHO, 1, IC_RISE);  // TIM4_CH1 as IC1 , rising edge detect
    ICAP_setup(ECHO, 2, IC_FALL);  // TIM4_CH2 as IC2 , falling edge detect

}
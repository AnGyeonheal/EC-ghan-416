#include "ecSTM32F411.h"
// Functions
void setup(void);
void TIM3_IRQHandler(void);
void motor_init(void);
void motor_stop(int motor);
void motor_operate(int motor, int dir);
// ARM Part
PinName_t PWM_PIN1 = PA_0;
#define DIR_PIN1 3
PinName_t PWM_PIN2 = PA_1;
#define DIR_PIN2 2
PinName_t PWM_PIN3 = PB_10;
#define DIR_PIN3  4
float period = 1;
float duty1;
float duty2;
float duty3;
int dir1 = 0;
int dir2 = 0;
int dir3 = 0;

// stepper motor
#define A 8
#define B 4
#define NA 5
#define NB 3
int RPM = 2;

// UltraSonic parameter define
#define TRIG PA_6
#define ECHO PB_6
uint32_t ovf_cnt = 0;
float distance = 0;
float timeInterval = 0;
float time1 = 0;
float time2 = 0;

// Buzzer define
#define BUZ_PIN

// Variables
int input;
char mode;
static int stop = 0;
uint32_t count = 0;
int temp = 0;
int operation = 0;
int flag = 0;
// Bluetooth Data
static volatile uint8_t BT_Data = 0;

int main(void) {
    // Initialization --------------------------------------------------
    setup();
    motor_init();
    // Infinite Loop ---------------------------------------------------
    while (1) {
        motor_init();
        if(mode == 'A'){
            distance = (float) timeInterval * 340.0 / 2.0 / 10.0; 	// [mm] -> [cm]
            if(distance > 4){
                motor_operate(1, 1);
                motor_stop(2);
                motor_stop(3);
                temp = 1;
            }
            else{
                temp = 0;
                // TIM_UI_disable(TIM4);
                // load
                // first step : 중간 모터 작동
                motor_stop(1);
                motor_operate(2, 0);
                delay_ms(5000);
                // second step : 삽 모터 작동
                motor_stop(2);
                motor_operate(3,0);
                delay_ms(5000);
                // third step 첫 번째 모터 복귀
                flag = 1;
                while(count > 0){
                    motor_operate(1, 0);
                    motor_stop(3);  
                }
                flag = 0;
                // rotate

                // unload
                // firth step0 : 중간 모터 작동
                motor_stop(1);
                motor_operate(2, 1);
                delay_ms(5000);
                // sixth step : 삽 모터 작동
                motor_stop(2);
                motor_operate(3,1);
                delay_ms(5000);
            }
        }
        else if(mode == 'M'){
            // First
            GPIO_write(GPIOC, DIR_PIN1, dir1);
            PWM_duty(PWM_PIN1, duty1);
            // Second
            GPIO_write(GPIOC, DIR_PIN2, dir2);
            PWM_duty(PWM_PIN2, duty2);
            // shovel
            GPIO_write(GPIOC, DIR_PIN3, dir3);
            PWM_duty(PWM_PIN3, duty3);
        }
        USART1_write(&BT_Data, 1);
				printf("%f cm\r\n", distance);
				printf("%d\r\n",count);
    }
}

void USART1_IRQHandler(){
    if(is_USART1_RXNE()){
        BT_Data = USART1_read();
        // First
        if(BT_Data == 'z') input = 10;
        else if(BT_Data == 'x') input = 15;
        else if(BT_Data == 'c') input = 20;

        if (BT_Data == 'M'){
            mode = 'M';
        }
        else if (BT_Data == 'A'){
            mode = 'A';
        }

        if(mode == 'A'){
            if(BT_Data == 'a'){
                Stepper_step(200, 0, FULL);
            }
            else if(BT_Data == 'd'){
                Stepper_step(200, 1, FULL);
            }
        }

        else if(mode == 'M'){
            if(BT_Data == 'u'){
                dir1 = 0;
                duty1 = 1;
            }
            else if(BT_Data == 'j'){
                dir1 = 1;
                duty1 = 0;
            }
            else if(BT_Data == 'i'){
                dir2 = 0;
                duty2 = 1;
            }
            else if(BT_Data == 'k'){
                dir2 = 1;
                duty2 = 0;
            }
            else if(BT_Data == 'o'){
                dir3 = 0;
                duty3 = 1;
            }
            else if(BT_Data == 'l'){
                dir3 = 1;
                duty3 = 0;
            }
            else if(BT_Data == 's'){
                dir1 = dir2 = dir3 = 0;
                duty1 = duty2 = duty3 = 0;
            }
        }
    }
}
void motor_init(void){
    GPIO_write(GPIOC, DIR_PIN1, 0);
    PWM_duty(PWM_PIN1, 0);
    GPIO_write(GPIOC, DIR_PIN2, 0);
    PWM_duty(PWM_PIN2, 0);
    GPIO_write(GPIOC, DIR_PIN3, 0);
    PWM_duty(PWM_PIN3, 0);
}

void motor_stop(int motor){
    int dir_pin;
    int motor_pin;
    if(motor == 1){
        dir_pin = DIR_PIN1;
        motor_pin = PWM_PIN1;
    }
    else if(motor == 2){
        dir_pin = DIR_PIN2;
        motor_pin = PWM_PIN2;
    }
    else if(motor == 3){
        dir_pin = DIR_PIN3;
        motor_pin = PWM_PIN3;
    }
    GPIO_write(GPIOC, dir_pin, 0);
    PWM_duty(motor_pin, 0);
}

void motor_operate(int motor, int dir){
    int dt;
    int dir_pin;
    int motor_pin;
    if(motor == 1){
        dir_pin = DIR_PIN1;
        motor_pin = PWM_PIN1;
    }
    else if(motor == 2){
        dir_pin = DIR_PIN2;
        motor_pin = PWM_PIN2;
    }
    else if(motor == 3){
        dir_pin = DIR_PIN3;
        motor_pin = PWM_PIN3;
    }
    GPIO_write(GPIOC, dir_pin, dir);
    if(dir == 1) dt = 0;
    else if(dir == 0) dt = 1;
    PWM_duty(motor_pin, dt);
}

void TIM4_IRQHandler(void){
    if(is_UIF(TIM4)){                     // Update interrupt
        ovf_cnt++;													// overflow count	        // count for 1sec
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


void TIM3_IRQHandler(void) {
    if (is_UIF(TIM3)) {            // Check UIF(update interrupt flag)
			if(temp == 1){
				count++;
			}
            if(flag == 1){
                count--;
            }
    }
    clear_UIF(TIM3);            // Clear UI flag by writing 0
}

// Initialiization
void setup(void) {
    RCC_PLL_init();

    // SYSTICK
    SysTick_init();

    // Bluetooth serial init
    UART1_init();
    UART1_baud(BAUD_9600);
    UART2_init();

    // DIR1
    GPIO_init(GPIOC, DIR_PIN1, OUTPUT);
    GPIO_otype(GPIOC, DIR_PIN1, EC_PUSH_PULL);
    // DIR2
    GPIO_init(GPIOC, DIR_PIN2, OUTPUT);
    GPIO_otype(GPIOC, DIR_PIN2, EC_PUSH_PULL);
    // DIR3
    GPIO_init(GPIOC, DIR_PIN3, OUTPUT);
    GPIO_otype(GPIOC, DIR_PIN3, EC_PUSH_PULL);

    // Stepper motor
    Stepper_init(GPIOA, A, GPIOB, B, GPIOB, NA, GPIOB, NB);
    Stepper_setSpeed(RPM);

    // TIM
    TIM_UI_init(TIM3, 1);

    // ARM Part
    // PWM1
    PWM_init(PWM_PIN1);
    PWM_period_ms(PWM_PIN1, period);
    // PWM2
    PWM_init(PWM_PIN2);
    PWM_period_ms(PWM_PIN2, period);
    // PWM3
    PWM_init(PWM_PIN3);
    PWM_period_ms(PWM_PIN3, period);

    // Input Capture configuration -----------------------------------------------------------------------
    ICAP_init(ECHO);    	// PB_6 as input caputre
    ICAP_counter_us(ECHO, 10);   	// ICAP counter step time as 10us
    ICAP_setup(ECHO, 1, IC_RISE);  // TIM4_CH1 as IC1 , rising edge detect
    ICAP_setup(ECHO, 2, IC_FALL);  // TIM4_CH2 as IC2 , falling edge detect
}
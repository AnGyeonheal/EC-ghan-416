# LAB: Line Tracing RC Car

**Date:** 2023-11-24

**Author/Partner: **GyeonhealAn / HanminKim

**Github:** https://github.com/AnGyeonheal/Embedded_Control_GH

**Demo Video:** 

## I. Introduction

Design an embedded system to control an RC car to drive on the racing track. The car is controlled either manually with wireless communication or automatically to drive around the track. When it sees an obstacle on the driving path, it should temporarily stop until the obstacle is out of the path.

> There can be more missions to complete.

![img](https://424033796-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2F-MgmrEstOHxu62gXxq1t%2Fuploads%2F2anmcAqBTR8yESOPG5Qi%2Fimage.png?alt=media&token=fe7b1120-ff25-49c4-97dc-a7a7202a394a)

### i. Requirement

**Hardware**

- MCU
  - NUCLEO-F411RE
- Actuator/Sensor/Others: Minimum
  - Bluetooth Module(HC-06)
  - DC motor x2, DC motor driver(L9110s)
  - IR Reflective Sensor (TCRT 5000) x2
  - HC-SR04
  - additional sensor/actuators are acceptable

**Software**

- Keil uVision, CMSIS, EC_HAL library

## II. Problem Definition

Design your RC car that has the following functions:

1. Line tracing on the given racing track
2. has 2 control modes: **Manual Mode** to **AUTO Mode**
3. stops temporally when it detects an object nearby on the driving path

- On the PC, connected to MCU via bluetooth

- Print the car status every 1 sec such as “ ( “ MOD: A DIR: F STR: 00 VEL: 00 ”)

### i. Manual Mode

- Mode Change( MOD):
  - When 'M' or 'm' is pressed, it should enter **Manual Mode**
  - LD2 should be ON in Manual Mode
- Speed (VEL):
  - Increase or decrease speed each time you push the arrow key “UP” or “DOWN”, respectively.
  - You can choose the speed keys
  - Choose the speed level: V0 ~ V3
- Steer (STR):
  - Steering control with keyboard keys
  - Increase or decrease the steering angles each time you press the arrow key “RIGHT” or “LEFT”, respectively.
  - Steer angles with 3 levels for both sides: e.g: -3, -2, -1, 0, 1, 2, 3 // '-' angle is turning to left
- Driving Direction (DIR)
  - Driving direction is forward or backward by pressing the key “F” or “B”, respectively.
  - You can choose the control keys
- Emergency Stop
  - RC car must stop running when key “S” is pressed.

### ii. Automatic Mode

- Mode Change:
  - When 'A' or 'a' is pressed, it should enter **AUTO Mode**
- LD2 should blink at 1 second rate in AUTO Mode
- It should drive on the racing track continuously
- Stops temporall when it detects an object nearby on the driving path
- If the obstacle is removed, it should drive continuously

## III. Procedure

1. Discuss with the teammate how to design an algorithm for this problem

2. In the report, you need to explain concisely how your system works with state tables/diagram or flow-chart.

   ● Listing all necessary states (states, input, output etc) to implement this design problem.

   ● Listing all necessary conditional FLAGS for programming.

   ● Showing the logic flow from the initialization

   and more

3. Select appropriate configurations for the design problem. Fill in the table.

| **Functions**             | **Register** | **PORT_PIN**       | **Configuration**                                     |
| ------------------------- | ------------ | ------------------ | ----------------------------------------------------- |
| System Clock              | RCC          |                    | PLL 84MHz                                             |
| delay_ms                  | SysTick      |                    |                                                       |
| Motor DIR                 | Digital Out  |                    |                                                       |
|                           | ….           |                    |                                                       |
| TIMER                     | TIMER1       |                    |                                                       |
|                           | TIMER2       |                    |                                                       |
| Timer Interrupt           | ...          |                    | 10msec                                                |
| ADC                       | ADC          |                    |                                                       |
|                           | ….           |                    |                                                       |
| DC Motor Speed            | PWM2         |                    |                                                       |
| ADC sampling trigger      | PWM3         |                    |                                                       |
| RS-232 USB cable(ST-LINK) | USART2       |                    | No Parity, 8-bit Data, 1-bit Stop bit 38400 baud-rate |
| Bluetooth                 | USART1       | TXD: PA9 RXD: PA10 | No Parity, 8-bit Data, 1-bit Stop bit 9600 baud-rate  |

### i. Flow Chart

![image](https://github.com/AnGyeonheal/Embedded_Control_GH/assets/118132313/fda08802-138c-4213-b245-d9bb4f2bdd62)

### ii. Circuit Diagram

> You need to include the circuit diagram

![img](https://user-images.githubusercontent.com/38373000/192134563-72f68b29-4127-42ac-b064-2eda95a9a52a.png)

### ii. Code

Your code goes here: [GITHUB](https://github.com/AnGyeonheal/Embedded_Control_GH/tree/main/lab)

**Parameter Definition**

```c
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
```

This is parameter definition. I set velocity as four levels. The DC motor we used has its maximum speed when the direction is set to 1 and the duty cycle is 0, and its minimum speed occurs when the duty cycle is 1.

**Main Function**

```c
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
                vel2 = 0.6;
            }
            else if(value1 < 1000 && value2 > 1000){	// Turn left
                vel1 = 0.6;
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
```

Main function은 Pin들을 initializing하는 setup() 함수를 실행시키고, 초기 모터 상태인 정지상태를 출력합니다. loop 안에는 Automation mode 일때와 Manual mode일 때로 나눈다. Auto mode 일때는 2개의 IR sensor를 통해 얻은 value1, value2 값을 비교하여 Line Tracing 기능을 수행한다. 또한 Ultra Sonic sensor를 통해 들어온 distance 값이 7cm 보다 작은 값이 들어왔을 때 RC car가 정지하도록 수행한다. 이 때 distance 값이 sensing 오류로 인해 매우 큰 값이 입력되는 현상이 생겼다. 이를 방지하기 위해 3000cm 이상의 distance 값이 들어왔을 때 이를 무시하게 하였다. 그리고 TIM4를 통해 얻은 _count 값으로 매 초 MCU의 LED가 반짝이며 현 자동차의 상태를 출력하도록 하였다. 마지막으로는 DC motor가 위에서 얻은 dir, vel1, vel2 값을 통해 dc motor를 가동한다.

**USART1_IRQHandler**

```c
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
```

이 함수는 Bluetooth를 통해 들어온 값이 감지 되었을 때 함수 내 명령을 수행한다. USART1_read() 함수를 통해 BT_Data 값을 얻고 mode를 결정한다. Auto mode는 main function 내에서 수행하였다면 Manual mode는 해당 함수 내에서 수행한다. 자동차의 속도를 높이고 낮추며 steering angle을 -3부터 3까지 조절할 수 있게 한다. 직진, 후진 기능을 수행하고 "E"를 읽으면 Emergency Stop 기능을 수행한다.

**IR sensor**

```c
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
```

해당 함수는 ADC Handler 기능을 통해 IR sensor의 값을 읽어온다. (value1, value2)

**TIM4_IRQHanldler()**

```c
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
```

이 함수는 Ultra Sonic의 

**printState()**

```c
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
```

이 함수는 Manual mode일 때와 Auto mode일 때의 상태를 Bluetooth를 통해 PC로 보내 출력한다.

USART1_write() 함수를 사용하여 화면으로 출력하였으며 이는 문자열밖에 전송이 되지 않으므로 sprintf()를 사용하여 속도, 조향 각도 값을 문자열로 바꿔 출력하도록 하였다.



**Auto Mode**

## IV. Results

[demo video link]()

## V. Reference

Complete list of all references used (github, blog, paper, etc)



## VI. Troubleshooting

### i. motor PWM duty ratio for different DIR

When, DIR=0 duty=0.8--> PWM 0.8 // 실제 모터에 전달되는 pwm

Whe, DIR=1 duty=0.8--> PWM 0.2 // 실제 모터에 전달되는 PWM

*** a solution ***

```c
float targetPWM;  // pwm for motor input 
float duty=abs(DIR-targetPWM); // duty with consideration of DIR=1 or 0

PWM_duty(PWM_PIN, duty);
```

### ii. Print a string for BT (USART1)

Use `sprintf()`

```c
#define _CRT_SECURE_NO_WARNINGS    // sprintf 보안 경고로 인한 컴파일 에러 방지
#include <stdio.h>     // sprintf 함수가 선언된 헤더 파일

char BT_string[20]=0;

int main()
{
	sprintf(BT_string, "DIR:%d PWM: %0.2f\n", dir, duty);    // 문자, 정수, 실수를 문자열로 만듦
	USART1_write(BT_string, 20);
	// ...
}
```

https://dojang.io/mod/page/view.php?id=352 **

### iii. Motor does not run under duty 0.5

SOL) Configure motor PWM period as 1kHa

### iv. Check and give different Interrupt Priority

Check if you have different NVIC priority number for each IRQs

### v. Ultrasoninc sensor does not measure properly when MCU is connected with motor driver

```

```


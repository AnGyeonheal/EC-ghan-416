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

### i. Circuit Diagram

> You need to include the circuit diagram

![img](https://user-images.githubusercontent.com/38373000/192134563-72f68b29-4127-42ac-b064-2eda95a9a52a.png)

### ii. Code

Your code goes here: [GITHUB](https://github.com/AnGyeonheal/Embedded_Control_GH/tree/main/lab)

**Manual Mode : **



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


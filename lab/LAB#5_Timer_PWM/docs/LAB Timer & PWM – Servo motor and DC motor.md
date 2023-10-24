# LAB: Timer & PWM – Servo motor and DC motor

**Date:** 2023-10-23

**Author:** Gyeonheal An

**Github:** https://github.com/AnGyeonheal/Embedded_Control_GH

**Demo Video:** https://youtube.com/shorts/owh8RJOokzI?feature=shared

## I. Introduction

Create a simple program that control a sevo motor and a DC motor with PWM output.

You must submit

### i. Requirement

**Hardware**

- MCU
  - NUCLEO-F411RE
- Actuator/Sensor/Others:
  - 3 LEDs and load resistance
  - RC Servo Motor (SG90)
  - DC motor (5V)
  - DC motor driver(LS9110s)
  - breadboard

**Software**

- Keil uVision, CMSIS, EC_HAL library
- Clion(with PlatformIO core plugin)
  Library: STM32Cube library package(Official), EC_HAL library
  Compiler: GNU Arm Embedded Toolchain
  Debugger: ST-Link

## II. Problem 1: RC servo motor

An RC servo motor is a tiny and light weight motor with high output power. It is used to control rotation angles, approximately 180 degrees (90 degrees in each direction) and commonly applied in RC car, and Small-scaled robots. The angle of the motor can be controlled by the pulse width (duty ratio) of PWM signal. The PWM period should be set at **20ms or 50Hz**. Refer to the datasheet of the RC servo motor for detailed specifications.

<img src="https://user-images.githubusercontent.com/38373000/195773601-f0f19e35-0a6f-49af-aa87-574c86bfec62.png" alt="img" style="zoom: 50%;" />

### i. Create HAL library

**ecTIM.h**

```c
// Timer Period setup
void TIM_init(TIM_TypeDef *TIMx, uint32_t msec);
void TIM_period(TIM_TypeDef* TIMx, uint32_t msec);
void TIM_period_ms(TIM_TypeDef* TIMx, uint32_t msec);
void TIM_period_us(TIM_TypeDef* TIMx, uint32_t usec);
```

```c
// Timer Interrupt setup
void TIM_UI_init(TIM_TypeDef* TIMx, uint32_t msec);
void TIM_UI_enable(TIM_TypeDef* TIMx);
void TIM_UI_disable(TIM_TypeDef* TIMx);
```

```c
// Timer Interrupt Flag 
uint32_t is_UIF(TIM_TypeDef *TIMx);
void clear_UIF(TIM_TypeDef *TIMx);
```



**ecPWM.h**

```c
/* PWM Configuration using PinName_t Structure */
```

```c
/* PWM initialization */
// Default: 84MHz PLL, 1MHz CK_CNT, 50% duty ratio, 1msec period
void PWM_init(PinName_t pinName);
void PWM_pinmap(PinName_t pinName, TIM_TypeDef **TIMx, int *chN);
```

```c
/* PWM PERIOD SETUP */
// allowable range for msec:  1~2,000
void PWM_period(PinName_t pinName,  uint32_t msec);	
void PWM_period_ms(PinName_t pinName,  uint32_t msec);	// same as PWM_period()
```

```c
// allowable range for usec:  1~1,000
void PWM_period_us(PinName_t pinName, uint32_t usec);
```

```c
/* DUTY RATIO SETUP */
// High Pulse width in msec
void PWM_pulsewidth(PinName_t pinName, uint32_t pulse_width_ms);
void PWM_pulsewidth_ms(PinName_t pinName, uint32_t pulse_width_ms);  // same as void PWM_pulsewidth
```

```c
// Duty ratio 0~1.0
void PWM_duty(PinName_t pinName, float duty);
```

### ii. Procedure

Make a simple program that changes the angle of the RC servo motor that rotates back and forth from 0 deg to 180 degree within a given period of time.

Reset to '0' degree by pressing the push button (PC13).

- Button input has to be an External Interrupt
- Use Port A Pin 1 as PWM output pin for TIM2_CH2.
- Use Timer interrupt of period 500msec.
- Angle of RC servo motor should rotate from 0° to 180° and back 0° at a step of 10° at the rate of 500msec.

You need to observe how the PWM signal output is generated as the input button is pushed, using an oscilloscope. You need to capture the Oscilloscope output in the report.

1. Connect the RC servo motor to MCU pin (PA1) , VCC and GND
2. Increase the angle of RC servo motor from 0° to 180° with a step of 10° every 500msec. After reaching 180°, decrease the angle back to 0°. Use timer interrupt IRQ.
3. When the button is pressed, it should reset to the angle 0° and start over. Use EXT interrupt.

### iii. Configuration

| Type                | Port - Pin        | Configuration                                      |
| :------------------ | :---------------- | :------------------------------------------------- |
| **Button**          | Digital In (PC13) | Pull-Up                                            |
| **PWM Pin**         | AF (PA1)          | Push-Pull, Pull-Up, Fast                           |
| **PWM Timer**       | TIM2_CH2 (PA1)    | TIM2 (PWM) period: 20msec, Duty ratio: 0.5~2.5msec |
| **Timer Interrupt** | TIM3              | TIM3 Period: 1msec, Timer Interrupt of 500 msec    |

### iv. Circuit Diagram



### v. Discussion

1. **Derive a simple logic to calculate CRR and ARR values to generate x[Hz] and y[%] duty ratio of PWM. How can you read the values of input clock frequency and PSC?**

   

2. **What is the smallest and highest PWM frequency that can be generated for Q1?**



### vi. Code

```

```





### vii. Results



## III. Problem 2: DC motor

### i. Procedure

Make a simple program that rotates a DC motor that changes the duty ratio from 25% -->75%--> 25% --> and so on.

The rotating speed level changes every 2 seconds.

By pressing the push button (PC13), toggle from Running and stopping the DC motor

1. Connect DC motor and DC motor driver.

- PA_1 for the DC motor PWM
- PC_2 for Direction Pin

2. Change DC motor from LOW Speed to HIGH Speed for every 2 seconds

- e.g. 25% -->75%--> 25% --> and so on.

3. When Button is pressed, it should PAUSE or CONTINUE motor run

### ii. Configuration

| Function            | Port - Pin        | Configuration                                   |
| :------------------ | :---------------- | :---------------------------------------------- |
| **Button**          | Digital In (PC13) | Pull-Up                                         |
| **Direction Pin**   | Digital Out (PC2) | Push-Pull                                       |
| **PWM Pin**         | AF (PA0)          | Push-Pull, Pull-Up, Fast                        |
| **PWM Timer**       | TIM2_CH1 (PA0)    | TIM2 (PWM) period: **1msec (1kHz)**             |
| **Timer Interrupt** | TIM3              | TIM3 Period: 1msec, Timer Interrupt of 500 msec |

### iii. Circuit Diagram







### iv. Results

[demo video link](https://github.com/ykkimhgu/course-doc/blob/master/ec-course/lab/link/README.md)

## IV. Reference

![image](https://github.com/AnGyeonheal/Embedded_Control_GH/assets/118132313/f9bfa75c-0131-4d28-8545-6ee9ae9e7179)

## V. Troubleshooting

(Option) You can write Troubleshooting section
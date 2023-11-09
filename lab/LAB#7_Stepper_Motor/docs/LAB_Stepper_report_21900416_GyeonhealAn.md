# LAB: Stepper Motor

**Date:** 2023-09-26

**Author**

**Github:** repository link

**Demo Video:** Youtube link

**PDF version:**

## I. Introduction

In this lab, we will learn how to drive a stepper motor with digital output of GPIOs of MCU. You will use a FSM to design the algorithm for stepper motor control.

### i. Requirement

**Hardware**

- MCU
  - NUCLEO-F411RE
- Actuator/Sensor/Others:
  - 3Stepper Motor 28BYJ-48
  - Motor Driver ULN2003
  - breadboard

**Software**

- Keil uVision, CMSIS, EC_HAL library

## II. Problem 1: Stepper Motor

### i. Hardware Connection

Read specification sheet of the motor and the motor driver for wiring and min/max input voltage/current.

![img](https://user-images.githubusercontent.com/91526930/197428440-9f4a9c8c-2d81-4d0e-a4e2-b4a4b9def44d.png)

![img](https://user-images.githubusercontent.com/91526930/197428469-a0d7a8fa-ba4c-482f-8688-ea87cfd9f4e0.png)

### ii. Stepper Motor Sequence

We will use unipolar stepper motor for this lab

**Full-stepping sequence**

![img](https://user-images.githubusercontent.com/91526930/197428513-f9a23147-3448-4bed-bda2-c90325b8c143.png)

![image](https://github.com/AnGyeonheal/Embedded_Control_GH/assets/118132313/b7a15f41-ffdc-4621-bb9e-cd94f9b7a4d7)

**Half-stepping sequence**

![img](https://user-images.githubusercontent.com/91526930/197429006-d552ab16-0bbf-4c52-bdce-a0f2bfe5f0d8.png)

![image](https://github.com/AnGyeonheal/Embedded_Control_GH/assets/118132313/b59bc243-655a-48dc-addf-d4661396cb0d)

### iii. Finite State Machine

- Full-Stepping Sequence

![image-20231109003941838](C:\Users\heal\AppData\Roaming\Typora\typora-user-images\image-20231109003941838.png)

- Half-Stepping Sequence

![image-20231109003958101](C:\Users\heal\AppData\Roaming\Typora\typora-user-images\image-20231109003958101.png)

## III. Problem 2: Firmware Programming

### i. Create HAL library

**ecStepper.h**

```c
// Initialize with 4 pins
// ( A, B,  AN,  BN)
void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4);
//or   using ecPinNames.h 
void Stepper_init(PinName_t A, PinName_t B,  PinName_t AN, PinName_t BN);
// whatSpeed [rev/min]
void Stepper_setSpeed(long whatSpeed);
// Run for n Steps
void Stepper_step(uint32_t steps, uint32_t direction, uint32_t mode); 
// Immediate Stop.
void Stepper_stop(void);
```

> Note that these are blocking stepper controllers. While the stepper is running, the MCU cannot process other polling commands. If you can, modify it to be the non-blocking controller.

> You can also create your own functions different from the given instructions.

### ii. Procedure

1. Find out the number of steps required to rotate 1 revolution using Full-steppping.
   - We need 2048 steps because the stepper motor's stride angle and gear ratio are 64 and 32 which means we need 64 x 32 steps. 
2. Then, rotate the stepper motor 10 revolutions with 2 rpm. Measure if the motor rotates one revolution per second.
3. Repeat the above process in the opposite direction.
4. Increase and decrease the speed of the motor as fast as it can rotate to find the maximum and minimum speed of the motor.
   - Maximum : 14 rpm, Minimum : 1 rpm
5. Apply the half-stepping and repeat the above.

### iii. Configuration

| Digital Out                                             | SysTick |
| :------------------------------------------------------ | :------ |
| PB10, PB4, PB5, PB3 NO Pull-up Pull-down Push-Pull Fast | delay() |

### iv. Discussion

1. Find out the trapezoid-shape velocity profile for a stepper motor. When is this profile necessary?

   > Answer discussion questions

2. How would you change the code more efficiently for micro-stepping control? You don’t have to code this but need to explain your strategy.

   > Answer discussion questions

### v. Code



### vi. Results

Experiment images and results



Add [demo video link](https://github.com/ykkimhgu/course-doc/blob/master/course/lab/link/README.md)

## IV. Reference

https://github.com/AnGyeonheal/Embedded_Control_GH

## V. Troubleshooting


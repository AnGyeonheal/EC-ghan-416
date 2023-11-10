# LAB: Stepper Motor

**Date:** 2023-11-10

**Author**: Gyeonheal An

**Github:** https://github.com/AnGyeonheal/Embedded_Control_GH

**Demo Video:** https://www.youtube.com/watch?v=A9xt91HmfZY

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

- Clion(with PlatformIO core plugin)
  Library: STM32Cube library package(Official), EC_HAL library
  Compiler: GNU Arm Embedded Toolchain
  Debugger: ST-Link

## II. Problem 1: Stepper Motor

### i. Hardware Connection

Read specification sheet of the motor and the motor driver for wiring and min/max input voltage/current.

![img](https://user-images.githubusercontent.com/91526930/197428440-9f4a9c8c-2d81-4d0e-a4e2-b4a4b9def44d.png)

![img](https://user-images.githubusercontent.com/91526930/197428469-a0d7a8fa-ba4c-482f-8688-ea87cfd9f4e0.png)

### ii. Stepper Motor Sequence

We will use unipolar stepper motor for this lab

**Full-stepping sequence**

<img src="https://user-images.githubusercontent.com/91526930/197428513-f9a23147-3448-4bed-bda2-c90325b8c143.png" alt="img" style="zoom: 80%;" />

<img src="https://github.com/AnGyeonheal/Embedded_Control_GH/assets/118132313/b7a15f41-ffdc-4621-bb9e-cd94f9b7a4d7" alt="image" style="zoom:67%;" />

**Half-stepping sequence**

<img src="https://user-images.githubusercontent.com/91526930/197429006-d552ab16-0bbf-4c52-bdce-a0f2bfe5f0d8.png" alt="img" style="zoom: 80%;" />

<img src="https://github.com/AnGyeonheal/Embedded_Control_GH/assets/118132313/b59bc243-655a-48dc-addf-d4661396cb0d" alt="image"  />

### iii. Finite State Machine

- Full-Stepping Sequence

<img src="https://github.com/AnGyeonheal/Embedded_Control_GH/assets/118132313/b05aba07-7991-467f-af2f-3ac9a4832558" alt="image" style="zoom:67%;" />

- Half-Stepping Sequence

<img src="https://github.com/AnGyeonheal/Embedded_Control_GH/assets/118132313/e7e7cc76-5eba-46e7-958b-5898ae4bd094" alt="image" style="zoom: 67%;" />

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

In the III.v.Code, when it’s executed, the MCU board initially operates within the main function. Unless there is an EXTI or Timer Interrupt event, the code inside the while loop doesn’t execute. Therefore, the Stepper_Step function runs within the while loop, and with each occurrence of a SysTick interrupt, the msTicks variable used in the delay_ms() function is brought into the main code. It is then used to count, which triggers the next state.

> You can also create your own functions different from the given instructions.

### ii. Procedure

1. Find out the number of steps required to rotate 1 revolution using Full-steppping.
2. Then, rotate the stepper motor 10 revolutions with 2 rpm. Measure if the motor rotates one revolution per second.

3. Repeat the above process in the opposite direction.
4. Increase and decrease the speed of the motor as fast as it can rotate to find the maximum and minimum speed of the motor.
5. Apply the half-stepping and repeat the above.

### iii. Configuration

| Digital Out                                             | SysTick |
| :------------------------------------------------------ | :------ |
| PB10, PB4, PB5, PB3 NO Pull-up Pull-down Push-Pull Fast | delay() |

### iv. Discussion

1. **Find out the trapezoid-shape velocity profile for a stepper motor. When is this profile necessary?**

<img src="https://github.com/AnGyeonheal/Embedded_Control_GH/assets/118132313/cf62bb8d-be55-4c72-ba28-233d0cebf8cb" alt="image" style="zoom: 80%;" />

A trapezoidal velocity profile is a common velocity profile used in motion control systems, including stepper motors. It consists of three distinct phases:

1. Acceleration: In this phase, the motor accelerates from standstill to a constant velocity. The acceleration rate is typically constant, resulting in a linear increase in velocity over time.

2. Constant Velocity: Once the desired velocity is reached, the motor maintains a constant speed, moving at a consistent rate.

3. Deceleration: In this phase, the motor decelerates to come to a stop. The deceleration rate is typically the same as the acceleration rate, resulting in a linear decrease in velocity.

This trapezoidal profile is necessary in situations where you need precise control of the motor's movement, especially when it's essential to minimize overshoot and ensure accurate positioning. Stepper motors, which move in discrete steps, can benefit from this profile to achieve smoother motion and minimize vibrations.

![image](https://github.com/AnGyeonheal/Embedded_Control_GH/assets/118132313/0238642e-a2d5-4f1d-a46c-df1859d77f94)

It's worth noting that for more advanced motion control applications, you might use other velocity profiles like S-curve or jerk-limited profiles to further improve motion quality and reduce wear and tear on the motor and mechanical components.

2. **How would you change the code more efficiently for micro-stepping control? You don’t have to code this but need to explain your strategy.**

- Fine Step Subdivision:
  - By subdividing the 4 sequences of the Full step model into finer steps, the motor's movement becomes smoother and more precise.
  - Use microstepping drivers to divide full steps into finer steps, allowing the motor to move at smaller angles.

- PID Control:
  - PID control allows for error correction and precise position control.
  - The Proportional term (P) measures the current error and adjusts the motor's speed proportionally to the error's magnitude, helping to quickly reduce the error and converge to the desired position.
  - The Integral term (I) eliminates steady-state error by compensating for long-term errors, enabling precise position control.
  - The Derivative term (D) reduces overshoot and improves stability by detecting the rate of change of previous errors, maintaining smooth motion.

<img src="https://github.com/AnGyeonheal/Embedded_Control_GH/assets/118132313/755607fb-ef05-4303-92f9-044efbf0ce39" alt="image" style="zoom:50%;" />

<img src="https://github.com/AnGyeonheal/Embedded_Control_GH/assets/118132313/70a5e408-8247-4b83-8e28-b1a9ff833478" alt="image" style="zoom:50%;" />

<img src="https://github.com/AnGyeonheal/Embedded_Control_GH/assets/118132313/7eff7d05-ac27-4e45-ab9a-9d008c7ce7ca" alt="image" style="zoom:50%;" />

### v. Code

```c
#include "ecSTM32F411.h"

#define A 10
#define B 4
#define NA 5
#define NB 3

int RPM = 2;

void setup(void);

int main(){
    setup();
    Stepper_step(2048*10, 0, FULL);
    while(1){
    }
}
```

This is the main function and it contains a function that operates the stepper motor. Factors of this function include number of steps, direction, and mode. By multiplying the number of steps by the number of revolutions, you can enter how many turns you want to rotate.

```c
void EXTI15_10_IRQHandler(void) {
    if (is_pending_EXTI(BUTTON_PIN)) {
        Stepper_stop();
        clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
    }
}
```

This function can make stepper motor stop using external interrupt (Button).

```c
void setup(void){
    RCC_PLL_init();
    SysTick_init();

    GPIO_init(GPIOC, BUTTON_PIN, INPUT);           // GPIOC pin13 initialization
    EXTI_init(GPIOC, BUTTON_PIN, FALL,15);           // External Interrupt Setting

    Stepper_init(GPIOB, A, GPIOB, B, GPIOB, NA, GPIOB, NB);
    Stepper_setSpeed(RPM);
}
```

This is initializing system clock, SysTick, Button Pin, Stepper motor.

It was initialized according to the configuration table above.

### vi. Results

<img src="https://github.com/AnGyeonheal/Embedded_Control_GH/assets/118132313/6edf0cd1-d333-4f5b-a19a-8ba5f032c40d" alt="image" style="zoom: 33%;" />

1. **Find out the number of steps required to rotate 1 revolution using Full-steppping.**
   - We need 2048 steps because the stepper motor's stride angle and gear ratio are 64 and 32 which means we need 64 x 32 steps. 
   - FULL : 2048 steps
   - HALF : 2048 * 2 steps
2. **Measure if the motor rotates one revolution per second.**
   - FULL : about 28 sec
   - HALF : about 56 sec
3. **Increase and decrease the speed of the motor as fast as it can rotate to find the maximum and minimum speed of the motor.**
   - FULL : Maximum : 14 rpm, Minimum : 1 rpm
   - HALF : Maximum : 29 rpm, Minimum : 1 rpm

Add [demo video link](https://www.youtube.com/watch?v=A9xt91HmfZY)

## IV. Reference

https://github.com/AnGyeonheal/Embedded_Control_GH

## V. Troubleshooting


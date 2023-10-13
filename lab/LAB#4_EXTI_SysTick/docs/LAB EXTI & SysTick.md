# **LAB: EXTI & SysTick**

**Date:** 2023-10-13

**Author:** Gyeonheal An

**Github:** https://github.com/AnGyeonheal/Embedded_Control_GH

**Demo Video:** Youtube link

## I. Introduction

In this lab, you are required to create two simple programs using interrupt:

(1) displaying the number counting from 0 to 9 with Button Press

(2) counting at a rate of 1 second

### i. Requirement

**Hardware**

- MCU
  - NUCLEO-F411RE
- Actuator/Sensor/Others:
  - 4 LEDs and load resistance
  - 7-segment display(5101ASR)
  - Array resistor (330 ohm)
  - breadboard

**Software**

- Keil uVision, CMSIS, EC_HAL library
- Clion(with PlatformIO core plugin)
  Library: STM32Cube library package(Official), EC_HAL library
  Compiler: GNU Arm Embedded Toolchain
  Debugger: ST-Link

## II. Problem 1: Counting numbers on 7-Segment using EXTI Button

### i. Create HAL library

**ecEXTI.h**

```c
void EXTI_init(GPIO_TypeDef *port, int pin, int trig_type, int priority);
void EXTI_enable(uint32_t pin);  // mask in IMR
void EXTI_disable(uint32_t pin);  // unmask in IMR
uint32_t  is_pending_EXTI(uint32_t pin);
void clear_pending_EXTI(uint32_t pin);
```

### ii. Procedure

1. Use the decoder chip (**74LS47**). Connect it to the bread board and 7-segment display.
2. First, check if every number, 0 to 9, can be displayed properly on the 7-segment.
3. Then, create a code to display the number counting from 0 to 9 and repeats
   - by pressing the push button. (External Interrupt)
4. You must use your library function of EXTI.
5. Refer to an [sample code](https://ykkim.gitbook.io/ec/firmware-programming/example-code#button-interrupt)

### iii. Configuration

| Digital In for Button (B1) | Digital Out for 7-Segment decoder             |
| :------------------------- | :-------------------------------------------- |
| Digital In                 | Digital Out                                   |
| PC13                       | PA7, PB6, PC7, PA9                            |
| PULL-UP                    | Push-Pull, No Pull-up-Pull-down, Medium Speed |

### iv. Circuit Diagram

![img](https://user-images.githubusercontent.com/38373000/192134563-72f68b29-4127-42ac-b064-2eda95a9a52a.png)

### v. Discussion

1. **We can use two different methods to detect an external signal: polling and interrupt. What are the advantages and disadvantages of each approach?**

   

2. **What would happen if the EXTI interrupt handler does not clear the interrupt pending flag? Check with your code**

   

### vi. Code





// YOUR MAIN CODE ONLY

// YOUR CODE

### vii. Results

Experiment images and results go here





Add [demo video link](https://github.com/ykkimhgu/course-doc/blob/master/ec-course/lab/link/README.md)

## III. Problem 2: Counting numbers on 7-Segment using SysTick

Display the number 0 to 9 on the 7-segment LED at the rate of 1 sec. After displaying up to 9, then it should display ‘0’ and continue counting.

When the button is pressed, the number should be reset ‘0’ and start counting again.

### i. Create HAL library

**ecSysTick.h**

```c
void SysTick_init(uint32_t msec);
void delay_ms(uint32_t msec);
uint32_t SysTick_val(void);
void SysTick_reset (void);
void SysTick_enable(void);
void SysTick_disable (void)
```

### ii. Procedure

1. Use the decoder chip (**74LS47**). Connect it to the bread board and 7-segment display.
2. First, check if every number, 0 to 9, can be displayed properly on the 7-segment.
3. Then, create a code to display the number counting from 0 to 9 and repeats at the rate of 1 second.
4. When the button is pressed, it should start from '0' again.

### iii. Configuration

| Digital In for Button (B1) | Digital Out for 7-Segment decoder             |
| :------------------------- | :-------------------------------------------- |
| Digital In                 | Digital Out                                   |
| PC13                       | PA7, PB6, PC7, PA9                            |
| PULL-UP                    | Push-Pull, No Pull-up-Pull-down, Medium Speed |

### iv. Circuit Diagram

> ![img](https://user-images.githubusercontent.com/38373000/192134563-72f68b29-4127-42ac-b064-2eda95a9a52a.png)

### v. Code



Your code goes here.

Explain your source code with necessary comments.

// YOUR MAIN CODE ONLY

// YOUR CODE

### vi. Results





Add [demo video link](https://github.com/ykkimhgu/course-doc/blob/master/ec-course/lab/link/README.md)

## IV. Reference

Complete list of all references used (github, blog, paper, etc)



## V. Troubleshooting

(Option) You can write a Troubleshooting section
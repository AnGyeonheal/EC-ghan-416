# [LAB#3] GPIO Digital InOut 7-segment

**Date: Sep.26.2023**

**Author: 21900416 Gyeonheal An**

**Github: https://github.com/AnGyeonheal/Embedded_Control**

**Demo Video: **https://www.youtube.com/shorts/HLNItlFfu7U

## I. Introduction

In this lab, you are required to create a simple program to control a 7-segment display to show a decimal number (0~9) that increases by pressing a push-button.

### i. Requirement

**Hardware**

- MCU

  - NUCLEO-F411RE

- Actuator/Sensor/Others:

  - 7-segment display(5101ASR)

  - Array resistor (330 ohm)

  - breadboard

**Software**

- Keil uVision, CMSIS, EC_HAL library

### ii. Exercise

| **Port/Pin**   | **Description**              | **Register setting**          |
| -------------- | ---------------------------- | ----------------------------- |
| Port A Pin 5   | Clear Pin5 mode              | GPIOA->MODER &=~(3<<(5*2))    |
| Port A Pin 5   | Set Pin5 mode = Output       | GPIOA->MODER \|=____________  |
| Port A Pin 6   | Clear Pin6 mode              | GPIOA->MODER &=~___________   |
| Port A Pin 6   | Set Pin6 mode = Output       | GPIOA->MODER \|=____________  |
| Port A Pin Y   | Clear PinY mode              | GPIOA->MODER &=~___________   |
| Port A Pin Y   | Set PinY mode = Output       | GPIOA->MODER \|=____________  |
| Port A Pin 5~9 | Clear Pin5~9 mode            | GPIOA->MODER &=~___________   |
|                | Set Pin5~9 mode = Output     | GPIOA->MODER \|=____________  |
| Port X Pin Y   | Clear Pin Y mode             | GPIOX->MODER &=~___________   |
|                | Set Pin Y mode = Output      | GPIOX->MODER \|=____________  |
| Port A Pin5    | Set Pin5 otype=push-pull     | GPIOA->OTYPER =____________   |
| Port A PinY    | Set PinY otype=push-pull     | GPIOA-> OTYPER =____________  |
| Port A Pin5    | Set Pin5 ospeed=Fast         | GPIOA->OSPEEDR =____________  |
| Port A PinY    | Set PinY ospeed=Fast         | GPIOA-> OSPEEDR =____________ |
| Port A Pin 5   | Set Pin5 PUPD=no pullup/down | GPIOA->OTYPER =____________   |
| Port A Pin Y   | Set PinY PUPD=no pullup/down | GPIOA-> OTYPER =____________  |

## II. Problem 1: Connecting 7-Segment Display

### i. Procedure

The popular BCD 7-segment decoder chips are **74LS47 and CD4511**.

Instead of using the decoder chip, we are going to make the 7-segment decoder with the MCU programming.

![img](https://424033796-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2F-MgmrEstOHxu62gXxq1t%2Fuploads%2FLmve9mRzHASR1OoYT0XN%2Fimage.png?alt=media&token=58563137-cbc7-432c-981c-7f93b3b68329)

Connect the common anode 7-segment with the given array resistors.

Apply VCC and GND to the 7-segment display.

Apply 'H' to any 7-segment pin 'a'~'g' and observe if that LED is turned on or off

- example: Set 'H' on PA5 of MCU and connect to 'a' of the 7-segment.

### ii. Connection Diagram



### iii. Discussion

1. **Draw the truth table for the BCD 7-segment decoder with the 4-bit input.**

   <img src="https://github.com/AnGyeonheal/Embedded_Control_GH/assets/118132313/4c678abb-2399-4db5-8075-d691d5e4b589" alt="image" style="zoom: 67%;" />

2. **What are the common cathode and common anode of 7-segment display?**



3. **Does the LED of a 7-segment display (common anode) pin turn ON when 'HIGH' is given to the LED pin from the MCU?**



## III. Problem 2: Display 0~9 with button press

### i. Procedure

1. Create a new project under the directory `\repos\EC\LAB\LAB_GPIO_7segment`

2. Include your updated library in `\repos\EC\lib\` to your project.

- **ecGPIO.h, ecGPIO.c**

- **ecRCC.h, ecRCC.c**

3. Declare and Define the following functions in your library

**ecGPIO.h**

```c
void sevensegment_init(void); 
void sevensegment_decoder(uint8_t  num);
```

- First, check if every number, 0 to 9, can be displayed properly
- Then, create a code to display the number from 0 to 9 with each button press. After the number '9', it should start from '0' again.

### ii. Configuration

| Digital In for Button (B1) | Digital Out for 7-Segment                                    |
| :------------------------- | :----------------------------------------------------------- |
| Digital In                 | Digital Out                                                  |
| PC13                       | PA5, PA6, PA7, PB6, PC7, PA9, PA8, PB10 ('a'~'h', respectively) |
| PULL-UP                    | Push-Pull, No Pull-up-Pull-down, Medium Speed                |

### iii. Code



### iv. Results

Experiment images and results

> Show experiment images /results

Add [demo video link](https://github.com/ykkimhgu/course-doc/blob/master/course/lab/link/README.md)

## III. Problem 3: Using both 7-Segment Decoder and 7-segment display

### i. Procedure

Now, use the decoder chip (**74LS47**). Connect it to the bread board.

Then, you need only 4 Digital out pins of MCU to display from 0 to 9.

![img](https://424033796-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2F-MgmrEstOHxu62gXxq1t%2Fuploads%2FOLBpZY2YOH4KNgHnb7du%2Fimage.png?alt=media&token=97cf1fb5-c747-40e4-b43a-4f743400bfa5)

### ii. Connection Diagram



1. Work on the same project and code.

- i.e. : project “**LAB_GPIO_7segment”.** and source file named as “**LAB_GPIO_7segment.c”**

void sevensegment_display_init(void); 

void sevensegment_display(uint8_t  num);

1. First, check if every number, 0 to 9, can be displayed properly

2. Then, create a code to display the number from 0 to 9 with each button press. After the number '9', it should start from '0' again.

### iii. Configuration

| Digital In for Button (B1) | Digital Out for 7-Segment                     |
| :------------------------- | :-------------------------------------------- |
| Digital In                 | Digital Out                                   |
| PC13                       | PA7, PB6, PC7, PA9                            |
| PULL-UP                    | Push-Pull, No Pull-up-Pull-down, Medium Speed |

## IV. Reference

Complete list of all references used (github, blog, paper, etc)



## V. Troubleshooting

(Option) You can write Troubleshooting section
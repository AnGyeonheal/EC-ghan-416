# Project

## LAB: EC Design Problem

**Date:** 2023-12-18

**Author/Partner:** Gyeonheal An / Hyeonho Moon

**Github:** https://github.com/AnGyeonheal/Embedded_Control_GH

**Demo Video:** 

## I. Introduction

### Overview

![image](https://github.com/AnGyeonheal/Embedded_Control_GH/assets/118132313/3bfbfedb-e67d-40e0-8084-b680e43ee9e8)

<img src="https://github.com/AnGyeonheal/Embedded_Control_GH/assets/118132313/d006be5a-4c33-458c-9404-164664acd856" alt="image" style="zoom:50%;" />

Machine Guidance and Machine Control excavators offer various innovative technologies and advantages compared to traditional excavators. Here's an explanation of these differences and strengths:

1. **Automation and Precision:**
   - *Machine Guidance:* Utilizes GPS and sensor technologies to monitor terrain in real-time and provide accurate location information, allowing operators to perform precise tasks.
   - *Machine Control:* Automated systems control the excavator, minimizing operator intervention and ensuring consistent accuracy.
2. **Productivity Enhancement:**
   - *Machine Guidance:* Defines job boundaries and efficiently operates within specified areas, improving productivity and preventing unnecessary work.
   - *Machine Control:* Accurate depth and angle control yield consistent results, increasing productivity and minimizing rework.
3. **Data Collection and Analysis:**
   - *Machine Guidance:* Collects various data during operations and analyzes it to enhance job efficiency. Real-time monitoring of soil conditions and job performance is possible.
   - *Machine Control:* Utilizes collected data to optimize productivity and predict maintenance tasks, enhancing machine reliability.
4. **Safety Reinforcement:**
   - *Machine Guidance:* Provides accurate location information of the work area, preventing collisions and enhancing safety.
   - *Machine Control:* Precise depth and angle control improve safety, reducing the risk of accidents during operations.
5. **Efficient Resource Utilization:**
   - *Machine Guidance and Machine Control:* Precise operations promote efficient resource utilization, optimizing fuel consumption for environmentally friendly operation.

*You need explain the overview of the design problem with a diagram and brief abstract*

[Please refer to past design problem](https://github.com/ykkimhgu/course-doc/blob/master/ec-course/project/LAB_DesignProblem_Smart Home_2021 (1).pdf)

- [Watch past final project videos](https://ykkim.gitbook.io/ec/ec-course/project/past-projects)

![img](https://user-images.githubusercontent.com/38373000/192134563-72f68b29-4127-42ac-b064-2eda95a9a52a.png)

### Requirements

You must use at least 3 sensors: including 1 analog sensor and 1 digital sensor

- You can choose sensors that were not used in the course LABs.
- You will get higher scores if you use more sensors
- You must use at least 2 actuators
- You must use at least 1 wireless communication.
- You must use at least 1 timer interrupt IRQ
- You will get a higher score if you use more numbers of MCUs, sensors, and actuators

#### Hardware

*Write down all the necessary hardware.*

| **Item**       | **Model/Description**                  | **Qty** |
| -------------- | -------------------------------------- | ------- |
| MCU            | NUCLEO -F411RE                         | 2       |
| Analog Sensor  | IR reflective optical sensor(TCRT5000) | 1       |
| Digital Sensor | -                                      | -       |
| Actuator       | -                                      | -       |
| Display        | -                                      | -       |
| Communication  | -                                      | -       |
| Others         | -                                      | -       |

#### Software

Keil uVision IDE, CMSIS, EC_HAL

## II. Problem

### Problem Description

*Detail description of the problem goes here.*

Your problem must have at least 3 modes(states). Explain necessary states(modes) and description

Example diagram:

![img](file:///C:/Users/ykkim/AppData/Local/Temp/msohtmlclip1/01/clip_image002.gif)

img

Description of system modes, how your MCU should function and more

*For example:*

**1.** **System Mode:**

| **MODE**                    | **Description**                                              |
| --------------------------- | ------------------------------------------------------------ |
| Normal Mode (NORM_MODE)     | Pressing MODE_button (B1 of MCU_1) toggles from NORM_mode 🡨🡪SECUR _mode |
| Security Mode (SECUR_MODE): | Pressing STOP_button (B1 of MCU_2) resets from SIREN_mode to SECUR_mode. |

**2.** **Front Door:**

| **MODE**                    | **Description**                                              |
| --------------------------- | ------------------------------------------------------------ |
| Normal Mode (NORM_MODE)     | When a person is detected within the given distance from the door(outside): Keep turning on the door light as long as the person is present. When a person is detected inside the house nearby the door: Turn on the door light for given period of time and does not generates the SIREN _TRG trigger. |
| Security Mode (SECUR_MODE): | When a person is detected within the given distance from the door(outside): Keep turning on the door light as long as the person is present and sends the VISITOR_LOG message to the server. When a person is detected inside the house nearby the door: Keep turning on the door light and generates the SIREN _TRG trigger. |

**3. Sensor Unit: MCU_1**

| **Function**                | **Sensor**                 | **Configuration**                                            | **Comments**                                                 |
| --------------------------- | -------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| Door Person detect (front)  | Ultrasonic distance sensor | Sampling 0.1 sec Check object presence within 30cm Generates PERSON_OUTSIDE flag | Need to check for outlier measurements (at least 7/10 Postive) VISITOR_LOG under SECUR_MODE |
| Door Person detect (inside) | PIR motion sensor          | Edge trigger Generates PERSON_INSIDE flag                    |                                                              |

### MCU Configuration

You are free to select appropriate configurations for the design problem. Create a configuration table to list all the necessary setup.

Example:

| **Functions**   | **Register** | **PORT_PIN**       | **Configuration**                                    |
| --------------- | ------------ | ------------------ | ---------------------------------------------------- |
| System Clock    | RCC          | -                  | PLL 84MHz                                            |
| delay_ms        | SysTick      | -                  |                                                      |
| Bluetooth       | USART1       | TXD: PA9 RXD: PA10 | No Parity, 8-bit Data, 1-bit Stop bit 9600 baud-rate |
| Timer Interrupt | TIMER1       | ...                | ...                                                  |

### Circuit Diagram

**MUST condition:** Show the wiring of each sensor/actuator component to MCUs.

*DO NOT draw by hand.!!!*

![img](https://user-images.githubusercontent.com/38373000/192134563-72f68b29-4127-42ac-b064-2eda95a9a52a.png)

## III. Algorithm

### Logic Design

In the report, you need to explain concisely how you designed the system with state tables/diagram or flow-chart.

- Listing all necessary states (states, input, output etc) to implement this design problem.
- Listing all necessary conditional FLAGS for programming.
- Showing the logic flow from the initialization
- and more

### Code

Your code (private repositor) goes here: [ADD Code LINK such as github](https://github.com/ykkimhgu/EC-student/)

// YOUR MAIN CODE ONLY



// YOUR CODE with Comments

## IV. Results and Demo

Experiment images and results

> Show experiment images /results
>
> You are required to show In-class demonstration.



Add [demo video link]

### **Analyze your results !!**





## V. Reference

Complete list of all references used (github, blog, paper, etc)

## Appendix

### Troubleshooting

### Other Appendix
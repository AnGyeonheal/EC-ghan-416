# PWM

## I. HOW TO USE PWM

PWM은 TimerInterrupt를 사용했을 때의 과정에서 OCyM의 값을 110 (PWM mode1)또는 111(PWM mode 2)로 설정하여 사용한다.

PWM은 Duty Cycle을 조정하여 출력 신호의 on-off를 나눈다.

![image](https://github.com/AnGyeonheal/Embedded_Control_GH/assets/118132313/db9ce4ea-509c-4e6d-8220-ec600f233c23)

여기서 Duty Cycle은 Carrier Signal이 Reference Signal(Threshold)을 넘었을 때 Switch On 상태가 되고 넘지 않았을 때를 Switch Off 상태로 정한다.
$$
D=\frac{T_{on}}{T_{on}+T_{off}}
$$
PWM을 Setting하는 방법은 아래의 순서와 같다.

1. **GPIO Pin Setting**

- RCC Timer Initialize

- AF(Alternate Function) Mode Selection

2. **Timer Setting**

- Enable Timer Peripheral Clock

- Set Counting Direction
- Set Timer Clock Pre-Scaler Value and Auto-Reload Value
- Enable Counter

3. **PWM Output Setting**

- Select Output Mode as PWM
- Set Compare Capture Value
- Select Output Polarity
- Enable Compare Capture Output

## II. GPIO Pin Setting

### 1. RCC Timer Initialize

- 아래의 함수를 사용하여 system clock을 PLL(=84Mhz)로 맞춰준다.

```c
void RCC_PLL_init()
```

### 2. AF Mode Selection

- GPIO를 Alternative Function 모드로 설정한다.

GPIO(General Purpose Input-Output)의 기능은 (Input, Output, AF) 세 가지이며 AF는 Input과 Output외의 다른 기능을 수행할 수 있도록 하는 기능이다.

AF mode 설정은 GPIOx_MODER에서 할 수 있다. (10)

![image](https://github.com/AnGyeonheal/Embedded_Control_GH/assets/118132313/e5324a05-3b6a-49d8-a59f-e2f1d144ce1d)

AF에는 AFRL, AFRH 두 가지로 나뉜다.

AFRL : AFR[0]으로 표현되며 Pin 0부터 7까지 이에 해당된다.

AFRH : AFR[1]으로 표현되며 Pin 8부터 15까지 이에 해당된다.

**GPIOx_AFR**

<img src="https://github.com/AnGyeonheal/Embedded_Control_GH/assets/118132313/6fc098db-d640-40dc-a96e-697f51cc7abc" alt="image" style="zoom:67%;" />

또한 AFR은 Timer에 따라 정해진 bit(AF0~AF15)값이 존재한다.

![image](https://github.com/AnGyeonheal/Embedded_Control_GH/assets/118132313/de189cb6-3f09-4c7b-b831-3d6a22411d32)

GPIO에서 



AFR은 4bit 메모리이고 예를 들어 5번 핀에 TIM1을 AF로 사용할 때 0x0001을 5번 핀인 AFRL5에 입력한다.

```c
GPIOA->AFR[0] |= 1UL << (4*PIN);  // AF1 at PA5 
GPIOA->MODER &= ~(3UL << 2*PIN);	// clear
GPIOA->MODER |= 2UL << (2 * PIN);	// AF
```

## II. Timer Setting

### 1. Enable Timer Pheriperal Clock

APB1ENR은 Timer Interrupt를 사용할 때와 마찬가지로 TIMx를 enable하는 역할을 한다. 

**RCC_APB1ENR**

![image](https://github.com/AnGyeonheal/Embedded_Control_GH/assets/118132313/8798aaff-0af5-40bb-9d4d-7ba0d94dc994)

```c
RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
```

해당 코드는 TIM2를 enable한 코드이다.

### 2. Set Counting Direction

다음은 Counting 방향을 Up으로 할지 Down으로 할지 결정하는 방법이다.

![image](https://github.com/AnGyeonheal/Embedded_Control_GH/assets/118132313/23a6f78b-aa15-4c2c-8d4a-e048e4dce653)

이는 Counting Up 또는 Counting Down하는 와중 Counter overflow가 생기면 event를 발생시키는 원리이다.

**TIMx_CR1**

![image](https://github.com/AnGyeonheal/Embedded_Control_GH/assets/118132313/5020d6e1-7830-40a1-9a98-aba93bb7cc2b)

0: Upcounter

1: Downcounter

```c
TIMx->CR1 &= ~(1UL << 4UL);	// Upcounting
```

### 3. Set Timer Clock Pre-Scaler Value and Auto-Reload Value

![image](https://github.com/AnGyeonheal/Embedded_Control_GH/assets/118132313/706f6db5-02d4-4cc3-91fa-48e2840f80ba)

#### (1). PSC

PSC값으로 1차적으로 System clock의 pulse를 조정한다. 이 때 PSC값은 아래와 같이 구할 수 있다.
$$
PSC = \frac{System\;Clock}{Setting\;CLK }-1
$$
예를 들어 System CLK을 PLL (84Mhz)을 사용하고 Setting CLK을 100kHz로 설정하고 싶다면 PSC값은 839를 입력하여 1초에 100,000번 주기를 가지는 Pulse파형을 생성할 수 있다.  

#### (2). ARR

PSC값을 설정하여 Setting CLK을 설정하였다면 ARR값을 설정하여 몇번의 카운트에서 Overflow 또는 Underflow를 발생시킬지를 결정한다.
$$
Count\;Period=\frac{(1+ARR)}{Setting\;CLK}
$$
따라서 Up\Down Counting일 때 ARR값은 아래의 식을 통해 구할 수 있다.
$$
ARR=Setting\;CLK\times Count\;Period-1
$$
예를 들어 1ms의 Counting Period를 만들고 싶다면

위의 PSC를 설정하여 구한 Setting CLK 100kHz와 Count Period 1m sec를 식에 대입한다.
$$
ARR=100\times10^3\times1\times10^{-3}-1=99
$$
따라서 PSC값을 839, ARR값을 99로 설정할 경우 1ms의 기준 Counter Period를 만들 수 있다.

이후 main문에서 해당 기준값을 1000번 반복하게 된다면 1초의 Event 발생 간격을 만들 수 있다.

**※ 주의할 점은 TIM2, TIM5에서는 ARR 값이 32bit 까지 가질 수 있고 나머지는 16bit까지 가질 수 있다.**

### 4. Enable Counter

Counting Period를 만들었으면 해당 Counter를 사용할 수 있도록 한다.


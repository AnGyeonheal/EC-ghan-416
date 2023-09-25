/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : GyeonhealAn
Created          : 05-03-2021
Modified         : 09-25-2023
Language/ver     : C++ in Keil uVision

Description      : Distributed to Students for LAB_GPIO
/----------------------------------------------------------------*/

#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"

#define LED_PIN 	5
#define BUTTON_PIN 13

void setup(void);

int main(void) {

    int delay;
    int cnt=4;

    // Initialiization --------------------------------------------------------
    setup();

    // Inifinite Loop ----------------------------------------------------------
    while(1){

        if((GPIO_read(GPIOC, BUTTON_PIN)==0) && (delay > 50000) && (cnt % 4 == 0)){
            GPIO_write(GPIOA, LED_PIN, HIGH);
            GPIO_write(GPIOA, 6, LOW);
            GPIO_write(GPIOA, 7, LOW);
            GPIO_write(GPIOB, 6, LOW);
            delay = 0;
            cnt++;
        }

        else if((GPIO_read(GPIOC, BUTTON_PIN)==0) && (delay > 50000) && (cnt % 4 == 1)){
            GPIO_write(GPIOA, 6, HIGH);
            GPIO_write(GPIOA, LED_PIN, LOW);
            GPIO_write(GPIOA, 7, LOW);
            GPIO_write(GPIOB, 6, LOW);
            delay = 0;
            cnt++;
        }

        else if((GPIO_read(GPIOC, BUTTON_PIN)==0) && (delay > 50000) && (cnt % 4 == 2)){
            GPIO_write(GPIOA, 7, HIGH);
            GPIO_write(GPIOA, LED_PIN, LOW);
            GPIO_write(GPIOA, 6, LOW);
            GPIO_write(GPIOB, 6, LOW);
            delay = 0;
            cnt++;
        }

        else if((GPIO_read(GPIOC, BUTTON_PIN)==0) && (delay > 50000) && (cnt %4 == 3)){
            GPIO_write(GPIOB, 6, HIGH);
            GPIO_write(GPIOA, LED_PIN, LOW);
            GPIO_write(GPIOA, 6, LOW);
            GPIO_write(GPIOA, 7, LOW);
            delay = 0;
            cnt++;
        }
        delay ++ ;
    }
}



// Initialiization
void setup(void)
{
    RCC_HSI_init();
    GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
    GPIO_init(GPIOA, LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()
    GPIO_init(GPIOA, 6, OUTPUT);    // calls RCC_GPIOA_enable()
    GPIO_init(GPIOA, 7, OUTPUT);    // calls RCC_GPIOA_enable()
    GPIO_init(GPIOB, 6, OUTPUT);    // calls RCC_GPIOB_enable()

    GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);
    GPIO_ospeed(GPIOA, LED_PIN, EC_MEDIUM);
    GPIO_otype(GPIOA, LED_PIN, EC_PUSH_PULL);
    GPIO_pupd(GPIOA, LED_PIN, EC_PU);

    GPIO_ospeed(GPIOA, 6, EC_MEDIUM);
    GPIO_otype(GPIOA, 6, EC_PUSH_PULL);
    GPIO_pupd(GPIOA, 6, EC_PU);

    GPIO_ospeed(GPIOA, 7, EC_MEDIUM);
    GPIO_otype(GPIOA, 7, EC_PUSH_PULL);
    GPIO_pupd(GPIOA, 7, EC_PU);

    GPIO_ospeed(GPIOB, 6, EC_MEDIUM);
    GPIO_otype(GPIOB, 6, EC_PUSH_PULL);
    GPIO_pupd(GPIOB, 6, EC_PU);
}
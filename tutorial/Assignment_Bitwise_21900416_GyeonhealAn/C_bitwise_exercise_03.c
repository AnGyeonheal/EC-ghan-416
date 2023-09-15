#include <stdio.h> 
#include <stdint.h>

/*
Class : Embedded Controller
Name : Gyeonheal An
Student No : 21900416
Date : 2023-09-12
*/

int main()
{
    //Exercise_1: Turning ON LEDs of Port A(PA)
    //Read PA6, 6th from LSB
    unsigned char PA = 0b00001111;                 // LED0 is LSB, Set to turn on LED
    uint8_t bits = PA & (1 << 6);	// check bit of a6


    // Exercise_2: Turning ON LEDs of Port A(PA)
    // assume 8 LEDs are connected to Digital Out pins of PA
    PA = 0b00001111;         // LED0 is LSB, Set to turn on LED
    PA |= (1 << 4); 	// turn ON LED4 
    PA |= (3 << 4); 	// turn ON LED4 and LED5


    // Exercise_3: Turning off LEDs of Port A(PA)
    PA = 0b00001111;         // LED0 is LSB, Set to turn on LED
    PA &= ~(1 << 2);      // turn off LED2 

    return 0;
}
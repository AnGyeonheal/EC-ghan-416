#include <stdio.h>

/*
Class : Embedded Controller
Name : Gyeonheal An
Student No : 21900416
Date : 2023-09-12
*/
void dec2bin(unsigned int n);

void main() {

    unsigned int a = 118;
    printf("\n118 as binary = ");
    dec2bin(a);

    //read 8th bit of a :   a[7]
    unsigned int bit_check = a & (1 << 7);
    printf("\nresult1 = %d", bit_check);

    // set 8th (a[7])  bit as HIGH
    a = a | (1 << 7);
    printf("\nresult2 = ");
    dec2bin(a);


	//set 3rd bit as LOW
    a = a & ~(1 << 2);
	printf("\nresult3 = ");
	dec2bin(a);


	//set 3rd and 4th bit as HIGH .  Use only one code line
    a = a | (3 << 2);
	printf("\nresult4 = ");
	dec2bin(a);


	//toogle 1st bit
    a ^= 1 << 0;
	printf("\nresult5 = ");
	dec2bin(a);


	//set 1st and 8th bit as LOW. Use only one code line 
    a = a & 0b01111110 << 0;
	printf("\nresult6 = ");
	dec2bin(a);

	printf("\n");

}

/////////////////////////
void dec2bin(unsigned int n) {
    unsigned int a = 0x80;
    for (int i = 0; i < 8; i++) {
        if ((a & n) == a)
            printf("1");
        else
            printf("0");
        a = a >> 1;
    }
}
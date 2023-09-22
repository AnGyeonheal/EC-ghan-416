/*-------------------------------------------------------------------------------\
@ C-Tutorial by Young-Keun Kim - Handong Global University
Author           : SSS LAB
Created          : 19-08-2022
Modified         : 08-09-2023
Language/ver     : C in MSVS2022
Description      : C_pointer_exercise2.c
-------------------------------------------------------------------------------*/

#include <stdio.h>

int main()
{
	int x = 10;
	double y = 2.5;
	int* ptrX = &x;
	int* ptrY = &y;

	/*
	-Print the address of variable ¡®x¡¯
	-Print the address of variable ¡®y¡¯
	-Print the value of pointer ¡®ptrX ¡®
	-Print the address of pointer ¡®ptrX ¡®
	-Print the size of pointer ¡®ptrX ¡®
	*/

	/*
	-Print the value of pointer ¡®ptrY ¡®
	-Print the address of pointer ¡®ptrY ¡®
	-Print the size of pointer ¡®ptrY
	*/

	printf("%d\n", &x);
	printf("%d\n", &y);
	printf("%d\n", ptrX);
	printf("%d\n", &ptrX);
	printf("%d\n\n", sizeof(ptrX));

	printf("%d\n", ptrY);
	printf("%d\n", &ptrY);
	printf("%d\n", sizeof(ptrY));


	return 0;
}
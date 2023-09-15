/*-------------------------------------------------------------------------------\
@ C-Tutorial by Young-Keun Kim - Handong Global University
Author           : SSS LAB
Created          : 19-08-2022
Modified         : 07-09-2023
Language/ver     : C in MSVS2022
Description      : C_array1d_exercise2.c
-------------------------------------------------------------------------------*/

#include <stdio.h>

int main() {

	int st[5] = { 1,2,3,4,5 };
	int* ptr;

	ptr = st;
	for (int i = 0; i < 5; i++) {
		//  print each element by using pointer  e.g.  (ptr)
		printf("%d\n", *(ptr + i));
	}
}

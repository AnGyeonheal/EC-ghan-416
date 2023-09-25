#include <stdio.h> 

/*
Class : Embedded Controller
Name : Gyeonheal An
Student No : 21900416
Date : 2023-09-12
*/

int main()
{
    unsigned char num1 = 4;    // 4: 0000 0100
    unsigned char num2 = 4;    // 4: 0000 0100
    unsigned char num3 = 4;    // 4: 0000 0100
    unsigned char num4 = 4;    // 4: 0000 0100
    unsigned char num5 = 4;    // 4: 0000 0100

    num1 &= 5;     // 5(0000 0101) AND ���� �� �Ҵ�
    num2 |= 2;     // 2(0000 0010) OR ���� �� �Ҵ�
    num3 ^= 3;     // 3(0000 0011) XOR ���� �� �Ҵ�
    num4 <<= 2;    // ��Ʈ�� �������� 2�� �̵��� �� �Ҵ�
    num5 >>= 2;    // ��Ʈ�� ���������� 2�� �̵��� �� �Ҵ�

    printf("%u\n", num1);   // 0000 0100
    printf("%u\n", num2);   // 0000 0110
    printf("%u\n", num3);   // 0000 0111
    printf("%u\n", num4);   // 0001 0000
    printf("%u\n", num5);   // 0000 0001

    return 0;
}
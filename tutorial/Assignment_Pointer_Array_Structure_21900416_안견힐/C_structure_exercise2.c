/*-------------------------------------------------------------------------------\
@ C-Tutorial by Young-Keun Kim - Handong Global University
Author           : SSS LAB
Created          : 19-08-2022
Modified         : 08-09-2023
Language/ver     : C in MSVS2022
Description      : C_structure_exercise2.c
-------------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
	char building_name[100];
	int room_number;
	char room_name[100];
} Handong;


int main()
{
	// Create structure type (Handong)  variable  room1
	Handong room1;
	// Create structure type (Handong)  variable  room2, room3
	Handong room2;
	Handong room3;

	// Create structure type (Handong)  Pointer , roomPt
	Handong* roomPt = malloc(sizeof(Handong));

	// Assign room3 address to roomPt
	roomPt = &room3;


	// Define structure variable  memeber values: room1
	strcpy(room1.building_name, "Newton");
	room1.room_number = 109;
	strcpy(room1.room_name, "SSSLAB");

	// Define structure variable  memeber values: room2
	strcpy(room2.building_name, "Newton");
	room2.room_number = 119;
	strcpy(room2.room_name, "Computer room");


	// Define structure variable  memeber values: room3
	strcpy(room3.building_name, "Bethel");
	room3.room_number = 116;
	strcpy(room3.room_name, "Dormitory");



	// Print each member values  : room1, room2, room3
	printf("%s building, room  %d  is %s\n", room1.building_name, room1.room_number, room1.room_name);
	printf("%s building, room  %d  is %s\n", room2.building_name, room2.room_number, room2.room_name);
	printf("%s building, room  %d  is %s\n", room3.building_name, room3.room_number, room3.room_name);
	

	// Print each member values by  pointer variable: roomPt
	printf("%s building, room  %d  is %s\n", roomPt->building_name, roomPt->room_number, roomPt->room_name);

	// Print address of   room3  and value of roomPt and compare. 
	printf("\n room3 address=%x ,  roomPt = %x  \n", &room3, roomPt);

	// Free mallocated variable 
	free(roomPt);

}

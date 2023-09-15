/*-------------------------------------------------------------------------------\
@ C-Tutorial by Young-Keun Kim - Handong Global University
Author           : SSS LAB
Created          : 19-08-2022
Modified         : 08-09-2023
Language/ver     : C in MSVS2022
Description      : C_array2d_exercise.c
-------------------------------------------------------------------------------*/


#include <stdio.h>
#include <stdlib.h>

// ASSUMPTION:  3x3 matrix only


// Matrix addtion
void addMat(double _matA[][3], double _matB[][3], double _matOut[][3], int _dim);
// Matrix multiplication
void multMat(double _matA[][3], double _matB[][3], double _matOut[][3], int _dim);
// Transposed matrix
void transMat(double _matA[][3], double _matOut[][3], int _dim);
// Subtract  matrix
void subtractMat(double _matA[][3], double _matB[][3], double _matOut[][3], int _dim);

// Print matrix
void printMat(double _matA[][3], int _dim);


int main() {

	// 2D array declaration and definition
	double A[3][3] = { 10, 20, 30, 40, 50, 60, 70, 80, 90 };
	double B[3][3] = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
	double addMat_out[3][3] = { 0 };
	double multMat_out[3][3] = { 0 };
	double transMat_out[3][3] = { 0 };

	// Calling functions
	addMat(A, B, addMat_out, 3);
	multMat(A, B, multMat_out, 3);
	transMat(A, transMat_out, 3);

	// Print results
	printf("addMat out = \n");
	printMat(addMat_out, 3);
	printf("\nmultMat_out = \n");
	printMat(multMat_out, 3);
	printf("\ntransMat_out = \n");
	printMat(transMat_out, 3);

	system("pause");
	return 0;
}

// Matrix addtion
void addMat(double _matA[][3], double _matB[][3], double _matOut[][3], int _dim) {
	for (size_t i = 0; i < _dim; i++)
		for (size_t j = 0; j < _dim; j++)
			_matOut[i][j] = _matA[i][j] + _matB[i][j];

}


// Subtract  matrix
void subtractMat(double _matA[][3], double _matB[][3], double _matOut[][3], int _dim)
{
	for (size_t i = 0; i < _dim; i++)
		for (size_t j = 0; j < _dim; j++)
			_matOut[i][j] = _matA[i][j] - _matB[i][j];
}



// Matrix multiplication
void multMat(double _matA[][3], double _matB[][3], double _matOut[][3], int _dim) {
	for (size_t i = 0; i < _dim; i++)
		for (size_t j = 0; j < _dim; j++) {
			_matOut[i][j] = 0;
			for (size_t k = 0; k < _dim; k++)
				_matOut[i][j] += _matA[i][k] * _matB[k][j];
		}
}

// Transposed matrix
void transMat(double _matA[][3], double _matOut[][3], int _dim) {
	for (size_t i = 0; i < _dim; i++)
		for (size_t j = 0; j < _dim; j++)
			_matOut[i][j] = _matA[j][i];
}


// Print matrix
void printMat(double _matA[][3], int _dim) {
	for (int i = 0; i < _dim; i++) {
		for (int j = 0; j < _dim; j++)
			printf("%f\t", _matA[i][j]);
		printf("\n");
	}
	printf("\n");
}

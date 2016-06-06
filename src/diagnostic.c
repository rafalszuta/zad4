/*
 * diagnostic.c
 *
 *  Created on: 3 cze 2016
 *      Author: Rafal
 */

#include "diagnostic.h"


 double getAverage( const int arr[ ], int size)
 {
 	int j= 0;
 	double sum;
 	double avg;

 	for (j = 0; j < size; j++)
 	{
 		DEBUGOUT("Funkcja: Tab[%d]=%d\n", j,arr[j]);
 		sum += arr[j];
 	}

	avg = sum / size;
 	return avg;
 }



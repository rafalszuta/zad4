/*
 * diagnostic.c
 *
 *  Created on: 3 cze 2016
 *      Author: Rafal
 */

#include "diagnostic.h"


void getAverage(const uint8_t array[], int size, double *avg_wsk)
 {
 	int j= 0;
 	int sum=0;

 	for (j = 0; j < size; j++)
 	{
 		sum += array[j];
 	}

	*avg_wsk = 1.0*sum/size;
 }



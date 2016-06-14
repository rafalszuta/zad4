#ifndef DIAGNOSTIC_H_
#define DIAGNOSTIC_H_
#include "stdint.h"
#include "board.h"

double getAverage(const uint8_t array[], int size, double *averagePointer);
uint8_t getMeasurementNumber(uint8_t bit2,uint8_t bit1,uint8_t bit0);
int getUserAverage(const uint8_t array[], double *avg_wsk,
		uint8_t measureNumber, int size);

#endif /* DIAGNOSTIC_H_ */

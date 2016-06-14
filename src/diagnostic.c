#include "diagnostic.h"
#include "Math.h"

/* LED NUMBERS */
#define RED_LED 0
#define GREEN_LED 1
#define BLUE_LED 2

double getAverage(const uint8_t array[], int size, double *averagePointer) {
	int sum = 0;
	for (int j = 0; j < size; j++) {
		sum += array[j];
	}

	*averagePointer = 1.0 * sum / size;
	return *averagePointer;
}

uint8_t getMeasurementNumber(uint8_t bit2, uint8_t bit1, uint8_t bit0) {

	uint8_t measurementNumber = (bit2 * 4) + (bit1 * 2) + (bit0 * 1);

	return measurementNumber;
}

int getUserAverage(const uint8_t array[], double *averagePointer,
		uint8_t measureNumber, int size) {

	int sum = 0;
	double userAverage = 0;
	uint8_t userAverageArray[10] = { 0 };
	int ledNumber;

	if (measureNumber > size) {
		measureNumber = size;
	}

	for (int i = 0; i < measureNumber; i++) {
		if (size < 0) {
			size = 9;
		}
		userAverageArray[i] = array[size - 1];
		sum += userAverageArray[i];
		size--;
	}

	for (int x = 0; x < 10; x++) {
		DEBUGOUT("userAverageArray[%d]=%d\n", x, userAverageArray[x]);
	}

	DEBUGOUT("User sum=%d\n", sum);
	userAverage = 1.0 * sum / measureNumber;

	DEBUGOUT("Average all: %lf\n", *averagePointer);
	DEBUGOUT("Average user: %lf\n", userAverage);

	if (*averagePointer == userAverage) {
		DEBUGOUT("Equal average!\n");
		Board_LED_Set(RED_LED, false);
		Board_LED_Set(GREEN_LED, true);
		Board_LED_Set(BLUE_LED, false);
		ledNumber = 1;
	}

	if (*averagePointer < userAverage) {
		DEBUGOUT("User average is higher!\n");
		Board_LED_Set(RED_LED, true);
		Board_LED_Set(GREEN_LED, false);
		Board_LED_Set(BLUE_LED, false);
		ledNumber = 0;
	}

	if (*averagePointer > userAverage) {
		DEBUGOUT("User average is lower!\n");
		Board_LED_Set(RED_LED, false);
		Board_LED_Set(GREEN_LED, false);
		Board_LED_Set(BLUE_LED, true);
		ledNumber = 2;
	}
	return ledNumber;
}


/*
 * tests.c
 *
 *  Created on: 3 cze 2016
 *      Author: Rafal
 */

#ifdef TEST
#include "unity.h"
#include "board.h"
#include "diagnostic.h"

/* LED NUMBERS */
#define RED_LED 0
#define GREEN_LED 1
#define BLUE_LED 2


void test1()
{

	uint8_t tempTabTest[3] = {10};
	uint8_t userTabTest[3] = {10};
	double averageTest = 0;   //average
	double *averagePointerTest;
	averagePointerTest = &averageTest;

	TEST_ASSERT_EQUAL(10,getAverage(tempTabTest,3,averagePointerTest));
	TEST_ASSERT_EQUAL(7,getMeasurementNumber(0,1,0));

	/////////////////////
	TEST_ASSERT_EQUAL(1,getUserAverage(userTabTest,averagePointerTest,3,5));
	//////////////////////////

	TEST_ASSERT_TRUE(Board_LED_Test(GREEN_LED));
	TEST_ASSERT_FALSE(Board_LED_Test(RED_LED));
	TEST_ASSERT_FALSE(Board_LED_Test(BLUE_LED));
}



void main(void )
{
	Board_Init();
	//......unity code

	UNITY_BEGIN();

    RUN_TEST(test1);


	return UNITY_END();
}
#endif

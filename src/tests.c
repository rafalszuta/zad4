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
//test functions



void main(void )
{
	Board_Init();
	//......unity code

	UNITY_BEGIN();

    RUN_TEST(test1);
    RUN_TEST(test2);
    RUN_TEST(test3);

	return UNITY_END();
}
#endif

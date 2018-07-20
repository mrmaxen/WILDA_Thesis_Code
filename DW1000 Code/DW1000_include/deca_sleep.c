/*
 *	 deca_sleep.c
 *	Delay function for MSP432
 *      Author: Max Nilsson
 */
#include "sleep.h"

void deca_sleep(unsigned int time_ms)
{
    sleep_ms(time_ms);
}

void sleep_ms(unsigned int time_ms)
{
    int i;
    for (i = time_ms; i>0; i--)
    {
        _delay_cycles(1000);
    }
}

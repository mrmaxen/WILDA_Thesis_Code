/*
	Header file for Platform specific library for MSP432P401R

*/

#ifndef _DWM_API_H_
#define _DWM_API_H_

#include "driverlib.h"

void sleep_ms(unsigned int time_ms);

/********		UART Functions 			********/

void msp_uart_init();

void msp_uart_tx(uint8_t* data, uint8_t length);

void msp_uart_rx(uint8_t* rx_data, uint8_t exp_length):

/********				Timer Functions				********/

void init_timer(uint16_t timer_val);

void start_timer();

uint32_t check_timer();

void halt_timer();

#endif
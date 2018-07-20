/*
 * msp432_setup.h
 *
 *      Author: Max Nilsson
 */

#ifndef MSP432_SETUP_H_
#define MSP432_SETUP_H_

#include "driverlib.h"

void peripherals_init();

void reset_DW1000();

void spi_init();

void interrupt_init();

void spi_select_slave(uint_fast8_t port, uint_fast16_t pin);

void init_timer();

void start_timer();

uint32_t check_timer();

void halt_timer();

#endif /* MSP432_SETUP_H_ */

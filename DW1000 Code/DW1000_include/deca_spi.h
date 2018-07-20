/*
 * msp432_setup.h
 *
 *  Created on: 1 mars 2018
 *      Author: Max
 */

#ifndef DECA_SPI_H
#define DECA_SPI_H_

void open_spi(uint_fast8_t port, uint_fast16_t pin);

void close_spi(uint_fast8_t port, uint_fast16_t pin);


#endif /* DECA_SPI_H_ */
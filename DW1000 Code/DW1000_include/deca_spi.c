/*
*
*	Library of SPI functions for interaction between MSP432 and DW1000
*
*	Author: Max Nilsson
*
*/
#include "driverlib.h"
#include "deca_device_api.h"
#include "msp432_setup.h"

// variables for choosing slave device
static uint_fast8_t selectedPort = GPIO_PORT_P2;
static uint_fast16_t selectedPin = GPIO_PIN3;
/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 */
int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer)
{
    int i;

    MAP_GPIO_setOutputLowOnPin(selectedPort, selectedPin);

	for(i = 0; i < headerLength; i++) {
		SPI_transmitData(EUSCI_B0_BASE, headerBuffer[i]); // send header
	}
	for(i = 0; i < bodylength; i++) {
		SPI_transmitData(EUSCI_B0_BASE, bodyBuffer[i]); // write values
	}
	MAP_GPIO_setOutputHighOnPin(selectedPort, selectedPin);
	return 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 */
int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer)
{
    decaIrqStatus_t stat;
    stat = decamutexon();
    int i;
    MAP_GPIO_setOutputLowOnPin(selectedPort, selectedPin);
	for(i = 0; i < headerLength; i++) {
	    while(!MAP_SPI_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT));
		SPI_transmitData(EUSCI_B0_BASE, headerBuffer[i]); // send header
		//while(!received)
		//{;}
		while(!(UCRXIFG));
		readBuffer[0] = SPI_receiveData(EUSCI_B0_BASE);
		//received = false;
	}
	for(i = 0; i < readlength; i++) {
	    while(!MAP_SPI_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT));
	    SPI_transmitData(EUSCI_B0_BASE, 0); // write dummy data

	    while(!(UCRXIFG));
		readBuffer[i] = SPI_receiveData(EUSCI_B0_BASE);
	}
	MAP_GPIO_setOutputHighOnPin(selectedPort, selectedPin);
	decamutexoff(stat);
	return 0;
}

// Function for initiating SPI data transfer
void open_spi(uint_fast8_t port, uint_fast16_t pin)
{
    GPIO_setOutputLowOnPin(port, pin);
}

// end SPI data transfer
void close_spi(uint_fast8_t port, uint_fast16_t pin)
{
    GPIO_setOutputHighOnPin(port, pin);
}

void spi_select_slave(uint_fast8_t port, uint_fast16_t pin)
{
	selectedPort = port;
	selectedPin = pin;
}

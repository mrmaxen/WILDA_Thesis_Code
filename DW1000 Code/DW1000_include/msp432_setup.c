/*
*	
*	 Hardware specific definitions and functions based on decawave source
*
*	
*
*/


#include "msp432_setup.h"
#include <stdio.h>

/*
	Function for resetting the DW1000, as specified in datasheet
*/
void reset_DW1000()
{
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN6);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6);

    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN1);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1);

    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN2);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);

    _delay_cycles(2000);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN6);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN1);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN2);
    _delay_cycles(10000);
}

/************		SPI Functions		**********/

/*	SPI config struct	*/
const eUSCI_SPI_MasterConfig spiMasterConfig = 
{	EUSCI_B_SPI_CLOCKSOURCE_SMCLK, 3000000, 1500000,
		EUSCI_B_SPI_MSB_FIRST,
		EUSCI_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT,
		EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW,
		EUSCI_B_SPI_3PIN };


/*Initialise SPI pins and MSP432 SPI module*/
void spi_init()
{
	// define MOSI, MISO and CLK pins
	MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, 
		GPIO_PIN5 | GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);
	MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, 
		GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
	
	// initialize as master
	SPI_initMaster(EUSCI_B0_BASE, &spiMasterConfig);
	SPI_enableModule(EUSCI_B0_BASE);
	
	/*SPI_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT);
	Interrupt_enableInterrupt(INT_EUSCIB0);
	Interrupt_enableSleepOnIsrExit();*/

	// Set up SS pins
	MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN3);
	MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN3);
	
	MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN4);
	MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN4);
	
	MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN7);
	MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7);
}

void peripherals_init()
{
    spi_init();
}



/************		Interrupt Functions		**********/
// disable interrupts
int decamutexon()
{
    Interrupt_disableMaster();

    return 0;
}

// enable interrupts
void decamutexoff(int s)
{
    Interrupt_enableMaster();
}

/************		Timer Functions		**********/

// Initialise MSP432 32 bit timer
void init_timer()
{
    MAP_Timer32_initModule(TIMER32_0_BASE, TIMER32_PRESCALER_256, TIMER32_16BIT, TIMER32_PERIODIC_MODE);
    MAP_Timer32_setCount(TIMER32_0_BASE, 0x2FFF); // second argument is the value the timer will count to
}

void start_timer()
{
    MAP_Timer32_startTimer(TIMER32_0_BASE, true);
}

// Function for checking current value of the timer.
uint32_t check_timer()
{
    uint32_t timerValue = MAP_Timer32_getValue(TIMER32_0_BASE);
    return timerValue;
}
void halt_timer()
{
    MAP_Timer32_haltTimer(TIMER32_0_BASE);
}







/*
	Platform specific library for MSP432P401R

	Author: Max Nilsson
*/
#include "msp432_lib.h"

volatile uint8_t rx_buffer[255];
volatile uint8_t readCount = 0;

void sleep_ms(unsigned int time_ms)
{
	// one millisecond is 12000 cycles when running the 12MHz clock
    uint16_t d_cycles = time_ms*12000;
    _delay_cycles(d_cycles);
    
}

/********		UART Functions 			********/
const eUSCI_UART_Config uartConfig =
{
	EUSCI_A_UART_CLOCKSOURCE_SMCLK, // SMCLK Clock Source
	6, // BRDIV = 6		Values for clock frequency 12 MHz and 115200 Baud
	8, // UCxBRF = 8
	32, // UCxBRS = 32
	EUSCI_A_UART_NO_PARITY, // No Parity
	EUSCI_A_UART_LSB_FIRST, // LSB First
	EUSCI_A_UART_ONE_STOP_BIT, // One stop bit
	EUSCI_A_UART_MODE, // UART mode
	EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION // Oversampling
};

void msp_uart_init()
{
	/* Selecting P1.2 and P1.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
            GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
			
	/* Configuring UART Module */
	MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);
	/* Enable UART module */
	MAP_UART_enableModule(EUSCI_A0_BASE);
	/* Enabling interrupts */
	MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
	MAP_Interrupt_enableInterrupt(INT_EUSCIA0);
	//MAP_Interrupt_enableSleepOnIsrExit();
	MAP_Interrupt_enableMaster();

}

void msp_uart_tx(uint8_t* data, uint8_t length)
{
	int i;
	for (i = 0; i<length; i++)
	{
		//while(!(UCA1IFG & UCTXIFG)); 
		
		transmitData(EUSCI_A0_BASE, data[i]);
	}
	
}

int msp_uart_rx(uint8_t* rx_data, uint8_t exp_length)
{
	int i;
	init_timer(0xFFFF); 
	start_timer();
	while(readCount < exp_length) // wait for interrupts to read data
	{
		if(check_timer() == 0x00)
		{
			break; // timeout
		}
	}
	for (i = 0; i < readCount; i++)
	{
		rx_data[i] = rx_buffer[i];
		rx_buffer[i] = 0;
	}
	
	return 0;
}

void EUSCIA0_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);
	static uint8_t rx_buffer[255];

    MAP_UART_clearInterruptFlag(EUSCI_A0_BASE, status);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        rx_buffer[readCount] = MAP_UART_receiveData(EUSCI_A0_BASE);
		readCount++;
    }

}

/********				Timer Functions				********/

void init_timer(uint32_t timer_val)
{
	MAP_Timer32_initModule(TIMER32_0_BASE, TIMER32_PRESCALER_256, TIMER32_16BIT, TIMER32_PERIODIC_MODE);
	MAP_Timer32_setCount(TIMER32_0_BASE, timer_val); // max value (0xFFFF) should be about 1.25 seconds
}

void start_timer()
{
	MAP_Timer32_startTimer(TIMER32_0_BASE, true);
}

uint32_t check_timer()
{
	uint32_t timer_value = MAP_Timer32_getValue(TIMER32_0_BASE);
	return timer_value;
}

void halt_timer()
{
	MAP_Timer32_haltTimer(TIMER32_0_BASE);
}



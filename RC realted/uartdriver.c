/*
 * b_uartdriver.c
 *
 *  Created on: 28.10.2015
 *      Author: hoeft
 */

#include "uartdriver.h"
#include "system.h"
#include "includes.h" //ucos ii stuff
#include <altera_avalon_uart_regs.h>
#include "sys/alt_irq.h"



/**
 * Definition of the UART isr
 */
void ISRUART(void* context, unsigned int id) {
	unsigned int stat;

	INT8U  chr;

	//get serial status
	stat = IORD_ALTERA_AVALON_UART_STATUS(UART_0_BASE);
	//character receiving
	if (stat & ALTERA_AVALON_UART_STATUS_RRDY_MSK) { //Check if UART is ready to offer access to the RXDATA register
		chr = IORD_ALTERA_AVALON_UART_RXDATA(UART_0_BASE);
		IOWR_ALTERA_AVALON_UART_STATUS(UART_0_BASE, 0); //clear the status bits again
		//if (chr == 254) {
		OSQPost(uartQsem, (void*) chr); //write new char in que
		//}
	}
	IOWR_ALTERA_AVALON_UART_STATUS(UART_0_BASE, 0); //reset interrupt
}

/**
 * Initiating UART Core with:
 * 115200 Baudrate
 * refer to /documentation/n2cpu_nii51010.pdf for more info
 */
void diverInitUART(int baud) {
	//Each Bit in the control register enables an IRQ for a corresponding bit in the status register.
	int control = ALTERA_AVALON_UART_CONTROL_RRDY_MSK | //enable Read Interrupts
	ALTERA_AVALON_UART_CONTROL_E_MSK; //enable Exceptions for each Interrupt
	IOWR_ALTERA_AVALON_UART_CONTROL(UART_0_BASE, control); //write contorl bitmask in the control register

	IOWR_ALTERA_AVALON_UART_STATUS(UART_0_BASE, 0x00); //writing 0 in the status Register clears the dcts,e,toe,roe,brk,fe, and pe bits

	//writing up the Baudrate-divisor
	//setting Baudrate-divisor: div = clk / desired Baudrate + 0,5
	int divisor = (int) (50000000 / baud + 0.5);  //div will be 434
	IOWR_ALTERA_AVALON_UART_DIVISOR(UART_0_BASE, divisor);

	//install IRQ service routine
	alt_irq_register(UART_0_IRQ, 0, ISRUART);
	alt_irq_enable(UART_0_IRQ);
}

/**
 * returning current Byte on the UART
 */
INT8U  serialRead(INT16U timeout)
{
	INT8U  rx = (INT8U) OSQPend(uartQsem, timeout, err); //reading latest byte out of OS Message Que
	if (*err != 0) {
		printf("errorcode: %d\n", (int) err);
	}
	return rx;
}

/**
 * reset uart read buffer
 */
void flushUARTBuffer()
{
	OSQFlush(uartQsem); //reset msg que
}


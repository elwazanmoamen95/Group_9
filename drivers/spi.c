/*
 * spi.c
 *
 * Created: 9/2/2024 3:07:37 PM
 *  Author: marwa
 */ 
#include "spi.h"

void init_SPI(void)
{
	// Set MOSI, SCK, and SS as output, MISO as input
	DDRB = (1 << DDB3) | (1 << DDB5) | (1 << DDB2);
	PORTB |= (1 << PORTB2); // Set SS high initially

	// Enable SPI, set as Master, and set clock rate fck/16
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

void SPI_send(uint8_t data)
{
	PORTB &= ~(1 << PORTB2); // Assert SS (Select slave)
	SPDR = data;             // Start transmission
	while (!(SPSR & (1 << SPIF))); // Wait for transmission to complete
	PORTB |= (1 << PORTB2);  // Deassert SS (Deselect slave)
}


#include "spi.h"

void init_SPI(void)
{
    // Set MOSI and SCK as output, others as input
    DDRB = (1 << DDB3) | (1 << DDB5); 
    // Enable SPI, set as Master, and set clock rate fck/16
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0); 
}

void SPI_send(uint8_t data)
{
    SPDR = data;  // Start transmission
    while (!(SPSR & (1 << SPIF)));  // Wait for transmission to complete
}

#ifndef SPI_H
#define SPI_H

#include <avr/io.h>

void init_SPI(void);
void SPI_send(uint8_t data);

#endif

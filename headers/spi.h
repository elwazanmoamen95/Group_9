/*
 * spi.h
 *
 * Created: 9/2/2024 3:07:19 PM
 *  Author: marwa
 */ 


#ifndef SPI_H_
#define SPI_H_
#include <avr/io.h>

void init_SPI(void);
void SPI_send(uint8_t data);
void deselectSlave(void);
void selectSlave(void);


#endif /* SPI_H_ */
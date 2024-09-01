#ifndef ADC_H
#define ADC_H

#include <avr/io.h>

void init_ADC(void);
uint16_t ADC_read(uint8_t channel);

#endif

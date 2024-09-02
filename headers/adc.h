/*
 * ADC.h
 *
 * Created: 9/2/2024 3:04:45 PM
 *  Author: marwa
 */ 


#ifndef ADC_H_
#define ADC_H_
#include <avr/io.h>

void init_ADC(void);
uint16_t ADC_read(uint8_t channel);


#endif /* ADC_H_ */
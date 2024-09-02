/*
 * GPIO.h
 *
 * Created: 9/2/2024 3:01:32 PM
 *  Author: marwa
 */ 
#ifndef GPIO_H
#define GPIO_H

#include <avr/io.h>

void init_GPIO(void);
void setMotorSpeed(uint8_t dutyCycle);

#endif
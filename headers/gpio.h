#ifndef GPIO_H
#define GPIO_H

#include <avr/io.h>

void init_GPIO(void);
void setMotorSpeed(uint8_t dutyCycle);

#endif

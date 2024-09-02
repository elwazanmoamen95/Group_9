/*
 * GPIO.c
 *
 * Created: 9/2/2024 3:02:09 PM
 *  Author: marwa
 */ 
#include "GPIO.h"

void init_GPIO(void)
{
	DDRD |= (1 << DDD0) | (1 << DDD1) | (1 << DDD6); // Set pins PD0, PD1, and PD6 as output
	DDRC &= ~(1 << DDC0); // Set pin PC0 as input (ADC channel 0)

	// Configure encoder pins as inputs
	DDRD &= ~((1 << DDD2) | (1 << DDD3)); // Assuming A and B channels are on PD2 and PD3

	// Set SS pin as output
	DDRB |= (1 << DDB2);  // Ensure SS pin is set as output
	PORTB |= (1 << PORTB2); // Set SS pin high (deselect slave)
}

void setMotorSpeed(uint8_t dutyCycle)
{
	PORTD |= (1 << PORTD0);  // Set pin PD0 high (motor forward direction)
	PORTD &= ~(1 << PORTD1); // Set pin PD1 low
	OCR0A = dutyCycle;  // Set PWM duty cycle to control motor speed
}
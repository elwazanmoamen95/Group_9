/*
 * timer.c
 *
 * Created: 9/2/2024 3:08:18 PM
 *  Author: marwa
 */ 
#include "timer.h"


void init_PWM(void)
{
	DDRD |= (1 << DDD6); // Set PD6 as output (PWM)
	TCCR0A |= (1 << COM0A1) | (1 << WGM00) | (1 << WGM01); // Fast PWM mode, non-inverted
	TCCR0B |= (1 << CS22); // Prescaler 64
	TCNT0 = 0;
	OCR0A = 0;  // Initialize PWM duty cycle to 0
}

void init_Timer1(void)
{
	TCCR1B |= (1 << WGM12);  // Configure Timer1 in CTC mode
	OCR1A = 7812;  // Set compare value for 1-second interval
	TCCR1B |= (1 << CS12) | (1 << CS10);  // Prescaler 1024
	TIMSK1 |= (1 << OCIE1A); // Enable Timer1 compare match interrupt
}
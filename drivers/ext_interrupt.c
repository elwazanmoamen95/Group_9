/*
 * ext_interrupt.c
 *
 * Created: 9/2/2024 3:06:16 PM
 *  Author: marwa
 */ 
#include "ext_interrupt.h"

void init_ExternalInterrupts(void)
{
	// Configure INT0 and INT1 for both rising and falling edges
	EICRA |= (1 << ISC00) | (1 << ISC10);  // INT0 and INT1 trigger on any change
	EIMSK |= (1 << INT0) | (1 << INT1);    // Enable INT0 and INT1 interrupts
}
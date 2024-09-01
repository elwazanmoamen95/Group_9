#include "ext_interrupt.h"

void init_ExternalInterrupts(void)
{
    EICRA |= (1 << ISC00);  // Trigger on any logical change for INT0
    EICRA &= ~(1 << ISC01); // Ensure ISC01 is cleared
    EIMSK |= (1 << INT0);   // Enable INT0 interrupt
}

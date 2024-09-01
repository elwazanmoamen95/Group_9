#include "adc.h"

void init_ADC(void)
{
    ADMUX |= (1 << REFS0);  // Set reference to AVCC
    ADMUX &= ~(1 << REFS1); // Clear REFS1 bit
    ADCSRA |= (1 << ADEN);  // Enable the ADC
    ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2); // Set prescaler to 128
}

uint16_t ADC_read(uint8_t channel)
{
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); // Clear older channel, set new channel
    ADCSRA |= (1 << ADSC);  // Start new conversion
    while (ADCSRA & (1 << ADSC)); // Wait until conversion ends
    return ADCW;  // Return the 10-bit ADC value
}

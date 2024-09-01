/*
 * main.c
 *
 * Created: 9/1/2024 7:12:10 PM
 *  Author: Mo'men
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include "gpio.h"
#include "adc.h"
#include "timer.h"
#include "ext_interrupt.h"
#include "spi.h"

volatile uint16_t pulse_count = 0;
uint16_t adc_value = 0;
uint16_t last_adc_value = 0;
uint8_t dutyCycle = 0;
uint16_t rpm = 0;

#define PPR 24  // Pulses per revolution, replace with your encoder's PPR

int main(void)
{
    init_GPIO();
    init_ADC();
    init_PWM();
    init_ExternalInterrupts();
    init_Timer1();
    init_SPI();

    sei();  // Enable global interrupts

    while (1)
    {
        adc_value = ADC_read(0);  // Read ADC value from channel 0 (potentiometer)

        // Only update PWM if ADC value has changed significantly
        if (abs(adc_value - last_adc_value) > 4) {
            dutyCycle = adc_value / 4;  // Scale 1023 to 255
            setMotorSpeed(dutyCycle);   // Update motor speed
            last_adc_value = adc_value; // Store the last ADC value
        }

        // Small delay to allow smooth operation
        _delay_ms(10);
    }
}

ISR(INT0_vect)
{
    pulse_count++;  // Increment pulse count on each pulse
}

ISR(TIMER1_COMPA_vect)
{
    rpm = (pulse_count * 60) / PPR;  // Calculate RPM
    pulse_count = 0;  // Reset pulse count for the next interval

    // Split the RPM into two bytes and send via SPI
    SPI_send((rpm >> 8) & 0xFF);  // Send the high byte
    SPI_send(rpm & 0xFF);         // Send the low byte
}

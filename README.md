# TASK10.2- Enhance the Turtle 

## Overview

- **Microcontroller:** AVR (e.g., ATmega328)
- **Encoder PPR (Pulses Per Revolution):** 24
- **Update Interval:** 8 seconds
- **Communication:** SPI for RPM data output

## Files

- **`main.c`**: Main application code integrating ADC, timer, encoder, and SPI functionalities.
- **`gpio.h`**: GPIO initialization and motor control functions.
- **`adc.h`**: ADC configuration functions for reading analog input.
- **`timer.h`**: Timer and PWM setup functions.
- **`ext_interrupt.h`**: External interrupt configuration functions for pulse counting.
- **`spi.h`**: SPI communication setup and data transmission functions.

## Functionality

### GPIO Initialization (`gpio.h` and `gpio.c`)

- **`init_GPIO()`**: Configures GPIO pins:
  - **PD0, PD1, PD6**: Set as outputs for motor control and PWM signal.
  - **PC0**: Set as input for ADC channel 0.

### ADC Configuration (`adc.h` and `adc.c`)

- **`init_ADC()`**: Sets up the ADC with:
  - **Reference Voltage**: AVCC (5V).
  - **Prescaler**: 128 to ensure accurate ADC conversion.

- **`ADC_read(uint8_t channel)`**: Reads the analog value from the specified ADC channel.

### Timer Configuration (`timer.h` and `timer.c`)

- **`init_PWM()`**: Configures PWM output on PD6 for motor speed control using Fast PWM mode.
  - **PWM Mode**: Fast PWM, non-inverted.
  - **Prescaler**: 64.

- **`init_Timer1()`**: Configures Timer1 for CTC (Clear Timer on Compare Match) mode to generate an interrupt every 1 seconds.
  - **Compare Match Value (OCR1A)**: 7812 (for an 8 MHz clock and 1024 prescaler).
  - **Prescaler**: 1024.

### External Interrupt Configuration (`ext_interrupt.h` and `ext_interrupt.c`)

- **`init_ExternalInterrupts()`**: Configures external interrupts to count encoder pulses.
  - **INT0**: Triggered on any change for encoder channel A.
  - **INT1**: Triggered on any change for encoder channel B.

### SPI Communication (`spi.h` and `spi.c`)

- **`init_SPI()`**: Initializes SPI in Master mode with a clock rate of fclk/16.
  - **MOSI** (Master Out Slave In): Pin B3.
  - **SCK** (Serial Clock): Pin B5.
  - **SS** (Slave Select): Pin B2 (added for proper SPI operation).

- **`SPI_send(uint8_t data)`**: Sends data via SPI. Waits for the transmission to complete.

### Main Application (`main.c`)

- **Initialization**: Calls functions to set up GPIO, ADC, PWM, external interrupts, Timer1, and SPI.
- **ADC Reading**: Reads from ADC channel 0 to adjust motor speed.
- **Pulse Counting**: Uses external interrupts to count pulses from the encoder.
- **RPM Calculation**: Calculates RPM every 8 seconds using the pulse count.
- **SPI Transmission**: Sends RPM data over SPI to another device or system.

## Setup and Configuration

- **Connect Hardware**
   - **Encoder**: Connect the encoder's output pins to the appropriate input pins (e.g., PD2 and PD3).
   - **Motor**: Connect motor control pins (PD0, PD1) and ensure proper power connections.
   - **SPI**: Connect SPI pins (MOSI, SCK, SS) to the SPI interface of your receiving device.
## OUTPUT ON SPI DEBUGGER
- **frist value is the higher bits in hex**
- **sec value is the lower bits in hex**
- **it sends separately**

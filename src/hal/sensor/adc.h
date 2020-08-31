/* 
Copyright 2020 LifeMech  Inc

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __ADC_SENSOR_HAL_H__
#define __ADC_SENSOR_HAL_H__

#include <avr/io.h>

#include <Arduino.h>

// As multiple sensors use the ADC, add several macros to
// define common functions for the ADC with writing a separate
// driver for it

// Check if ADC is enabled/initialized
#define adcHalEnabled() (bit_is_set(ADCSRA, ADEN))

// Check if ADC is currently doing a conversion or has data ready that hasnt been claimed
#define adcHalBusy() (bit_is_set(ADCSRA, ADSC) || bit_is_set(ADCSRA, ADIF))

// Check if ADC is currently doing a conversion, so data can be retreived once complete
#define adcHalInProgress() (bit_is_set(ADCSRA, ADSC))

// Initialized ADC to use AVcc Reference
#define adcHalInit()                                                \
  do {                                                              \
    ADMUX = bit(REFS0);                                             \
    ADCSRA = bit(ADEN) | bit(ADPS0) |  bit(ADPS1) | bit(ADPS2);     \
  } while (0);
  
// Begin an ADC conversion; first set the correct mux bits, then set the start bit
#define adcHalBegin(pin)                                            \
  do {                                                              \
    ADMUX = (ADMUX & (~0x7)) | (pin & 0x7);                         \
    ADCSRB = (ADCSRB & (~bit(MUX5))) | ((pin > 7) ? bit(MUX5) : 0); \
    ADCSRA |= bit(ADSC);                                            \
  } while (0);

// Get the value from an ADC conversion
#define adcHalGetValue() ADC

// Get the current pin being converted
#define adcHalGetCurrentPin() ((ADMUX & 0x7) | ((ADCSRB & bit(MUX5)) ? 0x80 : 0x00))

// Complete an ADC conversion by clearing the conversion complete flag
#define adcHalComplete()                                            \
  do {                                                              \
    ADCSRA |= bit(ADIF);                                            \
  } while (0);
  
#endif /* __ADC_SENSOR_HAL_H__ */
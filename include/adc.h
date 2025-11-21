#ifndef ADC_H
#define ADC_H

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

void adc_init (uint8_t channel, uint8_t en_IRQ);



#endif
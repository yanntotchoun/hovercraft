#ifndef ADC_H
#define ADC_H

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

void adc3_init (uint8_t en_IRQ);

uint16_t adc_read(volatile uint8_t *flag);
#endif
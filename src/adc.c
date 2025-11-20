
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

void adc_init (uint8_t channel, uint8_t en_IRQ) {
// ADC init
  ADMUX=((1<<ADLAR)|(channel&0x0F)); // "left-aligned" result for easy 8-bit reading. 
  												// AVcc as Aref |(1<<REFS0)
  												// Sets ADC to the specified channel. Can be changed later.
  ADCSRA=(1<<ADEN); //Enables ADC
  if (en_IRQ) ADCSRA|=(1<<ADIE); // enable ADC Complete Interrupt. NOTE: the ISR MUST be defined!!! // meaning that you let an interrupt fly every time a adc conversion is complete
  ADCSRA|=(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2); // ADC clock prescaler
  ADCSRA|=(1<<ADATE); // Continuosly running mode
  ADCSRA|=(1<<ADSC); // Start ADC
}
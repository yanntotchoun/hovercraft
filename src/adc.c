
#include "adc.h"

void adc3_init (uint8_t en_IRQ) {
// ADC init
  ADMUX=((1<<ADLAR)|(3&0x0F)|(1<<REFS0)); // "left-aligned" result for easy 8-bit reading. 
  												// AVcc as Aref |(1<<REFS0)
  												// Sets ADC to the specified channel. Can be changed later.
  ADCSRA |=(1<<ADEN); //Enables ADC
  if (en_IRQ) ADCSRA|=(1<<ADIE); // enable ADC Complete Interrupt. NOTE: the ISR MUST be defined!!! // meaning that you let an interrupt fly every time a adc conversion is complete
  ADCSRA|=(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2); // ADC clock prescaler
}


uint16_t adc_read(uint8_t *flag){
  ADCSRA|=(1<<ADSC); // Start ADC

  while (ADCSRA & (1 << ADSC));// until the bit is 0, so until the adc value is ready to be taken

  *flag= 1;

   return ADC;

}
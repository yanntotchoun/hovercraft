
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

//defining constants
#define const_name 0


//initialize general input/output
static void io_init() {
  DDRB |= (1 << DDB3) | (1 << DDB5);      //set pin3 and pin5 as outputs (for both fans?)
  PORTB &= ~(1 << PORTB5);                //turn initial value of pin5 as off
  PORTB |= (1 << PORTB3);                 //turn initial value of pin3 as on
  DDRC &= ~(1 << DDC0);                   //set pin0 as output
  DIDR0 |= (1 << ADC0D);                  //disable the digital input buffer on ADC0 (apperently improves accuracy and saves power) 
}

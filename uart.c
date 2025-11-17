#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

//defining constants
#define const_name 0

//initialize universal asynchronous receiver/transmitter
static void uart_init() {
  UBRR0H = 0; 
  UBRR0L = 103;                             //lines 10-11 for baud-rate register  
  UCSR0A = 0;                               //status & mode flags register
  UCSR0B = (1 << TXEN0);                    //enable bits (turn TX/RX on, interrupts, data size bits)
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);   //frame format (data bits, stop bits, parity, mode)
}

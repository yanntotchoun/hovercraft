#include "uart.h"


//initialize universal asynchronous receiver/transmitter
void uart_init() {
  // Set baud
  UBRR0H = (uint8_t)(BAUD_RATE >> 8);
  UBRR0L = (uint8_t)(BAUD_RATE & 0xFF); 

  UCSR0A = 0;                               //status & mode flags register
  UCSR0B = (1 << TXEN0);                    //enable bits (turn TX on)
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);   // setting the transmitted bits as 8-bit because if UCSZ01 & UCSz00 = 1 at the same time then it's 8 bits
}


void usart_transmit_char(char data){

/* Wait for empty transmit buffer */
while (!(UCSR0A & (1<<UDRE0)));// UDREn: USART Data Register Empty

/* Put data into buffer, sends the data */
UDR0 = data; // UDRn is the buffer where the 8 bit is stored

}

void usart_transmit_int(uint8_t data){

/* Wait for empty transmit buffer */
while (!(UCSR0A & (1<<UDRE0)));// UDREn: USART Data Register Empty

/* Put data into buffer, sends the data */
UDR0 = data; // UDRn is the buffer where the 8 bit is stored

}

void usart_transmit_16int(uint16_t v) {
   char buf[6];
   uint8_t i = 0;


   if (v == 0) {
       usart_transmit_char('0');
       return;
   }


   while (v > 0) {
       buf[i++] = '0' + (v % 10);
       v /= 10;
   }
   while (i--) {
      usart_transmit_char(buf[i]);
   }
}



void usart_print(char* string){
  while(*string){//until it reaches character null /0
    usart_transmit_char(*string);
    string++;
  }

}
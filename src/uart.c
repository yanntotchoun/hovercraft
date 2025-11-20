#include "uart.h"


//initialize universal asynchronous receiver/transmitter
void uart_init() {
  UBRR0H = 0; //USART baud rate register high bits
  UBRR0L = BAUD_RATE;                             // USART baud rate register low bits setting them to 
  UCSR0A = 0;                               //status & mode flags register
  UCSR0B = (1 << TXEN0)| (1<<RXEN0);                    //enable bits (turn TX/RX on) TXEN0: Transmitter Enable & RXEN0: Receiver Enable 
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
/*
void usart_transmit_16int(uint16_t data){
int8_t buf1;

buf1 = data & 0x0F;
data &= (8>>data);

while (!(UCSR0A & (1<<UDRE0)));// UDREn: USART Data Register Empty
for(int i =0;i<2;i++){

UDR0 =
}

}

*/
void usart_print(char* string){
  while(*string){//until it reaches character null /0
    usart_transmit_char(*string);
    string++;
  }

}
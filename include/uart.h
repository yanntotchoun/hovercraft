#ifndef UART_H
#define UART_H

//INCLUDES
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdlib.h>


//DEFINES
#define BAUD 9600UL
#define BAUD_RATE ((F_CPU/(16UL*BAUD))-1)



//FUNCTION PROTOTYPES
void uart_init();
void usart_transmit_char(char c);
void usart_transmit_int(uint8_t data);
void usart_transmit_16int(uint16_t data);
void usart_print(char* string);







#endif
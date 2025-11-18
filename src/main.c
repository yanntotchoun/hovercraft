#include "io.h"
#include "uart.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h> 


int main(void) {
    uart_init();  // Initialize UART
    
    // Main loop (runs forever)
    while(1) {
        usart_transmit_char('A');
        _delay_ms(1000);
    }
    
    return 0;  // Never reached in embedded systems
}
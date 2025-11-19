#include "io.h"
#include "uart.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h> 


int main(void) {
    uart_init();  // Initialize UART
    
    // Main loop (runs forever)
    while(1) {
        usart_print("Hello, UART!\n");
        _delay_ms(1000);
    }
    
    return 0; 
}
#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <avr/io.h>
#include <util/twi.h>

#ifndef SCL_CLOCK
#define SCL_CLOCK 400000UL
#endif

typedef struct {
    uint8_t TX_new_data:1;
    uint8_t TX_finished:1;
    uint8_t TX_buffer1_empty:1;
    uint8_t TX_buffer2_empty:1;
    uint8_t RX_flag:3;
    uint8_t TWI_ACK:1;
} TWI_Flags_t;

// Global variables (defined in i2c.c)
extern volatile TWI_Flags_t flags;
extern volatile uint8_t TWI_status;
extern volatile uint8_t TWI_byte;

// API
uint8_t TWI_start(uint8_t twi_addr, uint8_t read_write);
void    TWI_stop(void);
uint8_t TWI_write(uint8_t tx_data);
uint8_t TWI_ack_read(void);
uint8_t TWI_nack_read(void);

uint8_t Read_Reg(uint8_t TWI_addr, uint8_t reg_addr);
uint8_t Read_Reg_N(uint8_t TWI_addr, uint8_t reg_addr, uint8_t bytes, int16_t *data);
uint8_t Write_Reg(uint8_t TWI_addr, uint8_t reg_addr, uint8_t value);
void    twi_init(void);

#endif

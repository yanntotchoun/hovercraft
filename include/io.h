#ifndef IO_H
#define IO_H

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
// Constant for Timer1 50 Hz PWM (ICR mode)
#define PWM_TOP 2500
//pin assignment aren't final, I didn't have access to the arduino
//initialize general input/output

// Converter of [0;255] range of x to [0,PWM_top] range for OCR of Timer1 (16 bit)
#define D1B(x) (uint16_t)(((x)*(uint32_t)(PWM_TOP))>>8)



void timer1_50Hz_init(uint8_t en_IRQ);
uint16_t angleToTicks(uint8_t angle);
void set_servo_angle();
void timer0_init ();
void triggerReadingUs();
void timer1_init_for_timing();
void int1_init();
void stopPropFan();

void stopLiftFan();

void startPropFan();
void io_init();


#endif
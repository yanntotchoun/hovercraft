#ifndef TIM_H
#define TIM_H

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>
#define PWM_TOP 2500

  void timer1_reset();
  void timer1_pwm();
  void timer1_setup_servo();

void timer1_50Hz_init(uint8_t en_IRQ);
uint16_t angleToTicks(uint8_t angle);
void set_servo_angle();
void timer0_init ();
void timer1_init_for_timing();
void int1_init();
void int0_init();

#endif
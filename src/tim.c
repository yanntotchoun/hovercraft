
#include "tim.h"



const uint16_t Servo_angle [256]={ //  +/-90 degrees
60,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85, // 0...15
108,108,108,108,108,108,108,108,108,108,108,108,108,108,108,108, // 16...31 0.9ms pulse(108) 
119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119, // 32...47
131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131, // 48...63
142,142,142,142,142,142,142,142,142,142,142,142,142,142,142,142, // 64...79
154,154,154,154,154,154,154,154,154,154,154,154,154,154,154,154, // 80...95
165,165,165,165,165,165,165,165,165,165,165,165,165,165,165,165, // 96...111
177,177,177,177,177,177,177,177,177,177,177,177,188,188,188,188, // center - 1.5ms pulse (188) 112...127
188,188,188,188,198,198,198,198,198,198,198,198,198,198,198,198, // 128...143
208,208,208,208,208,208,208,208,208,208,208,208,208,208,208,208, // 143...159
218,218,218,218,218,218,218,218,218,218,218,218,218,218,218,218, // 160...175
228,228,228,228,228,228,228,228,228,228,228,228,228,228,228,228, // 175...191
238,238,238,238,238,238,238,238,238,238,238,238,238,238,238,238, // 192...207
248,248,248,248,248,248,248,248,248,248,248,248,248,248,248,248, // 208...223
270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270, // 224...239
290,290,290,290,290,290,290,290,290,290,290,290,290,290,290,290  // change the last value
};


static void timer1_reset(){
//stop the timer during configuration
  TCCR1B = 0;
  TCCR1A = 0;
  TCNT1 = 0;
  //clearing all of these registers to 0
}
static void timer1_pwm(){
  TCCR1A|=(1<<COM1A1)|(1<<COM1B1); // non-inv PWM on channels A and B
  TCCR1B|=(1<<WGM13);  //PWM, Phase and Frequency Correct. TOP=ICR1.
  ICR1=PWM_TOP; //50Hz PWM CHECK IF theres actually a hardware latch
}

static void timer1_setup_servo(){
    OCR1A=Servo_angle[113]; //put servo in neutral stare
  OCR1B=0; 
}

static void timer1_interrupts(){
  //clearing all of the interrupts
  
  TIMSK1 = 0; 
  TIMSK2 = 0;
  // Enable Timer1 Overflow Interrupt
   // TIMSK1 |= (1 << TOIE1);
}


void timer1_50Hz_init(uint8_t en_IRQ) { //en_IRQ enables 
  timer1_reset();
  timer1_pwm();
  timer1_setup_servo();
  timer1_interrupts();
  // Prescaler 64: CS11 + CS10
  TCCR1B |= (1 << CS11) | (1 << CS10);
  if (en_IRQ) {
    TIMSK1|=(1<<ICIE1);
  } // enable Input Capture Interrupt. NOTE: the ISR MUST be defined!!! 
}



void int0_init(void){
   // INT0 on PD2
    // Any logical change: ISC01=0, ISC00=1
    EICRA &= ~((1 << ISC01) | (1 << ISC00));  // clear INT0 sense bits
    EICRA |=  (1 << ISC00);                   // any change mode

    EIFR  |= (1 << INTF0); // clear any pending INT0 flag
    EIMSK |= (1 << INT0);  // enable INT0


}

uint16_t angleToTicks(uint8_t angle){
  if (angle>180){
angle=180;//error handling
}
  uint16_t pulse = 1000UL + ((uint32_t)angle*1000UL)/180UL;
  return pulse*2;
}

void set_servo_angle(uint16_t angle){

OCR1A = angleToTicks(angle);
}


 
void timer0_init () { 

    TCCR0A = 0;// free running no pwm complicating the math of the us
    // Prescaler 64 -> 16 MHz / 64 = 250 kHz -> 4 us per tick
    TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler 64 -> 16 MHz / 64 = 250 kHz -> 4 us per tick
    
    TCNT0  = 0;


    TIMSK0 |= (1 << TOIE0);// Enable Timer0 overflow interrupt
}
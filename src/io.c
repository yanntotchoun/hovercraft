
#include "io.h"



const uint16_t Servo_angle [256]={ //  +/-90 degrees
85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85, // 0...15
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
290,290,290,290,290,290,290,290,290,290,290,290,290,290,290,290  // 240...255 
};


void timer1_50Hz_init(uint8_t en_IRQ) { //en_IRQ eanbles 
  TCCR1A|=(1<<COM1A1)|(1<<COM1B1); // non-inv PWM on channels A and B
  TCCR1B|=(1<<WGM13);  //PWM, Phase and Frequency Correct. TOP=ICR1.

  TCNT1 = 0;
  ICR1=PWM_TOP; //50Hz PWM CHECK IF theres actually a hardware latch
  OCR1A=Servo_angle[127]; 
  OCR1B=0; 


  TCCR1B|=((1<<CS11)|(1<<CS10)|(1<<ICES1)); //timer prescaler
  if (en_IRQ) {
    TIMSK1|=(1<<ICIE1);
  } // enable Input Capture Interrupt. NOTE: the ISR MUST be defined!!! 
}

void int1_init(void) {
    // start with rising edge
    EICRA |= (1 << ISC11) | (1 << ISC10);   // INT1 on rising edge
    EIFR  |= (1 << INTF1);                  // clear any pending
    EIMSK |= (1 << INT1);                   // enable INT1
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
// ======= PWM2 and D4 control (8-bit timer0) ===================
  TCCR0A|=(1<<COM0A1)|(1<<COM0B1); //Clear on Compare Match when up-counting. Set on Compare Match when down-counting.  
  TCCR0A|=(1<<WGM00);  //PWM, Phase Correct            
  OCR0A=0; 
  OCR0B=0; 
  TCCR0B|=((1<<CS01)|(1<<CS00)); 
}

void io_init() {
  //P17 (INT1)
  DDRD |= (1 << DDD7);      //set pind3(lift fan) as output
  PORTD |= (1 << PD7);                 //turn initial value of pind3 as on ie lift fan being on is the initial value 
  

  //P1 (PWM)
  DDRD |= (1 << DDD5);                  //set pind6 (propulsion fan) as output
  PORTD &= ~(1 <<PD5);                 //turn initial value of pind6 as off ie propulsion fan being turned off is the initial value DONT FORGET TO SET IT TO 1

  //P9 SERVO, output high
  DDRB |= (1 << DDB1);      //set pinb1(servo) as output
  PORTB &= ~(1 << PB1);     // Start low (will be controlled by PWM)


  //P13
  //ECHO(PD3), input, no pull-up
  DDRD &= ~(1 << DDD3);      //set pind4(ultrasonic sensor) as input ECHO int1 as well
  PORTD &= ~(1<<PD3);// not pull-up resistor

  //TRIG(PB5), output,low
  DDRB |= (1 << DDB5);      //set pind4(ultrasonic sensor) as output TRIG
  PORTB &= ~(1 << PB5);  //turning off the trig pin by default

  //P16
  DDRC &= ~(1 << DDC3);      //set pind4(infrared sensor) as input
  PORTC &= ~(1<<PC3);// not pull-up resistor

  //P7
  //IMU


  //Pull-up resistor assignment
}


void triggerReadingUs(){
  //make sure that it starts low
  PORTB &= ~(1 << PB5);
  _delay_us(2);             // short settle
  //Trig to High
  PORTB |= (1<<PB5);
  _delay_us(10);
  //Trig t Low
  PORTB &= ~(1<<PB5);
}



void stopPropFan(){
  PORTD &= ~(1 <<PD5);
}

void stopLiftFan(){
  PORTD &= ~(1 <<PD3);
}

void startPropFan(){
  PORTD |= (1<<PD5);
}


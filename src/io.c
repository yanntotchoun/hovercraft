
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

// Constant for Timer1 50 Hz PWM (ICR mode)
#define PWM_TOP 2500
//pin assignment aren't final, I didn't have access to the arduino
//initialize general input/output

// Converter of [0;255] range of x to [0,PWM_top] range for OCR of Timer1 (16 bit)
#define D1B(x) (uint16_t)(((x)*(uint32_t)(PWM_TOP))>>8)

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
  ICR1=PWM_TOP; //50Hz PWM
  OCR1A=Servo_angle[127]; 
  OCR1B=0; 
  TCCR1B|=((1<<CS11)|(1<<CS10)); //timer prescaler
  if (en_IRQ) TIMSK1|=(1<<ICIE1); // enable Input Capture Interrupt. NOTE: the ISR MUST be defined!!! 
}

uint16_t angleToTicks(uint8_t angle){
  if (angle>180){
angle=180;//error handling
}
  uint16_t pulse = 1000UL + ((uint32_t)angle*1000UL)/255UL;
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
  DDRD |= (1 << DDD3);      //set pind3(lift fan) as output
  PORTD |= (1 << PD3);                 //turn initial value of pind3 as on ie lift fan being on is the initial value
  
  DDRD |= (1 << DDD6);                  //set pind6 (propulsion fan) as output
  PORTD &= ~(1 <<PD6);                 //turn initial value of pind6 as off ie propulsion fan being turned off is the initial value

  DDRB |= (1 << DDB1);      //set pinb1(servo) as output
  PORTB &= ~(1 << PB1);          //turn initial value of pinb1 as off ie servo motor being turned off is the initial value

  DDRD &= ~(1 << DDD4);      //set pind4(ultrasonic sensor) as input


  DDRC &= ~(1 << DDC3);      //set pind4(infrared sensor) as input


  //Pull-up resistor assignment
}


int read_Us(){
  return PIND & (1<<PD4);
}


int read_Ir(){

  return PINC &(1<<PC3);


}
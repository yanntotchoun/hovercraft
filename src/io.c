
#include "io.h"

static void ultrasonic_init(){
  //P6
  //ECHO(PD2), input, no pull-up
   DDRD &= ~(1 << DDD2);
   PORTD &= ~(1 << PD2);

   //TRIG(PB5), output,low
  DDRB  |= (1 << DDB3);
  PORTB &= ~(1 << PB3);

}
static void propFan_init(){

  //P1 (PWM)
  DDRD |= (1 << DDD5);                  //set pind6 (propulsion fan) as output
  PORTD &= ~(1 <<PD5);                 //turn initial value of pind6 as off ie propulsion fan being turned off is the initial value DONT FORGET TO SET IT TO 1
}

static void liftFan_init(){
   //P17 (INT1)
  DDRD |= (1 << DDD7);      //set pind3(lift fan) as output
  PORTD |= (1 << PD7);                 //turn initial value of pind3 as on ie lift fan being on is the initial value 
}

static void servo_init(){
    //P9 SERVO, output high
  DDRB |= (1 << DDB1);      //set pinb1(servo) as output
  PORTB &= ~(1 << PB1);     // Start low (will be controlled by PWM)

}

static void ir_init(){
  //P16
  DDRC &= ~(1 << DDC3);      //set pind4(infrared sensor) as input
  PORTC &= ~(1<<PC3);// not pull-up resistor
}

static void imu_init(){
  //P7
  DDRC &= ~((1<<PC4)|(1<<PC5));   // inputs
  PORTC |=  (1<<PC4)|(1<<PC5);    // weak pull-ups (replaced by external 4.7k on board)

}

void io_init() {

  cli();//disable all of the interrupts

  DDRC=0; // all pins are inputs. Not really needed as 0 is default value. 


  ultrasonic_init();
  propFan_init();
  liftFan_init();
  servo_init();
  ir_init();
  imu_init();
  


  //Pull-up resistor assignment
  sei();//enables them again
}


void triggerReadingUs(){
  //make sure that it starts low
  PORTB &= ~(1 << PB3);
  _delay_us(2);             // short settle
  //Trig to High
  PORTB |= (1<<PB3);
  _delay_us(10);
  //Trig t Low
  PORTB &= ~(1<<PB3);
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


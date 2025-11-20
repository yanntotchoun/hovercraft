#include "io.h"
#include "uart.h"
#include <avr/interrupt.h>
#include <string.h>

//DEFINES
#define BAT_min 108
//Vbatt min~=13.5V (ADC=108)
#define BAT_warn 133
//Vbatt warn~=14V (ADC=133)

#define ADC_sample_max 4 //Number of ADC samples to average per channel. So every reading is a sample of 4 ADC samples

#define DIST_TH 76   // Distance at where the IR sensor 1.5 *255/5 = 76



//STRUCTS

static volatile struct {
  uint8_t ADC0;
  uint8_t ADC1;
  uint8_t ADC2;
  uint8_t ADC3;
  uint8_t ADC6;
  uint8_t ADC7;
} ADC_data; 

struct Ultrasonic {
    uint16_t startEcho;
    uint16_t endEcho;
    uint16_t distance_ticks;
   volatile uint8_t doneUS;
};
 volatile struct Ultrasonic us;

  //FLAGS
  volatile uint8_t unhandled_interrupt_flag = 0;
  volatile uint8_t icp_state = 0;   // 0 = waiting rising, 1 = waiting falling

  //VARIABLES
  int en_IRQ=1;
  static volatile uint8_t  servo_idx, ADC_sample, V_batt;
  volatile uint8_t unhandled_interrupt_flag = 0;
  extern const uint16_t Servo_angle [256];
 

  //INTERRUPTS
  ISR(__vector_default) { //if there is an interrupt that is not accounted for, set flag to 1
     unhandled_interrupt_flag =1;
  }

  ISR(TIMER1_CAPT_vect) { // Interrupt that calculates the distance from the ultrasonic
    uint16_t time= TCNT1;//Time at the edge found by the ICP counted by the counter

    if(icp_state ==0){
      us.startEcho = time;
      icp_state =1;// we are now waiting for the falling edge
      TCCR1B &= ~(1<<ICES1);//make sure that the time captured is the falling edfe

    }else if  (icp_state==1){
      us.endEcho =time;
      us.distance_ticks= us.endEcho - us.startEcho;
      icp_state =0;// reset to rising edge

      us.doneUS=1;
      TCCR1B |= (1<<ICES1);//make sure that the time captured is the rising edfe

    }
      
  }

int main(void) {
    //VARIABLES
    uint16_t distance_cm;
    uint16_t ticks;

    //INITIALISATION
    sei();//enabling global interrupt
    uart_init();  // Initialize UART
    timer1_50Hz_init(en_IRQ);
    timer0_init ();
    io_init();
    adc_init(3,1);
    
    // Main loop (runs forever)
    while(1) {
      
        ticks=us.distance_ticks;
        distance_cm = (ticks *4)/58;
        triggerReadingUs();

        if(us.doneUS==1){
          us.doneUS=0;

          if(distance_cm<30){//if it detects a wall at 30 cm (arbitrary number)
              PORTD &= ~(1 <<PD5); //stop propulsion fan
              //turn servo left 90 degrees
              us.doneUS=0;
              triggerReadingUs();
              if(distance_cm<5){//if it detects a wall at 5 cm in it's left side (arbitrary number)
                  //turn servo right back 90 d
                //turn servo right for fan
                //turn on fan

              }else{
                  //turn servo right back 90 d
                //turn servo left for fan
                //turn on fan

              }

          


            }
          us.doneUS=0;
        }

        usart_print("Hello, UART!\n");
        _delay_ms(1000);
    }
    
    return 0; 
}
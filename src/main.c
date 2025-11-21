#include "io.h"
#include "uart.h"
#include "adc.h"
#include <avr/interrupt.h>
#include <string.h>

//DEFINES
#define BAT_min 108//Vbatt min~=13.5V (ADC=108)
#define BAT_warn 133//Vbatt warn~=14V (ADC=133)
#define DEBUG 1  // Set to 1 for debugging prints
#define ADC_sample_max 4 //Number of ADC samples to average per channel. So every reading is a sample of 4 ADC samples




//STRUCTS

static volatile struct {
  uint8_t ADC0;
  uint8_t ADC1;
  uint8_t ADC2;
  uint16_t ADC3;
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
  const int16_t  BAR_TH = 51;   //10 cm is 0.25/5*1023 = 51, If upward sensor reads less than 15 cm, the bar is detected.
  const int16_t FRONT_WALL = 30; // If front sensor reads less than 15 cm, a wall is detected
  const int16_t SERVO_LEFT =0;
  const int16_t SERVO_RIGHT =180;
  const int16_t SERVO_DEFAULT =90;
  const int8_t en_IRQ=1;
  extern const uint16_t Servo_angle [256];
  volatile uint8_t irFlag=0;
 





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


  ISR(ADC_vect) { //if there is an interrupt that is not accounted for, set flag to 1
     irFlag =1;
     ADC_data.ADC3=ADC;// the compiler puts ADCH and ADCL inside of ADC
  }





int main(void) {
    //VARIABLES
    uint16_t distance_cm;
    uint16_t distance_adc;
    uint16_t distance_cm_ir;
    float voltage;
    uint16_t ticks;
    uint16_t left_cm=0;
    uint16_t right_cm=0; 

    //INITIALISATION
    uart_init();  // Initialize UART
    timer1_50Hz_init(en_IRQ);
    timer0_init ();
    io_init();
    adc_init(3,1);
    sei();//enabling global interrupt

    
    // Main loop (runs forever)
     while(1) {
  

        if(irFlag==1){
          irFlag=0;
           distance_adc =ADC_data.ADC3;
          if(distance_adc > BAR_TH){
            stopPropFan(); 
            stopLiftFan();
            
          }
        }

        triggerReadingUs();

        if(us.doneUS==1){
           us.doneUS=0;    
            distance_cm = (us.distance_ticks *4)/58;
            if(distance_cm<FRONT_WALL){
              stopPropFan();

        
           
              set_servo_angle(SERVO_LEFT);
              us.doneUS=0; 
              triggerReadingUs();
              while(us.doneUS==0);
              left_cm = (us.distance_ticks *4)/58;
              

              set_servo_angle(SERVO_RIGHT);
              us.doneUS=0; 
              triggerReadingUs();
              while(us.doneUS==0);
              right_cm = (us.distance_ticks *4)/58;


              set_servo_angle(SERVO_DEFAULT);

              if(left_cm>right_cm){
                  set_servo_angle(SERVO_LEFT);
              }else{
                  set_servo_angle(SERVO_RIGHT);
              }
                startPropFan();
            }
        }
        
        voltage =(distance_adc* 5.0f)/1023.0f;
        if (voltage> 0.0f) {
        float distance_cm_ir_f = 27.61f * (1.0f / voltage) - 0.169f;
        if (distance_cm_ir_f < 0.0f) distance_cm_ir_f = 0.0f;
        distance_cm_ir = (uint16_t)distance_cm_ir_f;
      } else {
        distance_cm_ir = 0;
      }
      
        usart_print("Ultrasonic distance = ");
        usart_transmit_16int(distance_cm);
        usart_print("\r\n");
        _delay_ms(1000);
        usart_print("Infrared distance = ");
        usart_transmit_16int(distance_cm_ir);
        
    }
    
    return 0; 
}
#include "io.h"
#include "uart.h"
#include "adc.h"
#include <avr/interrupt.h>
#include <string.h>

//DEFINES
#define BAT_min 108//Vbatt min~=13.5V (ADC=108)
#define BAT_warn 133//Vbatt warn~=14V (ADC=133)
#define DEBUG 1  // Set to 1 for debugging prints, 0 to disable
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
 volatile struct Ultrasonic us= {0};//initialise every value to 0




  //FLAGS
  volatile uint8_t unhandled_interrupt_flag = 0;
  volatile uint8_t int1_state = 0;   // 0 = waiting rising, 1 = waiting falling
  


  //VARIABLES
  const uint16_t  BAR_TH = 51;   //10 cm is 0.25/5*1023 = 51, If upward sensor reads less than 15 cm, the bar is detected.
  const uint16_t FRONT_WALL = 30; // If front sensor reads less than 15 cm, a wall is detected
  const uint16_t SERVO_LEFT =0;
  const uint16_t SERVO_RIGHT =180;
  const uint16_t SERVO_DEFAULT =90;
  const uint8_t en_IRQ=1;
  extern const uint16_t Servo_angle [256];
  volatile uint8_t irFlag=0;
 





  //INTERRUPTS
  ISR(__vector_default) { //if there is an interrupt that is not accounted for, set flag to 1
     unhandled_interrupt_flag =1;
  }


  ISR(INT1_vect) { // Interrupt that calculates the distance from the ultrasonic
    uint16_t time= TCNT1 ;//Time at the edge found by the ICP counted by the counter

    if(int1_state ==0){
      us.startEcho = time;
      int1_state =1;// we are now waiting for the falling edge

       // next interrupt on falling edge: ISC11 = 1, ISC10 = 0
        EICRA &= ~(1 << ISC10);
        EICRA |=  (1 << ISC11);

    }else if  (int1_state==1){
      us.endEcho =time;
      us.distance_ticks= us.endEcho - us.startEcho;
      int1_state =0;// reset to rising edge
      us.doneUS=1;

       // back to rising edge: ISC11=1, ISC10=1
        EICRA |= (1 << ISC10) | (1 << ISC11);

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
    uint16_t left_cm=0;
    uint16_t right_cm=0; 

    //INITIALISATION
    uart_init();  // Initialize UART
    timer1_50Hz_init(en_IRQ);
    timer0_init ();
    io_init();
    adc3_init(1);
    int1_init();
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
        uint32_t timeout = 60000;
        while (!us.doneUS && timeout--) {
            //the program will continue until timer reaches 0 or when a US pulse is found
        }

        if(us.doneUS){   
            uint16_t ticks = us.distance_ticks;
            distance_cm = (ticks * 4U) / 58U;

            #ifdef DEBUG  
            usart_print("Echo: ticks=");
            usart_transmit_16int(ticks);
            usart_print("  cm=");
            usart_transmit_16int(distance_cm);
            usart_print("\r\n");
            #endif

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
        

        
        usart_print("Infrared distance = ");
        usart_transmit_16int(distance_cm_ir);
        usart_print("\r\n");
        _delay_ms(3000);
        
    }
    
    return 0; 
}
#include "io.h"
#include "uart.h"
#include "imu.h"
#include "adc.h"
#include "tim.h"
#include "i2c.h"
#include <string.h>

//DEFINES
#define BAT_min 108//Vbatt min~=13.5V (ADC=108)
#define BAT_warn 133//Vbatt warn~=14V (ADC=133)
#define DEBUG_ADC 1  // Set to 1 for debugging prints, 0 to disable
#define DEBUG_US 0  // Set to 1 for debugging prints, 0 to disable
#define DEBUG 0 // Set to 1 for debugging prints, 0 to disable
#define ADC_sample_max 4 //Number of ADC samples to average per channel. So every reading is a sample of 4 ADC samples


//INDEXES
#define SERVO_LEFT_INDEX    60
#define SERVO_CENTER_INDEX  127
#define SERVO_RIGHT_INDEX   195


#define US_LEFT_INDEX       0
#define US_CENTER_INDEX     127
#define US_RIGHT_INDEX      255







//STRUCTS

static volatile struct {   
    uint16_t acc;
  uint16_t ADC3;
  volatile uint16_t sample;
} ADC_data; 

struct Flags{
  volatile uint8_t unhandled_interrupt_flag;// if something bad happens
  volatile uint8_t int1_state;   // 0 = waiting rising, 1 = waiting falling
  volatile uint8_t ovf_count;  // number of Timer1 overflows during pulse
  volatile uint8_t doneUS;//done with ultrasonic readings
  volatile uint8_t irFlag;//if IR is detected
  volatile uint8_t turnDone;//0: no wall yet ready to be trigerred, 1:wall detected don't trigger again until  to prevent ultrasonic to turning multiple time inside of the algorith
};

volatile struct Flags flag= {0};//initialise every value to 0
 
struct Ultrasonic {
    uint16_t startEcho;
    uint16_t endEcho;
    uint16_t distance_ticks;//distance of the ultrasonic pulse but in ticks
};
 volatile struct Ultrasonic us= {0};//initialise every value to 0





  //CONSTANTS
extern const uint16_t Servo_angle[256];
  const uint16_t  BAR_TH = 51;   //10 cm is 0.25/5*1023 = 51, If upward sensor reads less than 15 cm, the bar is detected.
  const uint16_t FRONT_WALL = 30; // If front sensor reads less than 15 cm, a wall is detected
  const uint16_t US_LEFT =0;
  const uint16_t US_RIGHT =180;
 const uint16_t SERVO_LEFT=60;
const uint16_t SERVO_RIGHT =120;
const uint8_t en_IRQ=0;

 

 




  //INTERRUPTS
  ISR(__vector_default) { //if there is an interrupt that is not accounted for, set flag to 1
     flag.unhandled_interrupt_flag =1;
  }

 ISR(TIMER0_OVF_vect) {// to prevent negative ticks from happenning
    if (flag.int1_state == 1) {
        flag.ovf_count++;// because it is the only time that we are recording the pulse is when we are wiating for rising
    }
}
ISR(TIMER1_CAPT_vect) { }


  ISR(ADC_vect) { //if there is an interrupt that is not accounted for, set flag to 1
        if(ADC_data.sample==0){
            ADC_data.acc=0;
            ADC_data.sample++;
            return;
        }else if(ADC_data.sample<=ADC_sample_max){
            ADC_data.acc+=ADCH;
            ADC_data.sample++;
            return;
        }else if(ADC_data.sample>ADC_sample_max){

            ADC_data.sample=0;
            ADC_data.ADC3 = (uint8_t)ADC_data.acc/ADC_sample_max;
            flag.irFlag =1;
            return;
        }
  }

ISR(INT0_vect) {
  
    uint16_t time = ((uint16_t)flag.ovf_count << 8) | TCNT0;//time inside of the counter of time 0 (bit shifting so it is an 16 bit value to limit oveflows)


    uint8_t echo_high = (PIND & (1 << PD2)) != 0;//variable to see the pin level of PD2 (falling edge and rising edge included)

    if (flag.int1_state == 0 && echo_high) {
        // Rising edge
        us.startEcho = time;//time at the start of the echo pulse
        flag.int1_state = 1;

    } else if (flag.int1_state == 1 && !echo_high) {
        // Falling edge
        us.endEcho = time;//time at the end of the echo pulse
        us.distance_ticks = us.endEcho - us.startEcho; 
        flag.doneUS = 1;//done with the isr , the code continues inside of the main function
        flag.int1_state = 0;// waiting for a rising edge
    }
}

static void sweep_angle(uint16_t idx){

    //LEFT SCAN
                flag.doneUS     = 0;
                flag.int1_state = 0;
                OCR1A = Servo_angle[idx];          
                 _delay_ms(500);
               

}

int main(void) {
    uart_init();                // initialisation of UART
    timer1_50Hz_init(en_IRQ);   // Servo PWM
    timer0_init();              // Timer0 (Free running) US
    io_init();                  // initialiastion of gpio
    adc3_init(1);             
    int0_init();                
    sei();                      //enable interrupts

    while (1) {
       
       
        
        if(flag.irFlag){
            uint8_t distance_ir=ADC_data.ADC3;

             #ifdef DEBUG_ADC
            usart_print("IR Reading");
            usart_transmit_16int(distance_ir);
            usart_print("\r\n");
            #endif

            if(distance_ir<BAR_TH){
                stopLiftFan();
                stopPropFan();
            }
            flag.irFlag=0;
        }
        

        //FRONT SCAN
        flag.doneUS     = 0;
        flag.int1_state = 0;

        triggerReadingUs(); 
        _delay_ms(60);      // give sensor time before we start checking

        // Simple timeout to avoid hanging if no echo
        uint32_t timeout = 60000;
        while (!flag.doneUS && timeout--) {}// if there's a reading this while statement will get completely bypassed

            if (flag.doneUS) {
                uint16_t ticks = us.distance_ticks;
                uint16_t distance_cm = (ticks * 4U) / 58U;
                uint16_t distance_l=0;
                uint16_t distance_r=0;

                if (distance_cm > FRONT_WALL + 5) {
                 flag.turnDone = 0;
                }
               
                
                #ifdef DEBUG_US
                usart_print("Echo: ticks=");
                usart_transmit_16int(ticks);
                usart_print("  cm=");
                usart_transmit_16int(distance_cm);
                usart_print("\r\n");
                #endif

                if(distance_cm<=FRONT_WALL&& !flag.turnDone){
                    flag.turnDone=1; 

                    stopPropFan();
                    //stopLiftFan();
                    
                    //LEFT SCAN
                    sweep_angle(US_LEFT_INDEX);
                     timeout = 60000;

                    triggerReadingUs(); 
                    _delay_ms(60);      // give sensor time before we start checking
                   // while (!flag.doneUS && timeout--) {}// if there's a reading this while statement will get completely bypassed


                     uint16_t ticks_l = us.distance_ticks;
                    distance_l = (ticks_l * 4U) / 58U;
                    
                     _delay_ms(400);

                
                      #ifdef DEBUG_US
                    usart_print("LEFT SENSOR: ticks=");
                    usart_transmit_16int(ticks_l);
                    usart_print("  cm=");
                    usart_transmit_16int(distance_l);
                    usart_print("\r\n");
                    #endif

                    //RIGHT SCAN
                
                    sweep_angle(US_RIGHT_INDEX);

                      timeout = 60000;
                    triggerReadingUs(); 
                    _delay_ms(60);      // give sensor time before we start checking

                    // Simple timeout to avoid hanging if no echo
                        
                    //while (!flag.doneUS && timeout--) {}// if there's a reading this while statement will get completely bypassed
                    uint16_t ticks_r = us.distance_ticks;
                    distance_r =  (ticks_r * 4U) / 58U;


                     _delay_ms(400);
                    
                      #ifdef DEBUG
                    usart_print("TURN RIGHT");
                    usart_print("\r\n");
                    #endif

    
                      #ifdef DEBUG_US
                    usart_print("RIGHT SENSOR: ticks=");
                    usart_transmit_16int(ticks_r);
                    usart_print("  cm=");
                    usart_transmit_16int(distance_r);
                    usart_print("\r\n");
                    #endif

                    sweep_angle(SERVO_CENTER_INDEX);
                    _delay_ms(400);
                

              
                    if(distance_l>=distance_r){
                         sweep_angle(SERVO_RIGHT_INDEX);

                    }else{
                        sweep_angle(SERVO_LEFT_INDEX);
                    }
                    startPropFan();


                    #ifdef DEBUG
                    usart_print("LOGIC");
                    usart_print("\r\n");
                    #endif
                } 

            } else {
                
                #ifdef DEBUG_US
                usart_print("No echo (timeout)\r\n");
                #endif
                
           }

         _delay_ms(200);
    }

    return 0;
    /*
while(1){

    usart_transmit_16int(getImuAccX());
    usart_transmit_16int(getImuAccY());
    usart_transmit_16int(getImuAccZ());
}
*/
}

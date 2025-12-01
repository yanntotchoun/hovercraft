#include "io.h"
#include "uart.h"
#include "imu.h"
#include "adc.h"
#include "tim.h"
#include "i2c.h"
#include "imuLogic.h"
#include <string.h>

//DEFINES
#define BAT_min 108//Vbatt min~=13.5V (ADC=108)
#define BAT_warn 133//Vbatt warn~=14V (ADC=133)
#define DEBUG_ADC 0  // Set to 1 for debugging prints, 0 to disable
#define DEBUG_US 0  // Set to 1 for debugging prints, 0 to disable
#define DEBUG 0 // Set to 1 for debugging prints, 0 to disable
#define ADC_sample_max 4 //Number of ADC samples to average per channel. So every reading is a sample of 4 ADC samples


//INDEXES 

/*
NEGATIVE SIDE

| Degree | Index |
| ------ | ----- |
| -84    | 0     |
| -75    | 14    |
| -60    | 36    |
| -45    | 57    |
| -30    | 79    |
| -20    | 94    |
| -10    | 109   |
| -5     | 116   |
| -1     | 126   |
| -0.5   | 127   |
| 0      | 128   |


POSITIVE SIDE

| Degree | Index |
| ------ | ----- |
| +0.5   | 128   |
| +1     | 129   |
| +5     | 140   |
| +10    | 148   |
| +20    | 163   |
| +30    | 178   |
| +45    | 199   |
| +60    | 221   |
| +75    | 243   |
| +84    | 255   |


| Meaning    | Degree | Index | Servo_angle[index] |
| ---------- | ------ | ----- | ------------------ |
| Full left  | -84    | 0     | 85                 |
| Center     | 0      | 128   | 188                |
| Full right | +84    | 255   | 290                |



60

295
*/
#define SERVO_LEFT_INDEX   0// 90 degrees
#define SERVO_CENTER_INDEX  113
#define SERVO_RIGHT_INDEX   255//90 degrees

//14 
#define US_LEFT_INDEX       0
#define US_CENTER_INDEX     113
#define US_RIGHT_INDEX      255

#define BAR_TH  211 //SHOULD BE A PERFECT VALUE
#define FRONT_WALL 30





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
  volatile uint8_t imuStop;//0 = stop  ;1=continue
  volatile uint8_t turnDone;//0: no wall yet ready to be trigerred, 1:wall detected don't trigger again until  to prevent ultrasonic to turning multiple time inside of the algorith
};



volatile struct Flags flag= {0};//initialise every value to 0
 
struct Ultrasonic {
    uint16_t startEcho;
    uint16_t endEcho;
    uint16_t distance_ticks;//distance of the ultrasonic pulse but in ticks
};
 volatile struct Ultrasonic us= {0};//initialise every value to 0


struct IMU_data imu;


  //CONSTANTS
extern const uint16_t Servo_angle[256];
const uint8_t en_IRQ=0;

//VARIABLES

volatile uint32_t imuTime=0;



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

ISR(TIMER2_OVF_vect){
    imuTime+=1024;
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


static void turning_logic(uint16_t idx){

     sweep_angle(idx);
        _delay_ms(40);
        startPropFan();
    sweep_angle(SERVO_CENTER_INDEX);
    
}

int main(void) {
    uart_init();                // initialisation of UART
    timer1_50Hz_init(en_IRQ);   // Servo PWM
    timer0_init();              // Timer0 (Free running) US
    io_init();                  // initialiastion of gpio
    adc3_init(0);               // no interrupts for adc
    int0_init(); 

    
   
    sei();                      //enable interrupts

    imu_init();    //initialising the imu  
    imu_calibration(100);//500 samples before it keeps going
    imu.timeSinceLast =0.02f;//trying to put it here
   

    while (1) {
       
        /*startPropFan();
        uint16_t distance_ir= adc_read(&flag.irFlag);
        uint16_t raw = distance_ir >> 6;
            
        uint16_t cm = 4800 / (raw - 20);

        if (cm > 80) cm = 80;
        if (cm < 10) cm = 10;*/

        

        drift_algorithm();
        

        //if(flag.irFlag){
            
           /*
             #ifdef DEBUG_ADC
            usart_print("IR Reading =");
            usart_transmit_16int(cm);
            usart_print("\r\n");
            usart_print("IR Reading RAW=");
            usart_transmit_16int(raw);
            usart_print("\r\n");
            #endif
            */
/*            if(raw>BAR_TH){
                stopLiftFan();
                stopPropFan();
                return 0;//use a flag here instead of return
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
               
                /*
                #ifdef DEBUG_US
                usart_print("Echo: ticks=");
                usart_transmit_16int(ticks);
                usart_print("  cm=");
                usart_transmit_16int(distance_cm);
                usart_print("\r\n");
                #endif
                */

                /*if(distance_cm<=FRONT_WALL&& !flag.turnDone){
                    flag.imuStop =1;//stop the imu during the turning logic
                    flag.turnDone=1; 

                    stopPropFan();
                    _delay_ms(100);
                    
                    //LEFT SCAN
                    sweep_angle(US_LEFT_INDEX);
                    triggerReadingUs(); 
                   // _delay_ms(60);      // give sensor time before we start checking
                    _delay_ms(300);// make it slower a bit to put less stress on the fan structure
                     timeout = 60000;

                   
                   // while (!flag.doneUS && timeout--) {}// if there's a reading this while statement will get completely bypassed


                     uint16_t ticks_l = us.distance_ticks;
                    distance_l = (ticks_l * 4U) / 58U;
                    
                     _delay_ms(400);

                    /*
                      #ifdef DEBUG_US
                    usart_print("LEFT SENSOR: ticks=");
                    usart_transmit_16int(ticks_l);
                    usart_print("  cm=");
                    usart_transmit_16int(distance_l);
                    usart_print("\r\n");
                    #endif
                    */

                    //RIGHT SCAN
                
                    /*sweep_angle(US_RIGHT_INDEX);
                     triggerReadingUs(); 
                    _delay_ms(60);      // give sensor time before we start checking
                     _delay_ms(300);// make it slower a bit to put less stress on the fan structure

                      timeout = 60000;
                    

                    // Simple timeout to avoid hanging if no echo
                        
                    //while (!flag.doneUS && timeout--) {}// if there's a reading this while statement will get completely bypassed
                    uint16_t ticks_r = us.distance_ticks;
                    distance_r =  (ticks_r * 4U) / 58U;


                     _delay_ms(400);
                    
                    
                    /*
                      #ifdef DEBUG_US
                    usart_print("RIGHT SENSOR: ticks=");
                    usart_transmit_16int(ticks_r);
                    usart_print("  cm=");
                    usart_transmit_16int(distance_r);
                    usart_print("\r\n");
                    #endif
                        */
                    /*sweep_angle(SERVO_CENTER_INDEX);
                    _delay_ms(400);
                

              
                    if(distance_l>distance_r){
                        turning_logic(SERVO_LEFT_INDEX);

                    }else{
                        turning_logic(SERVO_RIGHT_INDEX);
                    }
                    


                     flag.imuStop =0;
                } 

            } 
                
                

                
            

         _delay_ms(20);
         */
    }
    

    return 0;
}
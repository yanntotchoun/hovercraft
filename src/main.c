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
//DRIFTING
#define DRIFT_LEFT_INDEX       105
#define DRIFT_RIGHT_INDEX      121

#define BAR_TH  211 //SHOULD BE A PERFECT VALUE
#define FRONT_WALL 30

//TIME DEFINES
#define TURN_LEFT_TIME 700
#define TURN_RIGHT_TIME 700


//STRUCTS

static volatile struct {
  uint16_t ADC3;
} ADC_data;

typedef enum{
    STATE_STOP,
    STATE_FORWARD,
    STATE_SWEEPING,
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT,

}hovercraft_states_t;

volatile hovercraft_states_t state = STATE_FORWARD;// setting the default state as going forward
uint32_t stateTimer =0;


struct Flags{
  volatile uint8_t unhandled_interrupt_flag;// if something bad happens
  volatile uint8_t int1_state;   // 0 = waiting rising, 1 = waiting falling
  volatile uint8_t ovf_count;  // number of Timer1 overflows during pulse
  volatile uint8_t doneUS;//done with ultrasonic readings
  volatile uint8_t irFlag;//if IR is detected
  volatile uint8_t frontWall;//0: no wall , 1: wall
  volatile uint8_t barDetected;//0: no bar , 1: bar
volatile uint8_t errorReading;//0: no error in us reading 1: error
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
const uint8_t en_IRQ=0;

//VARIABLES
volatile uint32_t msTicks = 0;



//INTERRUPTS
  ISR(__vector_default) { //if there is an interrupt that is not accounted for, set flag to 1
     flag.unhandled_interrupt_flag =1;
  }

 ISR(TIMER0_OVF_vect) {// to prevent negative ticks from happenning
    if (flag.int1_state == 1) {
        flag.ovf_count++;// because it is the only time that we are recording the pulse is when we are wiating for rising
    }
}
ISR(TIMER1_CAPT_vect) {

 }


ISR(TIMER2_COMPA_vect) {
    msTicks++;      // every 1 ms
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

static void set_angle(uint16_t idx){
                OCR1A = Servo_angle[idx];
                 _delay_ms(500);
}

static void readUsFront(){
    flag.doneUS     = 0;
    flag.int1_state = 0;
    triggerReadingUs();
    _delay_ms(60);// give time to the sensor

    // Simple timeout to avoid hanging if no echo
    uint32_t timeout = 60000;
    while (!flag.doneUS && timeout--) {}// if there's a reading this while statement will get completely bypassed

    if(flag.doneUS){
        uint16_t ticks = us.distance_ticks;
        uint16_t distance_cm = (ticks * 4U) / 58U;

        if(distance_cm<FRONT_WALL){
            flag.frontWall=1;
        }

    }
}

static void readIr(){
     uint16_t distance_ir= adc_read(&flag.irFlag);
    uint16_t raw = distance_ir >> 6;

    if(flag.irFlag){
        if(raw>BAR_TH)
       flag.barDetected=1;
    }
}

static void ArtDriftAlgorithm(){
    OCR1A = Servo_angle[DRIFT_LEFT_INDEX];
    _delay_ms(20);
   OCR1A = Servo_angle[DRIFT_RIGHT_INDEX];
    _delay_ms(20);

    }

static uint16_t readUsSide(){
    flag.doneUS     = 0;
    flag.int1_state = 0;
    triggerReadingUs();
    _delay_ms(60);// give time to the sensor

    // Simple timeout to avoid hanging if no echo
    uint32_t timeout = 60000;
    while (!flag.doneUS && timeout--) {}// if there's a reading this while statement will get completely bypassed

    if(flag.doneUS){
        uint16_t ticks = us.distance_ticks;
        uint16_t distance_cm = (ticks * 4U) / 58U;

       return distance_cm;

    }
    return 999;//errorr handling
}

int main(void) {
    uart_init();                // initialisation of UART
    timer1_50Hz_init(en_IRQ);   // servo PWM
    timer0_init();              // timer0 (Free running) US
    timer2_init();              // timer2 for turning logic
    io_init();                  // initialiastion of gpio
    adc3_init(0);               // no interrupts for adc
    int0_init();                // interrupts for ultrasonic sensor


    sei();                      //enable interrupts


    while (1) {



            switch (state)
            {
            case STATE_FORWARD :

            startPropFan();
            set_angle(SERVO_CENTER_INDEX);
           


            readUsFront();

            if(flag.frontWall){
                flag.frontWall = 0;
                state = STATE_SWEEPING;
                stateTimer =0;//set the timer to 0
            }
            // ArtDriftAlgorithm(); if needed
            readIr();
            if(flag.barDetected){
               state = STATE_STOP;
            }

            break;

            case STATE_SWEEPING:
            stateTimer = 0;
            stopPropFan();

            set_angle(US_LEFT_INDEX);
            _delay_ms(300);
            uint16_t distance_l = readUsSide();

            set_angle(US_RIGHT_INDEX);
            _delay_ms(300);
            uint16_t distance_r = readUsSide();


            set_angle(US_CENTER_INDEX);

               if(distance_l == 999 && distance_r ==999){// both are invalid
                state = STATE_TURN_RIGHT;
                flag.errorReading =1;
               }else if (distance_l==999){
                state = STATE_TURN_RIGHT;
                flag.errorReading =1;
               }else if (distance_r == 999){
                state = STATE_TURN_LEFT;
                flag.errorReading =1;
               } else{
                     if(distance_l>distance_r){
                        state =STATE_TURN_RIGHT;

                    }else{
                        state =STATE_TURN_LEFT;
                    }

               }
               stateTimer= msTicks;

                break;

            case STATE_TURN_LEFT :

             if(flag.errorReading){
                usart_print("ERROR \n");
              }

             startPropFan();
             set_servo_angle(SERVO_LEFT_INDEX);

              uint32_t timeLeft = msTicks-stateTimer;

            if(timeLeft>=TURN_LEFT_TIME){
                set_servo_angle(SERVO_CENTER_INDEX);
                state = STATE_FORWARD;
                stateTimer = msTicks;//setting it to 0 would make the logic bad

               }

            break;

            case STATE_TURN_RIGHT :

             if(flag.errorReading){
                usart_print("ERROR \n");
              }

             startPropFan();
             set_servo_angle(SERVO_RIGHT_INDEX);

            uint32_t timeRight = msTicks-stateTimer;

            if(timeRight>=TURN_RIGHT_TIME){

            set_angle(SERVO_CENTER_INDEX);
            state = STATE_FORWARD;
            stateTimer = msTicks;//setting it to 0 would make the logic bad

            }

            break;

            case STATE_STOP:
            stopLiftFan();
            stopPropFan();
            return 0;

            break;


            default:
                break;
            }
             _delay_ms(20);
                    }



          return 0;
    }



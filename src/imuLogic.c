#include "imuLogic.h"
#include "uart.h"

// yaw integration clamp (safety, not control aggressiveness)
#define DEBUG_IMU 1

// yaw error at which servo hits full deflection
#define YAW_FULL_DEFLECTION  15.0f //in degrees

#define GYRO_SENS_Z      65.5f    // LSB per deg/s at ±500 dps
#define GZ_CUTOFF_LSB    15000  // it removes bad gyro values
#define GZ_DEADBAND_LSB  20     // value that removes very small changes to 0, to prevent small changes from messing up the yaw

/*.    ALTERNATE DEADBAND VALUES
10   very sensitive, servo reacts easily  
20   standard  
30   heavy damping, stable but less responsive  
*/



// ADDITIONNAL PWM CONFIGURATION
#define PWM_CENTER  113.0f        // neutral PWM value )
#define PWM_SPAN     80.0f         // maximum angle from the center for the power steering , so max_right = Center+ span and max_left = center- span(physical limit)




struct IMU_data imu;


void imu_calibration(uint16_t samples){

long sum=0;//long because we need 32 bits and we want it to be signed
uint16_t count=0;

usart_print("CALIBRATING THE IMU FOR THE BIAS. KEEP STILL\n");

while(count<samples){

      imu.gyroZraw=readImuGyroZ();

    //this rejects garbage values
    if((imu.gyroZraw >= -GZ_CUTOFF_LSB && imu.gyroZraw <= GZ_CUTOFF_LSB)){
        sum+= imu.gyroZraw;
        count++;

        usart_print("sum = ");
        usart_transmit_16int((int16_t)sum);//FOR DEBUGGING REMOVE IF YOU DON'T NEED IT
         usart_print("\n");
    }
     _delay_ms(5);

}
usart_print("DONE WITH THE CALIBRATION\n");
imu.gyroZbias=(float)sum/(float)count;
imu.yaw =0.0f;

}


  uint16_t yaw_to_pwm(float yawDegres){
    if(yawDegres > MAX_YAW) yawDegres = MAX_YAW;
    if(yawDegres < -MAX_YAW) yawDegres= -MAX_YAW;

    float t = yawDegres/YAW_FULL_DEFLECTION; //convert into a percentage (how far you are from full steering)

    if(t > 1.0f) t = 1.0f;
    if(t < -1.0f) t = -1.0f;

    float pwm = PWM_CENTER + PWM_SPAN*(-t);// determines the pwm motion according to the center value and the max span of the servo

    if(pwm<0.0f)pwm=0.0f;
    if(pwm>65535.0f)pwm =65535.0f;



    return (uint16_t)(pwm + 0.5f);//conversion to integer
}



void drift_algorithm(volatile uint8_t *flag){

    if(*flag){//stop the imu during the turning logic
        return;
    }
    
    int16_t gzRawUnfiltered= readImuGyroZ();

    //FIlTERING 

    //removing terrible value
    if (gzRawUnfiltered > GZ_CUTOFF_LSB || gzRawUnfiltered < -GZ_CUTOFF_LSB) {
            return;
    }
    //removing bias
    float gzCorr = (float)gzRawUnfiltered - imu.gyroZbias;

       // deadband to counter slow drift
        if (gzCorr > -GZ_DEADBAND_LSB && gzCorr < GZ_DEADBAND_LSB) {
            gzCorr = 0;
        }
    //convert in deg/s
    float gz_dps = (float)gzCorr / GYRO_SENS_Z;


    imu.yaw -= gz_dps * imu.timeSinceLast;

    uint16_t pwm = yaw_to_pwm(imu.yaw);
    OCR1A = pwm;
    
   
   
  
     usart_print("YAW: ");
     usart_transmit_float(imu.yaw);
     usart_print("\n");
     usart_print("PWM: ");
     usart_transmit_16int(pwm);
     usart_print("\n");
      usart_print("gz degrees per second: ");
     usart_transmit_float(gz_dps);
     usart_print("\n");
     

  

}

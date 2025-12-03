#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>


#define GYRO_SENS     65.5f     
#define PWM_CENTER    188.0f
#define PWM_SPAN      80.0f
#define Kp            10.0f      // proportional gain

extern volatile uint32_t imuTime;     
extern int16_t readImuGyroZ();
extern void usart_print(const char*);
extern void usart_transmit_float(float f);

float gyro_bias = 0.0f;
float yaw = 0.0f;
static uint32_t lastTime = 0;

// CALIBRATION 
void imu_calibration(uint16_t samples)
{
    long sum = 0;

    usart_print("Calibrating gyro... keep craft STILL\n");

    for(uint16_t i=0; i<samples; i++)
    {
        sum += readImuGyroZ();
       _delay_ms(10);  
    }

    gyro_bias = (float)sum / samples;
    yaw = 0.0f;
    lastTime  = imuTime;     
    usart_print("Calibration done\n");
}
void imu_reset_yaw(void)
{

    yaw = 0.0f;
    lastTime = imuTime;  // prevents a huge dt jump 
}

void drift_algorithm(uint8_t flag)
{

     if (flag) return;   
   

    uint32_t now = imuTime;
    float dt = (now - lastTime) * 1e-6f;  
    lastTime = now;

    if (dt <= 0 || dt > 0.1f) dt = 0.02f;  


    int16_t raw = readImuGyroZ();
    float gz = (raw - gyro_bias) / GYRO_SENS;  // deg/s

    yaw += gz * dt;

    if (yaw > 180) yaw -= 360;
    if (yaw < -180) yaw += 360;


float correction = Kp * yaw;


if (correction >  PWM_SPAN) correction =  PWM_SPAN;
if (correction < -PWM_SPAN) correction = -PWM_SPAN;

    float pwm = PWM_CENTER + correction;
   OCR1A = (uint16_t)(pwm + 0.5f);

    usart_print("Yaw: ");
    usart_transmit_float(yaw);
    usart_print("  PWM: ");
    usart_transmit_float(pwm);
    usart_print("\n");
}
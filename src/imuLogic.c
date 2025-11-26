#include "imuLogic.h"

#define MAX_YAW 84
#define GYRO_SENS_Z 65.5f// LSB per deg/s

extern const uint16_t Servo_angle[256];

struct IMU_data{
    //RAW DATA FROM IMU
    uint16_t gyroXraw;
    uint16_t gyroYraw;
    uint16_t gyroZraw;
    uint16_t accXraw;
    uint16_t accYraw;
    uint16_t accZraw;

    
    //CONVERTED VALUES
    float gyroX;
    float gyroY;
    float gyroZ;
    float accX;
    float accY;
    float accZ;

    //BIAS
   float gyroXbias;
    float gyroYbias;
   float gyroZbias;
   float accXbias;
    float accYbias;
   float accZbias;
    

    //INTEGRATED VALUES
    float yaw;
    float timeSinceLast;//dt in yaw =yaw +angular velocity *dt
    //uint8_t tick20;
    
};

void imu_calibration(){
imu.yaw =0.0f;
imu.gyroZbias=100.0f;

imu.timeSinceLast =0.02f;

}


void getValuesImu(){
    imu.gyroZraw=readImuGyroZ();

    imu.gyroZ = ( (float)imu.gyroZraw - imu.gyroZbias ) / GYRO_SENS_Z;
}

 uint8_t yaw_to_idx(float y){
    if(y > MAX_YAW) y = MAX_YAW;
    if(y < -MAX_YAW) y = -MAX_YAW;
    float t = (y + MAX_YAW) / (2 * MAX_YAW);
    return (uint8_t)(t * 255.0f + 0.5f);
}



void drift_algorithm(){

    imu.yaw -= imu.gyroZ * imu.timeSinceLast;

    uint8_t idx = yaw_to_idx(imu.yaw);
        OCR1A = Servo_angle[idx];
        _delay_ms(60);





}

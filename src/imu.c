#include "imu.h"


void imu_init(){
    Write_Reg(IMU_ADDR, REG_PWR_MGMT_1, 0x01);// picking the most stable clock source
  _delay_ms(5);
    Write_Reg(IMU_ADDR, REG_DIG_LOW_PASS, 0x03);// so we are applying a digiral low pass filter to filter out the noise
  Write_Reg(IMU_ADDR, REG_SMPLRT_DIV, 0x04);//sample rate at 200 Hz
  Write_Reg(IMU_ADDR, REG_GYRO_CFG, 0x08);//0x08 = FS_SEL = 1 → ±500°/s
  Write_Reg(IMU_ADDR, REG_ACCEL_CFG, 0x00);//0x00 = AFS_SEL = 0 → ±2g
}

int16_t imu_read_word(uint8_t reg) {
    int16_t val = 0;
    if (Read_Reg_N(IMU_ADDR, reg, 2, &val) != 0) {
        return 0; // or some error code
    }
    return val;
}
uint16_t getImuAccX(){
    return imu_read_word(0x3B);
}
uint16_t getImuAccY(){
    return imu_read_word(0x3D);
}
uint16_t getImuAccZ(){
    return imu_read_word(0x3F);
}

uint16_t getImuTemp(){
    return imu_read_word(0x41);
}

uint16_t readImuGyroX(){
    return imu_read_word(0x43);
}
uint16_t readImuGyroY(){
    return imu_read_word(0x45);
}
uint16_t readImuGyroZ(){
    return imu_read_word(0x47);
}




/*
 * mpu6050.h
 *
 *  Created on: Aug 24, 2024
 *      Author: uzayr
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#define DEVICE_ADDRESS 0x68

#define FS_GYRO_250 0
#define FS_GYRO_500 8
#define FS_GYRO_1000 9
#define FS_GYRO_2000 10

#define FS_ACC_2G 0
#define FS_ACC_4G 8
#define FS_ACC_8G 9
#define FS_ACC_16G 10

#define REG_CONFIG_GYRO 27
#define REG_CONFIG_ACC 28
#define REG_USR_CTRL 107
#define REG_DATA_ACC 59
#define REG_DATA_GYRO 67

#define ACCEL_SCALE_FACTOR (1 / 16384.0)    // 8192 LSB/g for ±4g
#define GYRO_SCALE_FACTOR  (1 / 131.0) // 65536 LSB/°/s for ±500°/s

#define GYRO_WEIGHT 0.96

void mpu6050_init();
float mpu6050_read();

#endif /* INC_MPU6050_H_ */

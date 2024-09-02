#include <mpu6050.h>
#include <main.h>
#include <stdio.h>
#include <math.h>
#include <motors.h>

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;

#define DEVICE_ADDRESS 0x68 // MPU6050 I2C address
#define REG_PWR_MGMT_1 0x6B
#define REG_CONFIG_GYRO 0x1B
#define REG_CONFIG_ACC 0x1C
#define REG_USR_CTRL 0x6A
#define REG_DATA_ACC 0x3B
#define REG_DATA_GYRO 0x43

#define GYRO_WEIGHT 0.96 // Weight for gyroscope data in complementary filter

void mpu6050_init(){
    HAL_StatusTypeDef ret;

    // Check if the device is ready
    ret = HAL_I2C_IsDeviceReady(&hi2c1, (DEVICE_ADDRESS << 1), 1, 100);
    if (ret == HAL_OK){
        printf("The device is ready\n");
    } else {
        printf("The device is not ready. Check cables \n");
    }

    // Wake up the MPU6050 by writing 0 to the power management register
    uint8_t temp_data = 0x00;
    ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS << 1), REG_PWR_MGMT_1, 1, &temp_data, 1, 100);
    if (ret == HAL_OK){
        printf("Exiting from sleep mode\n");
    } else {
        printf("Failed to exit sleep mode \n");
    }

    // Configure gyroscope sensitivity
    temp_data = 0x00; // Full scale range of ±250°/s
    ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS << 1), REG_CONFIG_GYRO, 1, &temp_data, 1, 100);
    if (ret == HAL_OK){
        printf("Configuring gyroscope \n");
    } else {
        printf("Failed to configure gyroscope \n");
    }

    // Configure accelerometer sensitivity
    temp_data = 0x00; // Full scale range of ±2g
    ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS << 1), REG_CONFIG_ACC, 1, &temp_data, 1, 100);
    if (ret == HAL_OK){
        printf("Configuring accelerometer \n");
    } else {
        printf("Failed to configure accelerometer \n");
    }

    // Disable sleep mode (already done above, so this might be redundant)
    temp_data = 0;
    ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS << 1), REG_USR_CTRL, 1, &temp_data, 1, 100);
    if (ret == HAL_OK){
        printf("Disabling sleep mode\n");
    } else {
        printf("Failed to disable sleep mode \n");
    }
}

float mpu6050_read(){
    uint8_t data[6]; // Buffer for 6 bytes to store data

    float x_acc, y_acc, z_acc;
    float x_gyro, y_gyro, z_gyro;

    static uint32_t previous_time = 0;
    float current_time;
    float delta_time;
    float delta_time_seconds;
    float pitch;

    static float pitch_gyro_prev = 0.0;

    // Read accelerometer data
    HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1), REG_DATA_ACC, 1, data, 6, 100);
    x_acc = ((int16_t)((data[0] << 8) | data[1])) / 16384.0;
    y_acc = ((int16_t)((data[2] << 8) | data[3])) / 16384.0;
    z_acc = ((int16_t)((data[4] << 8) | data[5])) / 16384.0;

    // Calculate pitch angle based on accelerometer data
    float pitch_acc = atan2(y_acc, z_acc) * (180.0 / M_PI) + 1.58;  // Convert to degrees
    //float pitch_acc = (atan(-1 * x_acc / sqrt(pow(y_acc, 2) + pow(z_acc, 2))) * 180 / M_PI) + 1.58;
    // Read gyroscope data
    HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1), REG_DATA_GYRO, 1, data, 6, 100);
    x_gyro = ((int16_t)((data[0] << 8) | data[1])) / 131.0;
    y_gyro = ((int16_t)((data[2] << 8) | data[3])) / 131.0;
    z_gyro = ((int16_t)((data[4] << 8) | data[5])) / 131.0;

    // Optional: Correct the outputs with the calculated error values (if calibration is performed)

    x_gyro += 0.56;
    y_gyro -= 2;
    z_gyro += 0.79;

    // Get the current time from the timer
    current_time = HAL_GetTick();  // Get current time in milliseconds
    delta_time = current_time - previous_time;
    previous_time = current_time;
    delta_time_seconds = delta_time / 1000.0;  // Convert milliseconds to seconds

    // Calculate pitch from gyroscope
    float pitch_gyro = pitch_gyro_prev + (y_gyro * delta_time_seconds);

    // Store the previous pitch_gyro for the next iteration
    pitch_gyro_prev = pitch_gyro;

    // Combine accelerometer and gyroscope pitch using a complementary filter
    pitch = GYRO_WEIGHT * pitch_gyro + (1.0 - GYRO_WEIGHT) * pitch_acc;

    float control_output = 0;

    pid_control(pitch, &control_output, &delta_time_seconds);

    return control_output;
}

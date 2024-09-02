/*
 * motors.c
 *
 *  Created on: Aug 31, 2024
 *      Author: uzayr
 */

#include <mpu6050.h>
#include <main.h>
#include <stdio.h>
#include <math.h>
#include <motors.h>

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;

void pid_control(float pitch, float *control_output, float *delta_time_seconds) {

	static float setpoint = 0.0;  // Desired pitch angle
	static float error = 0.0, previous_error = 0.0;
	static float integral = 0.0, derivative = 0.0;

	static float Kp = 1.0;  // Proportional gain
	static float Ki = 0.0;  // Integral gain
	static float Kd = 0.0;  // Derivative gain

    // Calculate the error between the desired setpoint and the current pitch
    error = setpoint - pitch;

    // Calculate the integral (sum of errors over time)
    integral += error * *delta_time_seconds;

    // Calculate the derivative (rate of change of error)
    derivative = (error - previous_error) / *delta_time_seconds;

    // Compute the control output using the PID formula
    *control_output = Kp * error + Ki * integral + Kd * derivative;

    // Store the current error as previous_error for the next loop iteration
    previous_error = error;

    // Use the control output to adjust your system (e.g., motor speed)
    // Example: Adjust the PWM signal for the motors based on the control_output

    //adjust_motors(control_output);
}

void adjust_motors(float control_output) {

    // Convert control_signal to a PWM value or direction signal for motor control
    int pwm_value = (int)(control_output * 16383.75);

    if (pwm_value > MAX_PWM) pwm_value = MAX_PWM;
    if (pwm_value < MIN_PWM) pwm_value = MIN_PWM;

    if (control_output > 0) {
    	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0); // Forward direction
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0); // Forward direction
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
      } else if (control_output < 0) {
    	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1); // Forward direction
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1); // Forward direction
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
      } else {
    	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0); // Forward direction
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0); // Forward direction
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
      }

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_value);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_value);
}

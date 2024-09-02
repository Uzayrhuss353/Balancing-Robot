/*
 * motors.h
 *
 *  Created on: Aug 31, 2024
 *      Author: uzayr
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_
#define MAX_PWM  65535
#define MIN_PWM  0



#endif /* INC_MOTORS_H_ */

void pid_control(float pitch, float *control_output, float *delta_time_seconds);
void adjust_motors(float control_output);

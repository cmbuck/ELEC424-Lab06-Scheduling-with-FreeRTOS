/*
 * motor.h - Functions for controlling the Crazyflie motors
 *
 *  Created on: Oct 30, 2014
 *      Author: Jeremy Hunt, Christopher Buck
 *       Class: ELEC 424
 */

#include "scheduling.h"

#ifndef MOTOR_H_
#define MOTOR_H_

#define MOTOR_PWM_FREQ			36000
/* When the divider is 1, the timer multiplies the clock by 2 */
#define MOTOR_TIM_FREQ			(2*APB2_FREQ)
#define MOTOR_TIM_PRESCALER		(1-1)
#define MOTOR_TIM_PERIOD		((MOTOR_TIM_FREQ/(MOTOR_TIM_PRESCALER+1))/MOTOR_PWM_FREQ)

typedef enum {
	Motor1, Motor2, Motor3, Motor4
} MotorEnum;

#define MOTOR1_PIN		1
#define MOTOR2_PIN		0
#define MOTOR3_PIN		9
#define MOTOR4_PIN		8

#define MOTOR1_TIM		TIM3
#define MOTOR2_TIM		TIM3
#define MOTOR4_TIM		TIM4
#define MOTOR3_TIM		TIM4

/*
 * Motor 1: Tim3 channel 4
 * Motor 2: Tim3 channel 3
 * Motor 3: Tim4 channel 4
 * Motor 4: Tim4 channel 3
 */
#define MOTOR1_SET_COMPARE(a, b)	TIM_SetCompare4(a, b);
#define MOTOR2_SET_COMPARE(a, b)	TIM_SetCompare3(a, b);
#define MOTOR3_SET_COMPARE(a, b)	TIM_SetCompare4(a, b);
#define MOTOR4_SET_COMPARE(a, b)	TIM_SetCompare3(a, b);

/* Multiplier to convert speed to units in terms of the timer period */
#define MOTOR_MULT	(MOTOR_TIM_PERIOD / 100)

void motor_init();
void motor_set(MotorEnum m, int duty);

#endif /* MOTOR_H_ */

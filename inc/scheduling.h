/*
 * scheduling.h - A basic scheduler.
 *
 *  Created on: Oct 12, 2014
 *      Author: Jeremy Hunt and Christopher Buck
 */
#ifndef SCHEDULING_H_
#define SCHEDULING_H_

#include <sys/types.h>

/* 10ms Scheduler Ticks */
#define HIGH_PRIO_TICK_FREQ		100
#define LOW_PRIO_TICK_FREQ		2
#define APB1_FREQ				36000000
#define APB2_FREQ				36000000

#define TIM1_FREQ				(2*APB2_FREQ)
#define TIM1_PRESCALER			360
#define TIM1_PERIOD				((TIM1_FREQ/TIM1_PRESCALER)/HIGH_PRIO_TICK_FREQ)
#define TIM1_PRIO				1
#define TIM1_SUB_PRIO			0

#define TIM2_FREQ				(2*APB1_FREQ)
#define TIM2_PRESCALER			36000
#define TIM2_PERIOD				((TIM2_FREQ/TIM2_PRESCALER)/LOW_PRIO_TICK_FREQ)
#define TIM2_PRIO				1
#define TIM2_SUB_PRIO			1

#define MOTOR_PWM_FREQ			36000
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

#define MOTOR_MULT	(MOTOR_TIM_PERIOD / 100)

/* Create a quick boolean type */
typedef int boolean;
#define TRUE	1
#define FALSE	0

#endif /* SCHEDULING_H_ */

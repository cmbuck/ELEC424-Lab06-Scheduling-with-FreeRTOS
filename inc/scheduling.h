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

#define MOTOR_PWM_FREQ			36000
#define MOTOR_TIM_FREQ			(2*APB2_FREQ)
#define MOTOR_TIM_PRESCALER		(1-1)
#define MOTOR_TIM_PERIOD		((MOTOR_TIM_FREQ/(MOTOR_TIM_PRESCALER+1))/MOTOR_PWM_FREQ)


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

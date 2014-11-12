/*
 * motor.c - Functions for controlling the Crazyflie motors
 *
 *  Created on: Oct 30, 2014
 *      Author: Jeremy Hunt, Christopher Buck
 *       Class: ELEC 424
 */

#include "motor.h"

#include "stm32f10x.h"
#include "stm32f10x_conf.h"

#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>

static void motor_pwm_init() {
	TIM_OCInitTypeDef outputChannelInit;
	outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
	outputChannelInit.TIM_Pulse = 0; /* Set to 0% */
	outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
	outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

	//Motors 3&4
	TIM_OC3Init(TIM4, &outputChannelInit);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM4, &outputChannelInit);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	//Motors 1&2
	TIM_OC3Init(TIM3, &outputChannelInit);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM3, &outputChannelInit);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
}

/*
 * Initialize the GPIO which controls the Motors in alternate function mode (through TIM3/4)
 */
static void motor_pins_init() {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef gpioStructure;
	gpioStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	/* Note the 'AF' here */
	gpioStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpioStructure);

	gpioStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	/* Note the 'AF' here */
	gpioStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpioStructure);
}

/*
 * Initialize the timer and set the clock to the external oscillator
 */
static void motor_timers_init() {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_TimeBaseInitTypeDef timerInitStructure;
	timerInitStructure.TIM_Prescaler = MOTOR_TIM_PRESCALER;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = MOTOR_TIM_PERIOD;
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &timerInitStructure);
	TIM_Cmd(TIM4, ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	timerInitStructure.TIM_Prescaler = MOTOR_TIM_PRESCALER;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = MOTOR_TIM_PERIOD;
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &timerInitStructure);
	TIM_Cmd(TIM3, ENABLE);
}

/*
 * Initializes the motors. Must be called before motor_set.
 */
void motor_init() {
	motor_pins_init();
	motor_timers_init();
	motor_pwm_init();
}

/*
 * Set a motor to be on or off
 * @param  m : Select the motor to set the state for
 * 		Can be one of the following values
 * 		Motor1, Motor2, Motor3, Motor4
 * @param  s : Select the speed for the motor
 * 		Can be between 0 and 100
 */
void motor_set(MotorEnum m, int duty) {
	/* Calculate the number of pulse counts in ticks to get the needed desired duty cycle. */
	int pulse = duty * MOTOR_MULT;
	switch (m) {
	case Motor1:
		MOTOR1_SET_COMPARE(MOTOR1_TIM, pulse);
		break;
	case Motor2:
		MOTOR2_SET_COMPARE(MOTOR2_TIM, pulse);
		break;
	case Motor3:
		MOTOR3_SET_COMPARE(MOTOR3_TIM, pulse);
		break;
	case Motor4:
		MOTOR4_SET_COMPARE(MOTOR4_TIM, pulse);
		break;
	}
}


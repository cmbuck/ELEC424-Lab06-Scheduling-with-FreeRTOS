/*
 * Name: scheduling.c
 * Authors: Jeremy Hunt and Christopher Buck
 * Date: 10-11-14
 * Description: A simple task scheduler for varying priority tasks.
 */

#include "scheduling.h"

#include "sys_clk_init.h"
#include "lab06_task.h"

#include "stm32f10x.h"
#include "stm32f10x_conf.h"

#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>

void motorPwmInit() {
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
static void motorInit() {
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
static void motorTimersInit() {
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
 * Set a motor to be on or off
 * @param  m : Select the motor to set the state for
 * 		Can be one of the following values
 * 		Motor1, Motor2, Motor3, Motor4
 * @param  s : Select the speed for the motor
 * 		Can be between 0 and 256
 */
void motorSet(MotorEnum m, int duty) {
	//MULT: preprocessor define -
	//multiplier to convert speed to units in
	//terms of the timer period
	int pulse = duty * MOTOR_MULT;	//pulse width (in ticks) to achieve the
								//desired duty cycle
	switch (m)
	{
	case Motor1 :
		MOTOR1_SET_COMPARE(MOTOR1_TIM, pulse);
		break;
	case Motor2 :
		MOTOR2_SET_COMPARE(MOTOR2_TIM, pulse);
		break;
	case Motor3 :
		MOTOR3_SET_COMPARE(MOTOR3_TIM, pulse);
		break;
	case Motor4 :
		MOTOR4_SET_COMPARE(MOTOR4_TIM, pulse);
		break;
	}
}

/*
 * Initialize the GPIO which controls the LED
 */
static void ledsInit() {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef gpioStructure;
	gpioStructure.GPIO_Pin = GPIO_Pin_5;
	gpioStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpioStructure);

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE);

	gpioStructure.GPIO_Pin = GPIO_Pin_4;
	gpioStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpioStructure);

	GPIO_WriteBit(GPIOB, GPIO_Pin_4, Bit_RESET);
}

/*
 * Initialize the High Priority Timer to run at 10 ms
 */
static void tickHighPrioTimerInit() {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	TIM_TimeBaseInitTypeDef timerInitStructure;
	timerInitStructure.TIM_Prescaler = TIM1_PRESCALER;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = TIM1_PERIOD;
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &timerInitStructure);
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

	TIM_Cmd(TIM1, ENABLE);

	/* Initialize the interrupt for TIM1 */
	NVIC_InitTypeDef nvicStructure;
	nvicStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = TIM1_PRIO;
	nvicStructure.NVIC_IRQChannelSubPriority = TIM1_SUB_PRIO;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);
}

/*
 * Initialize the Low Priority Timer to run at 500 ms
 */
static void tickLowPrioTimerInit() {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseInitTypeDef tim2Init;
	tim2Init.TIM_Prescaler = TIM2_PRESCALER;
	tim2Init.TIM_CounterMode = TIM_CounterMode_Up;
	tim2Init.TIM_Period = TIM2_PERIOD;
	tim2Init.TIM_ClockDivision = TIM_CKD_DIV1;
	tim2Init.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &tim2Init);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	TIM_Cmd(TIM2, ENABLE);

	/* Initialize the interrupt for TIM2 */
	NVIC_InitTypeDef tim2Nvic;
	tim2Nvic.NVIC_IRQChannel = TIM2_IRQn;
	tim2Nvic.NVIC_IRQChannelPreemptionPriority = 0;
	tim2Nvic.NVIC_IRQChannelSubPriority = 1;
	tim2Nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&tim2Nvic);
}

/*
 * Main function.  Initializes the GPIO, Timers, and
 */
int main() {
	if (sysClkInit72Mhz() != SUCCESS) {
		for (;;) {}
	}
	ledsInit();
	tickHighPrioTimerInit();
	tickLowPrioTimerInit();
	motorInit();
	motorTimersInit();
	motorPwmInit();

	/* Loop. Forever. */
	for (;;) {
		/* When we aren't doing stuff in interrupts, run the logger. */
		logDebugInfo();
	}
}

/*
 * Interrupt service routines.
 * Function callers setup in "startup_stm32f10x_md.s"
 */

/* High priority tasks */
void TIM1_UP_IRQHandler() {
	static unsigned int updateSensorCount = 0;

	/* Clear the interrupt flag. */
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

		/* Detect and emergency once every 10ms */
		detectEmergency();

		/* Update the sensors once every 100ms */
		updateSensorCount++;
		if (updateSensorCount >= 10) {
			refreshSensorData();
			updateSensorCount = 0;
		}
	}
}

/* Low priority tasks, every 500ms */
void TIM2_IRQHandler() {
	static unsigned int greenLedState = 0;
	static unsigned int redLedState = 0;
	static unsigned int oneHzCount = 0;

	/* Check the interrupt and clear the flag. */
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		/* Toggle the green led */
		if (greenLedState) {
			GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_RESET);
		} else {
			GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_SET);
		}
		/* Flip the state for next operation */
		greenLedState = 1 - greenLedState;

		/* Update all the things that need to happen at 1 Hz */
		oneHzCount++;
		if (oneHzCount >= 2) {
			/* Toggle the red led */
			if (redLedState) {
				GPIO_WriteBit(GPIOB, GPIO_Pin_4, Bit_RESET);
			} else {
				GPIO_WriteBit(GPIOB, GPIO_Pin_4, Bit_SET);
			}
			/* Flip the state for next operation */
			redLedState = 1 - redLedState;

			/* Calculate our orientation. */
			calculateOrientation();

			/* Update the motors */
			MotorSpeeds newSpeeds = {0, 0, 0, 0};
			updatePid(&newSpeeds);
			motorSet(Motor1, 25*newSpeeds.m1);
			motorSet(Motor2, 25*newSpeeds.m2);
			motorSet(Motor3, 25*newSpeeds.m3);
			motorSet(Motor4, 25*newSpeeds.m4);

			oneHzCount = 0;
		}
	}
}


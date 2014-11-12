/*
 * Name: scheduling.c
 * Authors: Jeremy Hunt and Christopher Buck
 * Date: 10-11-14
 * Description: A simple task scheduler for varying priority tasks.
 */

#include "scheduling.h"

#include "sys_clk_init.h"
#include "lab06_task.h"
#include "motor.h"

#include "stm32f10x.h"
#include "stm32f10x_conf.h"

#include "task.h"

#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>

/*
 * Initialize the GPIO which controls the LED
 */
static void leds_init() {
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

void update_sensor_task(void* args);

#define UPDATE_SENSOR_TASK_PRIO		(tskIDLE_PRIORITY + 2)
#define CALC_ORIENTATION_TASK_PRIO	(tskIDLE_PRIORITY + 1)

/*
 * Main function.  Initializes the GPIO, Timers, and
 */
int main() {
//	__disable_irq();

	if (sys_clk_init_72mhz() != SUCCESS) {
		for (;;) {}
	}
	leds_init();
	tickHighPrioTimerInit();
	tickLowPrioTimerInit();
	motorInit();
	motorTimersInit();
	motorPwmInit();

	/* Create the task for updating sensors */
	xTaskCreate( update_sensor_task, "update_sensor_task", 1024, NULL, UPDATE_SENSOR_TASK_PRIO, NULL);

	/* Start the scheduler. This enables interupts. */
	vTaskStartScheduler();

	/* Loop. Forever. */
	for (;;) {
		/* When we aren't doing stuff in interrupts, run the logger. */
		logDebugInfo();
	}
}

/*
 * Updates the sensor data periodicly.
 */
void update_sensor_task(void* args) {
	while (1) {

	}
}

/*
 * Calculates the orientation. Blocks on the new sensor data semaphore.
 */
void calculate_orientation_task(void* args) {
	while (1) {

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


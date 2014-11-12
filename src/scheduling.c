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

#include "FreeRTOS.h"
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
void calculate_orientation_task(void* args);

#define UPDATE_SENSOR_TASK_PRIO		(tskIDLE_PRIORITY + 2)
#define CALC_ORIENTATION_TASK_PRIO	(tskIDLE_PRIORITY + 1)

/*
 * Main function.  Initializes the GPIO, Timers, and
 */
int main() {
	/* Disable interrupts so nothing funny happens */
	__disable_irq();

	/* Initialize the hardware */
	if (sys_clk_init_72mhz() != SUCCESS) {
		for (;;) {}
	}
	leds_init();
	motor_init();

	/* Create the task for updating sensors */
	xTaskCreate(update_sensor_task, "update_sensor_task", 1024, NULL,
			UPDATE_SENSOR_TASK_PRIO, NULL);
	/* And updating the orientation calculation */
	xTaskCreate(calculate_orientation_task, "calculate_orientation_task", 1024, NULL,
			CALC_ORIENTATION_TASK_PRIO, NULL);

	/* Start the scheduler. This enables interupts. */
	vTaskStartScheduler();

	/* When we aren't doing stuff in interrupts, run the logger. */
	logDebugInfo();

	return 1;
}

/*
 * Updates the sensor data periodicly.
 */
void update_sensor_task(void* args) {
	static unsigned int updateSensorCount = 0;
	(void)args;

	while (1) {
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

/*
 * Calculates the orientation. Blocks on the new sensor data semaphore.
 */
void calculate_orientation_task(void* args) {
	static unsigned int greenLedState = 0;
	static unsigned int redLedState = 0;
	static unsigned int oneHzCount = 0;
	(void)args;

	while (1) {
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
			motor_set(Motor1, 25*newSpeeds.m1);
			motor_set(Motor2, 25*newSpeeds.m2);
			motor_set(Motor3, 25*newSpeeds.m3);
			motor_set(Motor4, 25*newSpeeds.m4);

			oneHzCount = 0;
		}
	}
}

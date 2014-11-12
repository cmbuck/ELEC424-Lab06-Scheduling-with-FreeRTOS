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
#include "semphr.h"

#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>

/* Local helper functions */
static void led_init();

/* Define all of the tasks and their relationships */
void log_debug_info_task(void* args);
void update_sensor_task(void* args);
void calc_orientation_task(void* args);
void update_pid_task(void* args);
void detect_emergency_task(void* args);
void green_led_task(void* args);

#define UPDATE_PID_INTERVAL			(1000)
#define GREEN_LED_INTERVAL			(500)
#define CALC_ORIENTATION_INTERVAL	(100)
#define DETECT_EMERGENCY_INTERVAL	(10)

#define LOG_DEBUG_INFO_TASK_PRIO	(tskIDLE_PRIORITY + 0)
#define UPDATE_PID_TASK_PRIO		(tskIDLE_PRIORITY + 1)
#define CALC_ORIENTATION_TASK_PRIO	(tskIDLE_PRIORITY + 2)
#define UPDATE_SENSOR_TASK_PRIO		(tskIDLE_PRIORITY + 2)
#define GREEN_LED_TASK_PRIO			(tskIDLE_PRIORITY + 3)
#define DETECT_EMERGENCY_TASK_PRIO	(tskIDLE_PRIORITY + 3)

SemaphoreHandle_t sensor_new_data;

/*
 * Main function.  Initializes everything and makes it go.
 */
int main() {
	/* Disable interrupts so nothing funny happens */
	__disable_irq();

	/* Initialize the hardware */
	if (sys_clk_init_72mhz() != SUCCESS) {
		for (;;) {}
	}
	led_init();
	motor_init();

	/* Create the semaphore for the blocking calc orientation */
	sensor_new_data = xSemaphoreCreateBinary();
	if (sensor_new_data == NULL) {
		/* We badly failed. Give up. */
		return 1;
	}

	/* Look for emergencies every 10ms */
	xTaskCreate(detect_emergency_task, "detect_emergency_task", 256, NULL,
			DETECT_EMERGENCY_TASK_PRIO, NULL);
	/* Create the task for updating sensors */
	xTaskCreate(update_sensor_task, "update_sensor_task", 256, NULL,
			UPDATE_SENSOR_TASK_PRIO, NULL);
	/* And updating the orientation calculation */
	xTaskCreate(calc_orientation_task, "calculate_orientation_task", 256, NULL,
			CALC_ORIENTATION_TASK_PRIO, NULL);
	/* Update the motors every 1 second */
	xTaskCreate(update_pid_task, "update_pid_task", 256, NULL,
			UPDATE_PID_TASK_PRIO, NULL);
	/* Toggle the green LED at 2Hz */
	xTaskCreate(green_led_task, "green_led_task", 256, NULL,
			GREEN_LED_TASK_PRIO, NULL);
	/* Log the debug info when we get a chance */
	xTaskCreate(log_debug_info_task, "log_debug_info_task", 256, NULL,
			LOG_DEBUG_INFO_TASK_PRIO, NULL);

	/* Start the scheduler. This enables interrupts. */
	vTaskStartScheduler();

	return 1;
}

/*
 * Initialize the GPIO which controls the LED
 */
static void led_init() {
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
 * Just continually log the debug info in a loop.
 */
void log_debug_info_task(void* args) {
	(void)args;

	while (1) {
		logDebugInfo();
	}
}

/*
 * Blink the green LED at 1Hz
 */
void green_led_task(void* args) {
	(void)args;
	static unsigned int greenLedState = 0;
	TickType_t xLastWakeTime;

	/* Initialize the xLastWakeTime variable with the current time. */
	xLastWakeTime = xTaskGetTickCount();
	while (1) {
		/* Toggle the green led */
		if (greenLedState) {
			GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_RESET);
		} else {
			GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_SET);
		}
		/* Flip the state for next operation */
		greenLedState = 1 - greenLedState;


		/* Toggle at 2Hz */
		vTaskDelayUntil(&xLastWakeTime, GREEN_LED_INTERVAL);
	}
}

/*
 * Detects any emergency. Runs every 10ms at the highest priority.
 */
void detect_emergency_task(void* args) {
	(void) args;
	TickType_t xLastWakeTime;

	/* Initialize the xLastWakeTime variable with the current time. */
	xLastWakeTime = xTaskGetTickCount();
	while (1) {
		/* Look for an emergency every 10ms */
		detectEmergency();
		vTaskDelayUntil(&xLastWakeTime, DETECT_EMERGENCY_INTERVAL);
	}
}

/*
 * Updates the sensor data periodically.
 */
void update_sensor_task(void* args) {
	(void) args;
	TickType_t xLastWakeTime;

	/* Initialize the xLastWakeTime variable with the current time. */
	xLastWakeTime = xTaskGetTickCount();
	while (1) {
		/* Update the sensors once every 100ms */
		refreshSensorData();

		/* Trigger the new sensor data semaphore */
		xSemaphoreGive(sensor_new_data);

		vTaskDelayUntil(&xLastWakeTime, 100);
	}
}

/*
 * Calculates the orientation. Blocks on the new sensor data semaphore.
 */
void calc_orientation_task(void* args) {
	(void)args;

	while (1) {
		/* Block waiting for new data */
		xSemaphoreTake(sensor_new_data, portMAX_DELAY);

		/* Calculate our orientation. */
		calculateOrientation();
	}
}

/*
 * Update the motors once per second. Also toggle the red LED.
 */
void update_pid_task(void* args) {
	static unsigned int redLedState = 0;
	(void) args;
	TickType_t xLastWakeTime;

	/* Initialize the xLastWakeTime variable with the current time. */
	xLastWakeTime = xTaskGetTickCount();
	while (1) {
		/* Toggle the red led */
		if (redLedState) {
			GPIO_WriteBit(GPIOB, GPIO_Pin_4, Bit_RESET);
		} else {
			GPIO_WriteBit(GPIOB, GPIO_Pin_4, Bit_SET);
		}
		/* Flip the state for next operation */
		redLedState = 1 - redLedState;

		/* Update the motors */
		MotorSpeeds newSpeeds = {0, 0, 0, 0};
		updatePid(&newSpeeds);
		motor_set(Motor1, 25*newSpeeds.m1);
		motor_set(Motor2, 25*newSpeeds.m2);
		motor_set(Motor3, 25*newSpeeds.m3);
		motor_set(Motor4, 25*newSpeeds.m4);

		vTaskDelayUntil(&xLastWakeTime, UPDATE_PID_INTERVAL);
	}
}

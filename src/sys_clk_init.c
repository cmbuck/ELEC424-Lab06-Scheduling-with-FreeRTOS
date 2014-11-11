/*
 * Name: sys_clk_init.c
 * Authors: Jeremy Hunt and Christopher Buck
 * Date: 10-11-14
 * Description: Sets up the clock tree system on the crazyflie quad-copter.
 */

#include "sys_clk_init.h"

#include "stm32f10x_conf.h"

ErrorStatus sysClkInit72Mhz() {
	/* Reset the clock system back to reset state. */
	RCC_DeInit();

	/* 72 MHz requires the flash to have 2 wait states. Also turn on prefetch. */
	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    FLASH_SetLatency(FLASH_Latency_2);

    /* Set up some of the basic peripheral clocks to run off SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
    RCC_PCLK2Config(RCC_HCLK_Div2);
    RCC_PCLK1Config(RCC_HCLK_Div2);

	/*
	 * Select the 16 MHz HSE, divided by 2, for the PLL source with a
	 * 9x multiplier. This give a final clock speed of 72 MHz
	 */
	RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_9);

	/* Turn on the HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait until the HSE is stable. */
	if (RCC_WaitForHSEStartUp() != SUCCESS) {
		RCC_DeInit();
		return ERROR;
	}

	/* Turn on the PLL */
	RCC_PLLCmd(ENABLE);

	/* Wait for the PLL to lock. */
	volatile uint32_t startUpCounter = 0;
	FlagStatus pllStatus = RESET;
	do {
		pllStatus = RCC_GetFlagStatus(RCC_FLAG_PLLRDY);
		startUpCounter++;
		/*
		 * Just use the HSE timeout value here.
		 * There doesn't seem to be a better one.
		 */
	} while ((startUpCounter < HSE_STARTUP_TIMEOUT) && (pllStatus == RESET));
	if (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {
		RCC_DeInit();
		return ERROR;
	}

	/* Switch the SYSCLK over to the PLL. */
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

	/* Wait until PLL is used as system clock source */
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}

	/* Finally, turn off the HSI to save some power. */
	RCC_HSICmd(DISABLE);

	return SUCCESS;
}


/*
 * Name: sys_clk_init.c
 * Authors: Jeremy Hunt and Christopher Buck
 * Date: 10-11-14
 * Description: Sets up the clock tree system on the crazyflie quad-copter.
 */

/**
 * @brief Sets up SYSCLK from reset to be 72 MHz.
 * Sets up the clock tree to run off a 16 MHz HSE so that the internal
 * SYSCLK is 72 MHz. The chip may be in another clock state at the
 * begining of this function.
 * @retval An ErrorStatus enumuration value:
 *   - SUCCESS: SYSCLK is running and ready at 72 MHz
 * 	 - ERROR: The clock failed to start and was left in the default state
 */
ErrorStatus sys_clk_init_72mhz();

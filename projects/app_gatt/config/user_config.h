/**
 ****************************************************************************************
 *
 * @file user_config.h
 *
 * @brief Configuration of the BT function
 *
 * Copyright (C) Beken 2019
 *
 ****************************************************************************************
 */
#ifndef USER_CONFIG_H_
#define USER_CONFIG_H_

//// Sim UART to H4TL
#define VIRTUAL_UART_H4TL		0

#define BEKEN_CLKOFF_ENABLE		0

#define P192_PUB_KEY_GEN_ENABLE	0

#define BEKEN_PAGE_DELAY_ENABLE	0

#define BEKEN_BT_ADDR_NOT_CONST 1

#define KEIL_SIM_DEBUG_ENABLE	0	////0

 /******************************************************************************
  *############################################################################*
  * 							SYSTEM MACRO CTRL                              *
  *############################################################################*
  *****************************************************************************/

//如果需要使用GPIO进行调试，需要打开这个宏
#define GPIO_DBG_MSG					0
//UART使能控制宏
#define UART_PRINTF_EN				    1
//蓝牙硬件调试控制
#define DEBUG_HW						0

/*******************************************************************************
 *#############################################################################*
 *								DRIVER MACRO CTRL                              *
 *#############################################################################*
 ******************************************************************************/

//DRIVER CONFIG
#define UART_DRIVER						1
#define GPIO_DRIVER						1
#define AUDIO_DRIVER					0
#define RTC_DRIVER						0
#define ADC_DRIVER						1
#define ADC_CALIB					    0
#define I2C_DRIVER						0
#define PWM_DRIVER						1

#endif // USER_CONFIG_H_

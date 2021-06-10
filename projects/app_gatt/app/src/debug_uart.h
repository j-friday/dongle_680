/**
 ****************************************************************************************
 *
 * @file bim_uart.h
 *
 * @brief UART Driver for HCI over UART operation.
 *
 * Copyright (C) Beken 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef _DEBUG_UART_H_
#define _DEBUG_UART_H_

/**
 ****************************************************************************************
 * @defgroup UART UART
 * @ingroup DRIVERS
 * @brief UART driver
 *
 * @{
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdbool.h>          // standard boolean definitions
#include <stdint.h>           // standard integer functions

 

/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initializes the UART to default values.
 *****************************************************************************************
 */

typedef  void (*UART_DEBUG_CALLBACK_T)(const char *fmt,...);   

extern UART_DEBUG_CALLBACK_T uart_cb;

#define stack_printf  uart_cb
//extern _ARMABI int stack_printf(const char * __restrict /*format*/, ...) __attribute__((__nonnull__(1)));
#define stack_printf  uart_cb
void uart_stack_register(UART_DEBUG_CALLBACK_T cb);


/// @} UART
#endif /* _BIM_UART_H_ */

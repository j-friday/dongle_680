/**
 ****************************************************************************************
 *
 * @file uart.h
 *
 * @brief UART Driver for HCI over UART operation.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef _UART_H_
#define _UART_H_

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

#include "user_config.h"	  ////
/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */
//#define TOTAL0_BUF_NUM  256//2048///1024///256
//#define TOTAL_BUF_NUM  256///640///256


//#define BLE_RX_FIFO_MAX_COUNT  TOTAL0_BUF_NUM

//#define UART0_RX_FIFO_MAX_COUNT  TOTAL0_BUF_NUM
//#define UART0_TX_FIFO_MAX_COUNT  TOTAL0_BUF_NUM

//#define UART1_RX_FIFO_MAX_COUNT  TOTAL_BUF_NUM
//#define UART1_TX_FIFO_MAX_COUNT  TOTAL_BUF_NUM
/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initializes the UART to default values.
 *****************************************************************************************
 */
uint8_t uart_init(uint32_t baud);

#ifndef CFG_ROM
/**
 ****************************************************************************************
 * @brief Enable UART flow.
 *****************************************************************************************
 */
void uart_flow_on(void);

/**
 ****************************************************************************************
 * @brief Disable UART flow.
 *****************************************************************************************
 */
bool uart_flow_off(void);
#endif //CFG_ROM

/**
 ****************************************************************************************
 * @brief Finish current UART transfers
 *****************************************************************************************
 */
void uart_finish_transfers(void);

/**
 ****************************************************************************************
 * @brief Starts a data reception.
 *
 * @param[out] bufptr   Pointer to the RX buffer
 * @param[in]  size     Size of the expected reception
 * @param[in]  callback Pointer to the function called back when transfer finished
 * @param[in]  dummy    Dummy data pointer returned to callback when reception is finished
 *****************************************************************************************
 */
void uart_read(uint8_t *bufptr, uint32_t size, void (*callback) (void*, uint8_t), void* dummy);

/**
 ****************************************************************************************
 * @brief Starts a data transmission.
 *
 * @param[in] bufptr   Pointer to the TX buffer
 * @param[in] size     Size of the transmission
 * @param[in] callback Pointer to the function called back when transfer finished
 * @param[in] dummy    Dummy data pointer returned to callback when transmission is finished
 *****************************************************************************************
 */
void uart_write(uint8_t *bufptr, uint32_t size, void (*callback) (void*, uint8_t), void* dummy);

#if defined(CFG_ROM)
/**
 ****************************************************************************************
 * @brief Poll UART on reception and transmission.
 *
 * This function is used to poll UART for reception and transmission.
 * It is used when IRQ are not used to detect incoming bytes.
 *****************************************************************************************
 */
void uart_poll(void);
#endif //CFG_ROM

/**
 ****************************************************************************************
 * @brief Serves the data transfer interrupt requests.
 *
 * It clears the requests and executes the appropriate callback function.
 *****************************************************************************************
 */
void uart_isr(void);

#if (VIRTUAL_UART_H4TL == 1)
	void poki_hci_test_cmd(void);
	void set_app_role(uint8_t rol);
	void host_get_event_cbReg(void (*callback) (void*, uint16_t));
	uint8_t uart_init(uint32_t baud);
	void host_send_cmd(uint8_t *bufptr, uint16_t length);
	void uart_h4tl_data_switch(void);
#endif

void UartDbgInit(uint32_t baud_rate);
void uart_send(void *buff, uint16_t len);
void uart1_send(void *buff, uint16_t len);
extern int DbgPrintf(const char *fmt,...);
int Dbg1Printf(const char *fmt,...);
uint8_t store_uart_ringbuf_data(uint8_t *buf,uint16_t len);
void uart_ringbuf_clean(void);
uint8_t  read_uart_ringbuf_data(uint8_t *buf,uint16_t len);
void TRAhcit_UART_Rx(void);
/// @} UART
#endif /* _UART_H_ */

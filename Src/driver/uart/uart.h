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

#ifndef __UART_H__
#define __UART_H__

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



#if USER_USE_UART_1
#define UART_PRINTF  uart_printf_null//Dbg1Printf//////uart_printf //uart_printf 
#define USER_PRINTF  DbgPrintf //Dbg1Printf//uart_printf_null//////
#else
//#define UART_PRINTF  uart_printf_null//Dbg1Printf/// //uart_printf 
#define USER_PRINTF  Dbg1Printf //uart_printf_null
#endif

#define TX_FIFO_LENGTH 15
#define UART_SEND_LENGTH 128

#define UART_WRITE_BYTE(v)               (addUART0_Reg0x3=v)
#define UART2_WRITE_BYTE(v)               (addUART1_Reg0x3=v)

#if USER_USE_UART_1
#define TOTAL0_BUF_NUM      1024///256///1024///
#define TOTAL_BUF_NUM      256///1024///640///768///2048///640///256

#define BLE_RX_FIFO_MAX_COUNT  TOTAL0_BUF_NUM

#define USER_UART_RX_BUFFER      uart_rx_buf
#define USER_UART_TX_BUFFER      uart_tx_buf
#define USER_UART_RX_BUF_SIZE    TOTAL0_BUF_NUM
#define USER_UART_TX_BUF_SIZE    TOTAL0_BUF_NUM
#define USER_UART_RX_DONE        uart_rx_done
#define USER_UART_RX_BUFF_TAILOR uart_rx_buff_tailor
#define USER_UART_RX_INDEX       uart_rx_index

#define USER_UART_DATA_SEND      uart_data_send
#define USER_RING_BUFF_CLEAN     uart_ringbuf_clean
#else
#define TOTAL0_BUF_NUM  256///1024///
#define TOTAL_BUF_NUM   1024///640///768///2048///640///256

#define BLE_RX_FIFO_MAX_COUNT  TOTAL_BUF_NUM

#define USER_UART_RX_BUFFER      uart1_rx_buf
#define USER_UART_TX_BUFFER      uart1_tx_buf
#define USER_UART_RX_BUF_SIZE    TOTAL_BUF_NUM
#define USER_UART_TX_BUF_SIZE    TOTAL_BUF_NUM
#define USER_UART_RX_DONE        uart1_rx_done
#define USER_UART_RX_BUFF_TAILOR uart1_rx_buff_tailor
#define USER_UART_RX_INDEX       uart1_rx_index

#define USER_UART_DATA_SEND      uart1_data_send
#define USER_RING_BUFF_CLEAN     uart1_ringbuf_clean
#endif

#if !USB_DRIVER  //master use uart1
#define AT_RX_BUF_SIZE           TOTAL0_BUF_NUM
#define AT_RX_BUFFER             uart_rx_buf
#define AT_RX_INDEX              uart_rx_index
#define UART_SEND_AT             uart_send
#define AT_RING_BUFF_CLEAN       uart_ringbuf_clean
#else            //slave use uart2
#define AT_RX_BUF_SIZE           TOTAL_BUF_NUM
#define AT_RX_BUFFER             uart1_rx_buf
#define AT_RX_INDEX              uart1_rx_index
#define UART_SEND_AT             uart1_send
#define AT_RING_BUFF_CLEAN       uart1_ringbuf_clean
#endif

extern uint8_t uart_rx_buf[TOTAL0_BUF_NUM];
extern uint8_t uart_tx_buf[TOTAL0_BUF_NUM];
extern volatile uint32_t uart_rx_index;
extern uint32_t uart_rx_buff_tailor;
extern volatile uint8_t  uart_rx_done;

extern uint8_t uart1_rx_buf[TOTAL_BUF_NUM];
extern uint8_t uart1_tx_buf[TOTAL_BUF_NUM];
extern uint32_t uart1_rx_buff_tailor;
extern volatile uint8_t  uart1_rx_done;
extern volatile uint32_t uart1_rx_index;
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
void Uart1DbgInit(uint32_t baud_rate);
void uart_send(void *buff, uint16_t len);
void uart1_send(void *buff, uint16_t len);

int DbgPrintf(const char *fmt,...);
extern int Dbg1Printf(const char *fmt,...);
void set_parity_mode(uint8_t mode);
uint8_t store_uart_ringbuf_data(uint8_t *buf,uint16_t len);

void uart_ringbuf_clean(void);
void uart1_ringbuf_clean(void);
uint8_t  read_uart_ringbuf_data(uint8_t *buf,uint16_t len);
void TRAhcit_UART_Rx(void);

void uart_data_send(uint8_t * buffer,   uint16_t len);

int uart_printf_null(const char *fmt,...);
/// @} UART
#endif /* _UART_H_ */

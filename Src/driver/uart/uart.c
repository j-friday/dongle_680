/**
 ****************************************************************************************
 *
 * @file uart.c
 *
 * @brief UART driver
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup UART
 * @{
 ****************************************************************************************
 */
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stddef.h>     // standard definition
#include "uart.h"       // uart definition
#include "reg_uart.h"   // uart register
#ifndef CFG_ROM
#include "rwip.h"       // SW interface
#if (PLF_NVDS)
#include "nvds.h"      // NVDS
#endif // (PLF_NVDS)
#endif // CFG_ROM

#include "dbg.h"
#include "gpio.h"
#include "user_config.h"
#include "BK3633_RegList.h"
#include "../header/BK3633_Config.h"


extern int DbgPrintf(const char *fmt,...);
extern int Dbg1Printf(const char *fmt,...);

/*
 * DEFINES
 *****************************************************************************************
 */

/// Max baudrate supported by this UART (in bps)
#define UART_BAUD_MAX      3500000
/// Min baudrate supported by this UART (in bps)
#define UART_BAUD_MIN      9600

/// Duration of 1 byte transfer over UART (10 bits) in us (for 921600 default baudrate)
#define UART_CHAR_DURATION        11

#if (VIRTUAL_UART_H4TL == 1)
	////#define UART_TXRX_BUF_SIZE		128
	#define HCI_DATA_BUF_SIZE		512			////256		////128
	#define HCI_DATA_TYPE_CMD		0x01
	#define HCI_DATA_TYPE_EVENT		0x02
#endif
/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */

///UART Character format
enum UART_CHARFORMAT
{
    UART_CHARFORMAT_8 = 0,
    UART_CHARFORMAT_7 = 1
};

///UART Stop bit
enum UART_STOPBITS
{
    UART_STOPBITS_1 = 0,
    UART_STOPBITS_2 = 1 /* Note: The number of stop bits is 1.5 if a character format
                                 with 5 bit is chosen*/
};

///UART Parity enable
enum UART_PARITY
{
    UART_PARITY_DISABLED = 0,
    UART_PARITY_ENABLED  = 1
};

///UART Parity type
enum UART_PARITYBIT
{
    UART_PARITYBIT_EVEN  = 0,
    UART_PARITYBIT_ODD   = 1,
    UART_PARITYBIT_SPACE = 2, // The parity bit is always 0.
    UART_PARITYBIT_MARK  = 3  // The parity bit is always 1.
};

///UART HW flow control
enum UART_HW_FLOW_CNTL
{
    UART_HW_FLOW_CNTL_DISABLED = 0,
    UART_HW_FLOW_CNTL_ENABLED = 1
};

///UART Input clock select
enum UART_INPUT_CLK_SEL
{
    UART_INPUT_CLK_SEL_0 = 0,
    UART_INPUT_CLK_SEL_1 = 1,
    UART_INPUT_CLK_SEL_2 = 2,
    UART_INPUT_CLK_SEL_3 = 3
};

///UART Interrupt enable/disable
enum UART_INT
{
    UART_INT_DISABLE = 0,
    UART_INT_ENABLE = 1
};

///UART Error detection
enum UART_ERROR_DETECT
{
    UART_ERROR_DETECT_DISABLED = 0,
    UART_ERROR_DETECT_ENABLED  = 1
};

/*
 * STRUCT DEFINITIONS
 *****************************************************************************************
 */
/* TX and RX channel class holding data used for asynchronous read and write data
 * transactions
 */
/// UART TX RX Channel
struct uart_txrxchannel
{
    /// call back function pointer
    void (*callback) (void*, uint8_t);
    /// Dummy data pointer returned to callback when operation is over.
    void* dummy;
};

/// UART environment structure
struct uart_env_tag
{
    /// tx channel
    struct uart_txrxchannel tx;
    /// rx channel
    struct uart_txrxchannel rx;
    /// error detect
    uint8_t errordetect;
    /// external wakeup
    bool ext_wakeup;
	//// Modified
	uint8_t *uart_tx_buf;
	uint8_t *uart_rx_buf;
	uint32_t uart_tx_length;
	uint32_t uart_rx_length;
	uint8_t uart_tx_enable;		////Maybe no need
	uint8_t uart_rx_enable;		////Maybe no need
};
#if (VIRTUAL_UART_H4TL == 1)

	//// HCI CMD, Event
	struct hci_cmd_event_data
	{
	    /// call back function pointer
	    void (*callback) (void*, uint16_t);
	    /// Dummy data pointer returned to callback when operation is over.
	    uint8_t data_buf[HCI_DATA_BUF_SIZE];
		uint32_t data_len;
	};
#endif

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
/// uart environment structure

	volatile static struct uart_env_tag uart_env;

#if (VIRTUAL_UART_H4TL == 1)
	volatile struct hci_cmd_event_data host_cmd_data;
	volatile struct hci_cmd_event_data host_event_data;
#endif
/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
#if (VIRTUAL_UART_H4TL == 1)
	uint8_t app_role = 'N';
	void set_app_role(uint8_t rol)
	{
		app_role = rol;
	}

void hci_data_init(uint8_t type)
{	////type: 0x01-Clear host cmd data; 0x02-Clear host event data
    //bk_printf("%s,type:%x\r\n",__func__,type);
    if(type & HCI_DATA_TYPE_CMD)
    {
        host_cmd_data.callback = NULL;
        memset((void *)&host_cmd_data.data_buf[0], 0, HCI_DATA_BUF_SIZE);
        host_cmd_data.data_len = 0;
    }
    if(type & HCI_DATA_TYPE_EVENT)
    {
    ////	host_event_data.callback = NULL;		////Will clear callback func
        memset((void *)&host_event_data.data_buf[0], 0, HCI_DATA_BUF_SIZE);
        host_event_data.data_len = 0;
    }
}

	void host_send_cmd(uint8_t *bufptr, uint16_t length)
	{
		uint16_t tmpCnt = 0;
		ASSERT_ERR(length >= HCI_DATA_BUF_SIZE)
		host_cmd_data.callback = NULL;		////Test Only
		memcpy((void *)&host_cmd_data.data_buf[0], bufptr, length);
		host_cmd_data.data_len = length;

		bk_printf("host_send_cmd(%C):", app_role);
		if(length >= 20)
			length = 20;
		////for(tmpCnt = 0; tmpCnt < host_cmd_data.data_len; tmpCnt++)
        bk_printf("0x");
		for(tmpCnt = 0; tmpCnt < length - 1; tmpCnt++)
		{
            //GPIO_DEBUG_TRIGER(0x4); 
			bk_printf("%02X:", host_cmd_data.data_buf[tmpCnt]);
		}
        bk_printf("%02X", host_cmd_data.data_buf[tmpCnt]);
		bk_printf("\r\n");
	}

	////void host_get_event(uint8 *bufptr, uint8 length)
void host_get_event(void)
{
    uint16_t tmpCnt = 0;
    uint8_t tmpLen = host_event_data.data_len;
    
    if(tmpLen >= 20)
        tmpLen = 20;
    
    bk_printf("0x");
    ////for(tmpCnt = 0; tmpCnt < host_event_data.data_len; tmpCnt++)
    for(tmpCnt = 0; tmpCnt < tmpLen - 1; tmpCnt++)
    {

        bk_printf("%02X:", host_event_data.data_buf[tmpCnt]);

    }bk_printf("%02X", host_event_data.data_buf[tmpCnt]);
    bk_printf("\r\n");
    if(host_event_data.callback != NULL)
    {
        host_event_data.callback((void *)host_event_data.data_buf, host_event_data.data_len);
    }
    hci_data_init(HCI_DATA_TYPE_EVENT);
}

void host_get_event_cbReg(void (*callback) (void*, uint16_t))
{
    host_event_data.callback = callback;
}



#endif

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

uint8_t uart_init(uint32_t baud)
{
    
  //   UartDbgInit(baud);
#if (VIRTUAL_UART_H4TL == 1)		////

	//// Initialize RX and TX transfer callbacks
	uart_env.rx.callback = NULL;
	uart_env.tx.callback = NULL;
	uart_env.uart_tx_buf = NULL;
	uart_env.uart_rx_buf = NULL;
	uart_env.uart_tx_length = 0;
	uart_env.uart_rx_length = 0;
	uart_env.uart_tx_enable = 0;
	uart_env.uart_rx_enable = 0;

	hci_data_init((HCI_DATA_TYPE_CMD | HCI_DATA_TYPE_EVENT));
    host_get_event_cbReg(uart_send);
#endif
    return 0;
}

void uart_flow_on(void)
{
#if 0		

#endif
}

bool uart_flow_off(void)
{
    bool flow_off = true;
#if 0		

#endif
    return flow_off;
}

void uart_finish_transfers(void)
{
#if 0		////
    uart_force_rts_setf(1);
    // Wait TX FIFO empty
    while(!uart_tx_fifo_empty_getf());
#endif
}

void uart_read(uint8_t *bufptr, uint32_t size, void (*callback) (void*, uint8_t), void* dummy)
{	////Register Read callback func, write buf ptr, data size
    //bk_printf("%s\r\n",__func__);
#if (VIRTUAL_UART_H4TL == 1)		////
    
	ASSERT_ERR(bufptr != NULL);
    ASSERT_ERR(size != 0);
    ASSERT_ERR(callback != NULL);
    uart_env.rx.callback = callback;
    uart_env.rx.dummy    = dummy;

	uart_env.uart_rx_buf = bufptr;	////	uart_rx_ptr_setf((uint32_t) bufptr);
    uart_env.uart_rx_length = size; ////	uart_rx_size_setf(size);
    uart_env.uart_rx_enable = 1;	////	uart_rx_start_setf(1);

#endif
}

void uart_write(uint8_t *bufptr, uint32_t size, void (*callback) (void*, uint8_t), void* dummy)
{	////Register Read callback func, write buf ptr, data size
    //bk_printf("%s\r\n",__func__);
#if (VIRTUAL_UART_H4TL == 1)		////
	// Sanity check
    ASSERT_ERR(bufptr != NULL);
    ASSERT_ERR(size != 0);
    ASSERT_ERR(callback != NULL);
    uart_env.tx.callback = callback;
    uart_env.tx.dummy    = dummy;

	uart_env.uart_tx_buf = bufptr;
	uart_env.uart_tx_length = size;
	uart_env.uart_tx_enable = 1;


#endif
}

#if (VIRTUAL_UART_H4TL == 1)
	void uart_h4tl_data_switch(void)
	{
        
		void (*callback) (void*, uint8_t) = NULL;
		void* data =NULL;
		uint16_t data_len = 0;
		//uint16_t tmpCnt = 0;
		while(uart_env.uart_tx_enable == 1)
		{
            //bk_printf("uart_h4tl_data_switch tx_enable\r\n");
			// Retrieve callback pointer
	        callback = uart_env.tx.callback;
	        data     = uart_env.tx.dummy;

			uart_env.uart_tx_enable = 0;
			memcpy((void *)&host_event_data.data_buf[data_len], uart_env.uart_tx_buf, uart_env.uart_tx_length);
			data_len += uart_env.uart_tx_length;
			host_event_data.data_len += uart_env.uart_tx_length;
	        if(callback != NULL)
	        {
	            // Clear callback pointer
	            uart_env.tx.callback = NULL;
	            uart_env.tx.dummy    = NULL;

	            // Call handler
	            callback(data, RWIP_EIF_STATUS_OK);
	        }
	        else
	        {
	            ASSERT_ERR(0);
	        }
		}
		if(host_event_data.data_len != 0)
		{	////New Event
			host_get_event();           
		}

		data_len = 0;
        
		if(host_cmd_data.data_len > 0)
		{
            //bk_printf("host_cmd_data.data_len:%d\r\n",host_cmd_data.data_len);
				
			while(uart_env.uart_rx_enable == 1)
			{
                //bk_printf("uart_h4tl_data_switch Rx_enable uart_rx_length:%d data_len:%d\r\n",uart_env.uart_rx_length,data_len);
				// Retrieve callback pointer
				callback = uart_env.rx.callback;
		        data     = uart_env.rx.dummy;

				uart_env.uart_rx_enable = 0;
                
                
			
				memcpy((void *)uart_env.uart_rx_buf, (void *)&host_cmd_data.data_buf[data_len], uart_env.uart_rx_length);
			
		
				data_len += uart_env.uart_rx_length;
		        if(callback != NULL)
		        {
		            // Clear callback pointer
		            uart_env.rx.callback = NULL;
		            uart_env.rx.dummy    = NULL;

		            // Call handler
		            callback(data, RWIP_EIF_STATUS_OK);
		        }
		        else
		        {
		            ASSERT_ERR(0);
		        }
				if(data_len >= host_cmd_data.data_len)
					break;
			}////while(uart_env.uart_rx_enable == 1)
			if(data_len != host_cmd_data.data_len)
			{
				//bk_printf("HCI_CMD_WriteErr\r\n");
			}
			////else
			{	////Clear HCI Cmd data
				hci_data_init(HCI_DATA_TYPE_CMD);
			}

		}////if(host_cmd_data.data_len > 0)

	}
#endif

extern    volatile uint32_t uart_rx_index;
extern uint8_t uart_rx_buf[TOTAL0_BUF_NUM];
extern uint8_t uart_tx_buf[TOTAL0_BUF_NUM];

void uart_isr(void)
{

    DBG_SWDIAG(ISR, UART, 1);
    bk_printf("uart_isr\r\n");
   // while (uart_isr_stat_get())
    {
        void (*callback) (void*, uint8_t) = NULL;
        {
         #if (VIRTUAL_UART_H4TL == 1)
            if((uart_rx_buf[0] == 0x01) && ((uart_rx_buf[3] + 4) == uart_rx_index))
            {
                bk_printf("cmd len:%d,uart_rx_index:%d,\r\n",uart_rx_buf[3],uart_rx_index);
                host_send_cmd(uart_rx_buf, uart_rx_index);
                uart_rx_index = 0;
            }           
         #endif  
            // Retrieve callback pointer
          //  callback = uart_env.rx.callback;
           // data     = uart_env.rx.dummy;

            if(callback != NULL)
            {
                // Clear callback pointer
            //    uart_env.rx.callback = NULL;
             //   uart_env.rx.dummy    = NULL;
                //bk_printf("rx.callback\r\n");
                // Call handler
               // callback(data, RWIP_EIF_STATUS_OK);
            }
            else
            {
              ;//  ASSERT_ERR(0);
            }
        } 
    }              
}


//reaord index
static volatile uint16_t pbuff_write = 0;

static  uint8_t ringbuf[AT_RX_BUF_SIZE] = {0};
//read index
static volatile uint16_t pbuff_read = 0;
uint32_t uart_rx_buff_tailor = 0;///uart_rx_buff_header=0,volatile 

/*******************************************************************************
 * Function: store_uart_ringbuf_data
 * Description: store data into loop buffer
 * Input: uint8_t*
 * Input: uint16_t
 * Output: void
 * Return: uint8_t
 * Others: void
*******************************************************************************/

uint8_t store_uart_ringbuf_data(uint8_t *buf,uint16_t len)
{
	uint16_t free_cnt;
	uint8_t status ;
	
	//Calculates the number of empty buffer in the circular queue (the 
	//data stored in this queue is encoded)
	if(pbuff_write >= pbuff_read)
	{
		free_cnt = (AT_RX_BUF_SIZE - pbuff_write + pbuff_read);
	}else
	{
		free_cnt = pbuff_read - pbuff_write;
	}
	//bk_printf("free cnt: %d,store len : %d\r\n", free_cnt,len);
	
	//If there are at least two empty buffer in the loop queue, the current 
	//encoded data will be stored in buffer. 
	if(free_cnt >= len) 
	{
		memcpy((uint8_t *)&ringbuf[pbuff_write],buf,len);
		
		if(0){
			bk_printf("buf %d:",AT_RX_INDEX);
			for(int i =0;i < len;i++)
			{
				bk_printf("%02x ",ringbuf[pbuff_write  + i]);
			}
			bk_printf("%\r\n");
		}
		//Update the buffer index of the data 
		//(in fact, the index is between 0-79)
		pbuff_write = ((pbuff_write + len )% AT_RX_BUF_SIZE);//TOTAL0_BUF_NUM);
		status = 1;
		//bk_printf("enough data write!! : pbuff_write : %d, pbuff_read :%d\r\n",pbuff_write,pbuff_read);	
	}
    else
	{
		bk_printf("no enough buff fill data %d,%d!!!\r\n",pbuff_write,pbuff_read); // for test show
		status = 0;
	}
		
	return status;
}


 /*******************************************************************************
 * Function: read_uart_ringbuf_data
 * Description: read data from loop buffer
 * Input: uint8_t*
 * Output: void
 * Return: uint8_t readed cnt
 * Others: void
*******************************************************************************/
uint8_t  read_uart_ringbuf_data(uint8_t *buf,uint16_t len)
{
	uint16_t unread_cnt;
	//Read 20 encode data from loop buffer to designated buffer 
	if(pbuff_write >= pbuff_read)
	{
		unread_cnt = pbuff_write - pbuff_read;		
		//bk_printf("read 0x%x\r\n",pbuff_read);	
	}else
	{
		unread_cnt = (AT_RX_BUF_SIZE - pbuff_read  + pbuff_write);
		//bk_printf("buff empty!!0x%x\r\n",pbuff_read);	
	} 
	//bk_printf("unread_cnt : %d,read len :%d\r\n", unread_cnt,len); 	
	if(unread_cnt >= len)
	{
        memcpy(buf,(uint8_t *)&ringbuf[pbuff_read],len);

        if(0){
            bk_printf("bufR: %d:",len);
            for(int i =0;i < len;i++)
            {
                bk_printf("%c ",buf[i]);
            }
            bk_printf("%\r\n");
        }
					//Update the buffer index of the data 
		//(in fact, the index is between 0-255)
        pbuff_read = ((pbuff_read + len ) % AT_RX_BUF_SIZE);
      //  bk_printf("enough data read!! pbuff_write : %d,pbuff_read :%d\r\n",pbuff_write,pbuff_read);	
        return len;
	}
    else
	{
       // bk_printf("buff no enough data read!!pbuff_write :%d,pbuff_read :%d\r\n",pbuff_write,pbuff_read);	
        return 0;
	}	
}

 /*******************************************************************************
 * Function: uart_ringbuf_clean
 * Description: read data from loop buffer
 * Input: uint8_t*
 * Output: void
 * Return: uint8_t readed cnt
 * Others: void
*******************************************************************************/
void uart_ringbuf_clean(void)
{
    pbuff_write = pbuff_read = 0;
    memset((void *)uart_rx_buf,0,TOTAL0_BUF_NUM);
    memset((uint8_t *)uart_tx_buf,0,TOTAL0_BUF_NUM);
    uart_rx_index = 0;
    uart_rx_buff_tailor = 0;
    //bk_printf("uart0_ringbuf_clean \r\n");
}

int uart_printf_null(const char *fmt,...)
{
	return 0;
}

void uart1_ringbuf_clean(void)
{
    pbuff_write = pbuff_read = 0;
    memset((void *)uart1_rx_buf,0,TOTAL_BUF_NUM);
    memset((uint8_t *)uart1_tx_buf,0,TOTAL_BUF_NUM);
    uart1_rx_index = 0;
    uart1_rx_buff_tailor = 0;
//    bk_printf("uart1_ringbuf_clean \r\n");
}    

/// @} UART

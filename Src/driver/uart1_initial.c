#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "../header/BK3633_RegList.h"
#include "../header/BK3633_Config.h"
#include "../header/BK_HCI_Protocol.h"

#include "uart.h"
#include "user_config.h"
#include "arch.h"
#include "ke_event.h"
#include "user_func.h"
#include "icu.h"

volatile uint8_t  uart1_rx_done = FALSE;
volatile uint32_t uart1_rx_index = 0;
uint32_t uart1_rx_buff_tailor = 0;
uint8_t uart1_rx_buf[TOTAL_BUF_NUM];
uint8_t uart1_tx_buf[TOTAL_BUF_NUM];

static uint8_t parity_en = 0;
static uint8_t parity_mode = 1;

void uart2_parity_mode(uint8_t mode)
{
    if(mode & 0x10)parity_en = 0x01;
    else parity_en = 0x00;
    
    if(mode & 0x01)parity_mode = 0x01;
    else parity_mode = 0x00;
}



//==============================UART_INITIAL===================

void Uart1DbgInit(uint32_t baud_rate)
{
    uint32_t    uart_clk_div;
    clrf_SYS_Reg0x3_uart1_pwd ;   //open periph
    uart_clk_div = (UART_CLK_FREQ*1000000)/baud_rate - 1;
    addAON_GPIO_Reg0xe = (0x01 << 4) | (0x01 << 5)|(0x01 << 6); //second_func pull up
    addAON_GPIO_Reg0xf = (0x01 << 4) | (0x01 << 5)|(0x01 << 6); //second_func pull up
    addUART1_Reg0x0 = (uart_clk_div << posUART1_Reg0x0_UART_CLK_DIVID) |
                      (0x0          << posUART1_Reg0x0_UART_STOP_LEN ) |
				  #if (NCV_SIM == 0)
                      (parity_mode  << posUART1_Reg0x0_UART_PAR_MODE ) |
                      (parity_en    << posUART1_Reg0x0_UART_PAR_EN   ) |
				  #else
					  (0x0          << posUART1_Reg0x0_UART_PAR_MODE ) |
                      (0x1          << posUART1_Reg0x0_UART_PAR_EN   ) |
				  #endif
                      (0x3          << posUART1_Reg0x0_UART_LEN      ) |
                      (0x0          << posUART1_Reg0x0_UART_IRDA     ) |
                      (0x1          << posUART1_Reg0x0_UART_RX_ENABLE) |
                      (0x1          << posUART1_Reg0x0_UART_TX_ENABLE) ;
    addUART1_Reg0x1 = 0x00004010;
    addUART1_Reg0x4 = 0x42;
    addUART1_Reg0x6 = 0x0;
    addUART1_Reg0x7 = 0x0;

    setf_SYS_Reg0x10_int_uart1_en; //enable uart_int irq

    uart1_rx_done = FALSE;
    uart1_rx_index = 0;

}




static void uart1_send_byte(unsigned char data)
{
    while (!get_UART1_Reg0x2_FIFO_WR_READY);
    addUART1_Reg0x3 = data;
}

void uart1_send(void *buff, uint16_t len)
{
    uint8_t *tmpbuf = (uint8_t *)buff;
    while (len--)
        uart1_send_byte(*tmpbuf++);
}


void clear_uart1_buffer(void)
{
    uart1_rx_index = 0;
    uart1_rx_done = FALSE;
    memset(uart1_rx_buf, 0, sizeof(uart1_rx_buf)); /**< Clear the RX buffer */
    memset(uart1_tx_buf, 0, sizeof(uart1_tx_buf)); /**< Clear the TX buffer */
}


//char DbgBuff[128];
int Dbg1Printf(const char *fmt,...)
{
    va_list ap;
    int n;
	char DbgBuff[128];
    va_start(ap, fmt);
    n=vsprintf(DbgBuff, fmt, ap);
    va_end(ap);

#if 1	
	uart1_send(DbgBuff, n);
#endif
    return n;
}


//==========================UART_INTERRUPT==========================

__RAM_CODE void UART1ISR(void)
{
    unsigned long uart_int_status;
    unsigned char uart_fifo_rdata;	
#if (!USB_DRIVER)
	int pos = 0, sum_len;
#endif
//    Dbg1Printf("UART1ISR\r\n");
    uart_int_status = addUART1_Reg0x5;
    if (uart_int_status & (bitUART1_Reg0x5_RX_FIFO_NEED_READ | bitUART1_Reg0x5_UART_RX_STOP_END))
    {
        
        while (get_UART1_Reg0x2_FIFO_RD_READY)
	    {
            uart_fifo_rdata = (uint8_t)addUART1_Reg0x3;
            
            //bk_printf("%02x ",uart_fifo_rdata);
            uart1_rx_buf[uart1_rx_index++] = uart_fifo_rdata;
            if (uart1_rx_index == TOTAL_BUF_NUM)
            {
                uart1_rx_index = 0;
            }
            #if CLI_CONSOLE && USB_DRIVER
			store_uart_ringbuf_data(&uart_fifo_rdata,1);
			#endif//
       }
        //bk_printf("\r\n");
        if (uart_int_status & bitUART1_Reg0x5_UART_RX_STOP_END)
        {
            uart1_rx_done = TRUE;    // 此处应该先判断是否满足RX STOP
            #if CLI_CONSOLE  && USB_DRIVER
            //bk_printf(" SET KE_EVENT_AOS_CLI\r\n");
            set_at_rsp_ch(0);
            ke_event_set(KE_EVENT_AOS_CLI);
            #endif//
            #if (!USB_DRIVER)
			sum_len = uart1_rx_index;
			do 
			{
				int tx_len = sum_len < 240 ? sum_len : 240;
				app_send_ble_data(dmo_channel, tx_len, &uart1_rx_buf[pos]);
				pos += tx_len;
				sum_len -= tx_len;
				Delay_us(10000);
			} while(sum_len > 0);
            clear_uart1_buffer();
            #endif
        }
    }
    
    addUART1_Reg0x5 = uart_int_status;
}



//============================================================================================


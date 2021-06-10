#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "../header/BK3633_RegList.h"
#include "../header/BK3633_Config.h"
#include "../header/BK_HCI_Protocol.h"
#include "ke_event.h"
#include "uart.h"
#include "user_config.h"
#include "app_init.h"
#include "app_fee0.h"
#include "user_func.h"

volatile uint8_t  uart_rx_done = FALSE;
volatile uint32_t uart_rx_index = 0;

uint8_t uart_rx_buf[TOTAL0_BUF_NUM];///UART_RX_FIFO_MAX_COUNT
uint8_t uart_tx_buf[TOTAL0_BUF_NUM];///UART_TX_FIFO_MAX_COUNT

DbgPrtBuf_t	*pDbgPrtBuf;
DbgPrtBuf_t DbgPrtBufInst;

HCI_COMMAND_PACKET *pHCIrxBuf = (HCI_COMMAND_PACKET *)(&uart_rx_buf[0]);
HCI_EVENT_PACKET   *pHCItxBuf = (HCI_EVENT_PACKET *)(&uart_tx_buf[0]);

static uint8_t parity_en = 0;
static uint8_t parity_mode = 1;

void set_parity_mode(uint8_t mode)
{
    if(mode & 0x10)parity_en = 0x01;
    else parity_en = 0x00;
    
    if(mode & 0x01)parity_mode = 0x01;
    else parity_mode = 0x00;
}



//==============================UART_INITIAL===================

void UartDbgInit(uint32_t baud_rate)
{
    uint32_t    uart_clk_div;
    clrf_SYS_Reg0x3_uart0_pwd ;   //open periph
    uart_clk_div = (UART_CLK_FREQ*1000000)/baud_rate - 1;
    addAON_GPIO_Reg0x0 = (0x01 << 4) | (0x01 << 5)|(0x01 << 6); //second_func pull up
    addAON_GPIO_Reg0x1 = (0x01 << 4) | (0x01 << 5)|(0x01 << 6); //second_func pull up
    addUART0_Reg0x0 = (uart_clk_div << posUART0_Reg0x0_UART_CLK_DIVID) |
                      (0x0          << posUART0_Reg0x0_UART_STOP_LEN ) |
				  #if (NCV_SIM == 0)
                      (parity_mode  << posUART0_Reg0x0_UART_PAR_MODE ) |
                      (parity_en    << posUART0_Reg0x0_UART_PAR_EN   ) |
				  #else
					  (0x0          << posUART0_Reg0x0_UART_PAR_MODE ) |
                      (0x1          << posUART0_Reg0x0_UART_PAR_EN   ) |
				  #endif
                      (0x3          << posUART0_Reg0x0_UART_LEN      ) |
                      (0x0          << posUART0_Reg0x0_UART_IRDA     ) |
                      (0x1          << posUART0_Reg0x0_UART_RX_ENABLE) |
                      (0x1          << posUART0_Reg0x0_UART_TX_ENABLE) ;
    addUART0_Reg0x1 = 0x00004010;
    addUART0_Reg0x4 = 0x42;
    addUART0_Reg0x6 = 0x0;
    addUART0_Reg0x7 = 0x0;

    setf_SYS_Reg0x10_int_uart0_en; //enable uart_int irq

    uart_rx_done = FALSE;
    uart_rx_index = 0;

    pDbgPrtBuf = &DbgPrtBufInst;
    pDbgPrtBuf->pos_sta = 0;
    pDbgPrtBuf->pos_end = 0;
    //return 0;
}



static void uart_send_byte(unsigned char data)
{
    while (!get_UART0_Reg0x2_FIFO_WR_READY);
    addUART0_Reg0x3 = data;
}



void uart_send(void *buff, uint16_t len)
{
    uint8_t *tmpbuf = (uint8_t *)buff;
    while (len--)
        uart_send_byte(*tmpbuf++);
}


void uart_buff_send(volatile uint8_t **buf, volatile uint32_t *length)
{   
    uint32_t count = *length;
    uint32_t fifo_length;
    do
    {
        fifo_length=posUART0_Reg0x2_TX_FIFO_COUNT&0xff;
    }while(fifo_length>(TX_FIFO_LENGTH/2));    
    /* Wait until the UART transmitter is empty */      

    fifo_length=TX_FIFO_LENGTH-fifo_length;//need tune again
    if(count > fifo_length)
        count = fifo_length;       

    (*length)-= count; 
     do
    {
        while(!get_UART0_Reg0x2_FIFO_WR_READY);
        UART_WRITE_BYTE(*((*buf)++));
        count--;            
    }
    while(count);
     
}

void uart_data_send(uint8_t * buffer,   uint16_t len)
{
    uint32_t length2 = len;   
    uint8_t* buffer2 = &buffer[0];
    #if UART_PRF_DEBUG//UART_PRINTF_EN 
    uart_printf("UARTSEND: ");
    for(uint8_t i = 0;i<len;i++)
    {
    uart_printf("%02X ",buffer[i]);
    }
    uart_printf(" \r\n");
    #else
    while(length2)
    {
        uart_buff_send((volatile uint8_t**)&buffer2, (uint32_t*)&length2);
    }
    #endif
    
}

void clear_uart_buffer(void)
{
    uart_rx_index = 0;
    uart_rx_done = FALSE;
    memset(uart_rx_buf, 0, sizeof(uart_rx_buf)); /**< Clear the RX buffer */
    memset(uart_tx_buf, 0, sizeof(uart_tx_buf)); /**< Clear the TX buffer */
    bk_printf("clear_uart_buffer\r\n");
}

void TRAhcit_UART_Tx(void)
{
    uint32_t tx_len  = HCI_EVENT_HEAD_LENGTH+pHCItxBuf->total;
    pHCItxBuf->code  = TRA_HCIT_EVENT;
    pHCItxBuf->event = HCI_COMMAND_COMPLETE_EVENT;
    uart_send(uart_tx_buf, tx_len);
    
    clear_uart_buffer();
    bk_printf("1 clear\r\n");
}

int DbgPrintf(const char *fmt,...)
{
  //  return 0;
    va_list ap;
    int n;	
    va_start(ap, fmt);
    n = vsprintf(pDbgPrtBuf->buf, fmt, ap);
    va_end(ap);
    
//    if(n > 128)
//    {
//        uart_send("uart err\r\n", 20);
//    }

#if 1	////
	uart_send(pDbgPrtBuf->buf, n);
#else
    for(i = 0; i < n; i++)
	{
		pDbgPrtBuf->buf[pDbgPrtBuf->pos_end++] = DbgBuff[i];
		if(pDbgPrtBuf->pos_end >= 1024) //@@//(pDbgPrtBuf->pos_end >= 2048)
            pDbgPrtBuf->pos_end = 0;
	}
	setf_UART0_Reg0x4_TX_FIFO_NEED_WRITE_MASK;
#endif
    return n;
}



int Dbg_stackPrintf(const char *fmt,...)
{	
    int n =0;
    return n;
}


#include "reg_access.h"
#if 0
extern volatile uint32_t XVR_ANALOG_REG_BAK[32];
extern uint32_t flash_mid;
void TRAhcit_UART_Rx(void)
{
#if 0
	////Test
	static uint8 tmpcnt = 0;
	if(tmpcnt == 0)
	{
		tmpcnt = 8;
	////	RW_BT_master_test();
		func_test(&tmpcnt );
	}
#endif
    if ((uart_rx_done != TRUE) || (uart_rx_index == 0))
        return;

    if (   (pHCIrxBuf->code       != TRA_HCIT_COMMAND)
        || (pHCIrxBuf->opcode.ogf != VENDOR_SPECIFIC_DEBUG_OGF)
        || (pHCIrxBuf->opcode.ocf != BEKEN_OCF)
        || (uart_rx_index         != (HCI_COMMAND_HEAD_LENGTH+pHCIrxBuf->total))
       )
        goto ret;

    bk_printf("cmd=%x\r\n",pHCIrxBuf->cmd); 
    switch (pHCIrxBuf->cmd) {

    case LINK_CHECK_CMD:
        pHCItxBuf->total = uart_rx_index;
        memcpy(pHCItxBuf->param, uart_rx_buf, pHCItxBuf->total);
        break;

    case REGISTER_WRITE_CMD: {   // 01e0fc09010028800068000000
        signed   long reg_index;
        REGISTER_PARAM *rx_param        = (REGISTER_PARAM *)pHCIrxBuf->param;
        REGISTER_PARAM *tx_param        = (REGISTER_PARAM *)&pHCItxBuf->param[HCI_COMMAND_HEAD_LENGTH];
        pHCItxBuf->total                = uart_rx_index-1;
        memcpy(pHCItxBuf->param, uart_rx_buf, 3);
        pHCItxBuf->param[3]             = pHCIrxBuf->cmd;
        tx_param->addr                  = rx_param->addr;
        tx_param->value                 = rx_param->value;
        *(volatile uint32_t *)rx_param->addr = rx_param->value;
        reg_index                       = (rx_param->addr-BASEADDR_XVR)/4;
        if ((reg_index>=0) && (reg_index<=0x0f))
        {
            XVR_ANALOG_REG_BAK[reg_index] = rx_param->value;
        }else if ((reg_index>=0x1c) && (reg_index<=0x1f))
        {
            XVR_ANALOG_REG_BAK[reg_index] = rx_param->value;
        }
        
        bk_printf("reg_index =%d,rx_addr:0x%x,value:0x%x\r\n",reg_index,rx_param->addr,rx_param->value); 
        break;
    }

    case REGISTER_READ_CMD: {   // 01e0fc050300288000
        signed   long reg_index;
        REGISTER_PARAM *rx_param = (REGISTER_PARAM *)pHCIrxBuf->param;
        REGISTER_PARAM *tx_param = (REGISTER_PARAM *)&pHCItxBuf->param[HCI_COMMAND_HEAD_LENGTH];
        pHCItxBuf->total         = uart_rx_index+3;
        memcpy(pHCItxBuf->param, uart_rx_buf, 3);
        pHCItxBuf->param[3]      = pHCIrxBuf->cmd;
        tx_param->addr           = rx_param->addr;
        reg_index                = (rx_param->addr-BASEADDR_XVR)/4;
        if ((reg_index>=0) && (reg_index<=0x0f))
        {
            tx_param->value        = XVR_ANALOG_REG_BAK[reg_index];
            bk_printf("reg_index 0\r\n"); 
        }
        else if ((reg_index>=0x1c) && (reg_index<=0x1f))
        {
            tx_param->value        = XVR_ANALOG_REG_BAK[reg_index];
            bk_printf("reg_index 1\r\n"); 
        }
        else
        {
            tx_param->value          = *(volatile uint32_t *)rx_param->addr;
            bk_printf("reg_index 2,tx_param->value:%x\r\n",REG_BLE_RD(rx_param->addr)); 
        }
        
        bk_printf("reg_index =%d,rx_addr:0x%x\r\n",reg_index,rx_param->addr); 
        break;
    }
    
    case CHIP_RST_CMD: {// 01e0fc05FE 95 27 95 27
    REGISTER_PARAM *rx_param = (REGISTER_PARAM *)pHCIrxBuf->param;
        
    bk_printf("addr:%x\r\n",rx_param->addr); 
        
        
    if(rx_param->addr == 0x27952795)
    {
        setf_FLASH_Reg0x7_fwren_flash_cpu;
        bk_printf("%s\r\n",__func__);
//        *(volatile uint32_t *)0x100 = 0x0;
//        *(volatile uint32_t *)0x104 = 0x0;
//        *(volatile uint32_t *)0x108 = 0x0;
//        *(volatile uint32_t *)0x10c = 0x0;
//        *(volatile uint32_t *)0x110 = 0x0;
     //   flash_erase(flash_mid, 0x7E000, 0x1000, NULL);
        set_AON_WDT_Reg0x2_aon_wdt_period(0x10);//close WDT
		set_AON_WDT_Reg0x1_aon_wdt_feed(0x5a);
        set_AON_WDT_Reg0x1_aon_wdt_feed(0xa5);
    }
        
    }break;

///========== BK3633 TEST Module ========
     case MODULE_TEST_CMD : {       //01 E0 FC + 0x3(total_len) +  0xCC + 0x00 (module_cmd)
       // func_test(pHCIrxBuf->param);
        goto ret;
      }
//========== BK3633 TEST Module ========
     default:
        goto ret;
    }

    TRAhcit_UART_Tx();

ret:
    clear_uart_buffer();
    bk_printf("2 clear\r\n");
}
#endif

//==========================UART_INTERRUPT==========================
__RAM_CODE void UART0ISR(void)
{
    unsigned long uart_int_status;
    unsigned char uart_fifo_rdata;
//    uint8_t		    uart_txcnt = 0;

    uart_int_status = addUART0_Reg0x5;
    if (uart_int_status & (bitUART0_Reg0x5_RX_FIFO_NEED_READ | bitUART0_Reg0x5_UART_RX_STOP_END))
    {
        
        while (get_UART0_Reg0x2_FIFO_RD_READY)
	    {
            uart_fifo_rdata = (uint8_t)addUART0_Reg0x3;
            
            uart_rx_buf[uart_rx_index++] = uart_fifo_rdata;
            if (uart_rx_index == TOTAL0_BUF_NUM)
            {
                uart_rx_index = 0;
            }
            #if CLI_CONSOLE && !USB_DRIVER
			store_uart_ringbuf_data(&uart_fifo_rdata,1);
			#endif//
       }
      
       // bk_printf("\r\n");
        if (uart_int_status & bitUART0_Reg0x5_UART_RX_STOP_END)
        {
            uart_rx_done = TRUE;    // 此处应该先判断是否满足RX STOP
            #if CLI_CONSOLE  && !USB_DRIVER
            //bk_printf(" SET KE_EVENT_AOS_CLI\r\n");
            set_at_rsp_ch(0);
            ke_event_set(KE_EVENT_AOS_CLI);
            #endif
        }        
    }
    addUART0_Reg0x5 = uart_int_status;
}


//============================================================================================
















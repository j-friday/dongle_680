#include "../header/BK3633_Config.h"

#include "RWBTDM_RegList.h"
extern	void	SysInit(void) ;
extern	void	UartDbgInit(uint32_t baud_rate) ;
extern	void	Uart1DbgInit(uint32_t baud_rate) ;
extern	void	p_function_initial(void);
extern	void    TRAhcit_UART_Rx(void);

extern	void	rw_main(void) ;

extern  void    btdm_initial(void);

#include "co_math.h"        // math library definition
#include "user_config.h"	////

uint8_t RW_BT_TargetBTAddr[6] = {0};

extern void RW_BT_master_test(void);
extern void RW_BT_slave_test(void);

#include "uart.h"
#include "string.h"
#include "icu.h"
int main(void)
{

	SysInit();
    set_FLASH_Reg0x7_flash_clk_conf(8);
    icu_init();
    #if (!USB_DRIVER)
	UartDbgInit(115200);
    #endif
    Uart1DbgInit(115200);

    bk_printf("BK3633 Initial Finish!\r\n");    
	bk_printf("@@%s %s!!!\r\n",__DATE__, __TIME__ );
   
    bk_printf("CHIP ID:%x\r\n",get_XVR_Reg0x10_chip_id);
    bk_printf("reset_reason:%x\r\n",get_PMU_Reg0x0_reset_reason);
    
    
	while(1) {

//		TRAhcit_UART_Rx();	
        rw_main();		

	}
}

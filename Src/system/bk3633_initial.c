#include "../header/BK3633_RegList.h"
#include "../header/BK3633_Config.h"
#include "../header/BK_HCI_Protocol.h"
#include "string.h"


extern	HCI_COMMAND_PACKET *pHCIrxBuf  ;


void cpu_delay( volatile unsigned int times)
{
    while(times--) ;
}

void feed_watchdog(void)
{
    set_AON_WDT_Reg0x1_aon_wdt_feed(0x5a);
    set_AON_WDT_Reg0x1_aon_wdt_feed(0xa5);
}




void	SysInit()	{
		set_AON_WDT_Reg0x2_aon_wdt_period(0x0);//close WDT
		feed_watchdog();
		clrf_SYS_Reg0x0_jtag_mode;//close jtag
    
        clrf_PMU_Reg0x1_boot_rom_en;
        set_SYS_Reg0x3_rwbt_pwd(0);

}


//=============================================================
//=============================================================



//==============================================================
//==============================================================


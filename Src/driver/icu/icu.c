/**
****************************************************************************************
*
* @file icu.c
*
* @brief icu initialization and specific functions
*
* Copyright (C) Beken 2009-2016
*
* $Rev: $
*
****************************************************************************************
*/

/**
****************************************************************************************
* @addtogroup ICU
* @ingroup ICU
* @brief ICU
*
* This is the driver block for ICU
* @{
****************************************************************************************
*/


#include <stddef.h>     // standard definition
#include "rwip_config.h"
#include "rwip.h"
#include "../header/BK3633_RegList.h"
#include "../header/BK3633_Config.h"
#include "reg_ipcore.h"
#include "icu.h"      // timer definition
#include "rf.h"
#include "gpio.h"
//#include "flash.h"
//#include "uart.h"


static volatile uint8_t reduce_voltage_set=0;
static uint8_t default_sleep_mode = 0;  //0:Ωµ—π–›√ﬂ   1:∆’Õ®idel

uint8_t system_clk;
uint8_t system_sleep_flag;
uint8_t system_mode;

uint8_t system_sleep_status = 0;
extern uint32_t flash_mid;

void system_sleep_init(void)
{

    system_sleep_flag = RW_MCU_IDLE_SLEEP;

    if((system_mode & RW_DUT_MODE) == RW_DUT_MODE)
    {
		system_sleep_flag = RW_NO_SLEEP;
    }
}


void icu_init(void)
{
    clrf_SYS_Reg0x0_jtag_mode;   ///close JTAG
    //mcu_clk_switch(MCU_DEFAULT_CLK);
    set_SYS_Reg0x2_core_sel(0x01);
    set_SYS_Reg0x2_core_div(0x0);
    setf_SYS_Reg0xb_pwd_on_boostsel; 
    addSYS_Reg0x17 = 0x80;
    setf_SYS_Reg0xd_PLL_PWR_sel;
    addPMU_Reg0x14=0x6666;
}


uint8_t icu_get_sleep_mode(void)
{
	return default_sleep_mode;
}


void icu_set_sleep_mode(uint8_t v)
{
	default_sleep_mode = v;
}


void cpu_reduce_voltage_sleep(uint8_t level)
{


    uint32_t tmp_reg;
    uint32_t core_sel0,core_sel1 ;
    uint32_t work_voltage0,work_voltage1;
//    GPIO_DEBUG_TRIGER(0x4);
    set_SYS_Reg0x1_gotosleep(1);
    set_SYS_Reg0x1_gotosleep(0);

//    set_PMU_Reg0x14_voltage_ctrl_work_aon(0x06);
//    set_PMU_Reg0x14_voltage_ctrl_work_core(0x06);
  //   bk_printf("rv:%x\r\n",addPMU_Reg0x14);

//    set_PMU_Reg0x14_voltage_ctrl_sleep_aon(03);
//    
//   set_PMU_Reg0x14_voltage_ctrl_sleep_core(level);
       
//   setf_PMU_Reg0x14_sleep_sel; // don't set if use reduce viltage sleep
           
    set_SYS_Reg0x2_core_sel(0x01);
    set_SYS_Reg0x2_core_div(0x0);
    
    setf_SYS_Reg0x17_enb_busrt_sel;
    
    setf_SYS_Reg0x17_CLK96M_PWD;
    setf_SYS_Reg0x17_HP_LDO_PWD;
    setf_SYS_Reg0x17_cb_bias_pwd;


    tmp_reg = addSYS_Reg0x17 | 0x08;
    system_sleep_status = 1;
    
//    GPIO_DEBUG_TRIGER(0x4);
//    GPIO_DEBUG_TRIGER(0x4);
    
    
    core_sel0 = ((addSYS_Reg0x2 & (~0x180)) | ((0x00) << 7));
    core_sel1 = ((addSYS_Reg0x2 & (~0x180)) | ((0x01) << 7)); 
    work_voltage0 = ((addPMU_Reg0x14 & (~0xF000)) | ((0x03) << 12));
    work_voltage0 = ((work_voltage0 & (~0xF0)) | ((0x03) << 4));
    
    work_voltage1 = ((addPMU_Reg0x14 & (~0xF000)) | ((0x06) << 12));
    work_voltage1 = ((work_voltage1 & (~0xF0)) | ((0x06) << 4));

#if 0
    set_SYS_Reg0x2_core_sel(0x00);
    set_PMU_Reg0x14_voltage_ctrl_work_aon(0x03);
    set_PMU_Reg0x14_voltage_ctrl_work_core(0x03);
#else   
    addSYS_Reg0x2 = core_sel0;
    addPMU_Reg0x14 = work_voltage0;      
#endif    
    addSYS_Reg0x17 = tmp_reg;
    setf_SYS_Reg0x1_CPU_PWD;  
    
    addSYS_Reg0x17 = 0x82;
    
#if 0   
    set_PMU_Reg0x14_voltage_ctrl_work_aon(0x06);
    set_PMU_Reg0x14_voltage_ctrl_work_core(0x06);
    set_SYS_Reg0x2_core_sel(0x01);
#else     
    
    addPMU_Reg0x14 = work_voltage1;  
    addSYS_Reg0x2 = core_sel1;
#endif         

}
void cpu_wakeup(void)
{
  //  uint8_t calc_num = 0;    
    if(system_sleep_status == 1)
    {
        addSYS_Reg0x17 = 0x82;
        switch(system_clk)
        {
            case MCU_CLK_16M:
            {

                set_SYS_Reg0x2_core_sel(0x01);
                set_SYS_Reg0x2_core_div(0x1);
              
            }break;
        
            case MCU_CLK_32M:
            {
                
                set_SYS_Reg0x2_core_div(0x2);            
                set_SYS_Reg0x2_core_sel(0x03);
                                               
            }break;
        
            case MCU_CLK_48M:
            {

                set_SYS_Reg0x2_core_div(0x2);         
                set_SYS_Reg0x2_core_sel(0x03);
            
            }break;
        
            case MCU_CLK_64M:
            {
                set_SYS_Reg0x2_core_div(0x1);            
                set_SYS_Reg0x2_core_sel(0x03);
            
            }break;
        
            case MCU_CLK_80M:
            {


                set_SYS_Reg0x2_core_div(0x1);            
                set_SYS_Reg0x2_core_sel(0x03);
            
            }break;
        
            default:break; 
        }
       
        system_sleep_status = 0;
       // bk_printf("cpu_wakeup:%x\r\n",addSYS_Reg0x17);        
    }
	   
}

void cpu_idle_sleep(void)
{
//    GPIO_DEBUG_TRIGER(0x5);
   
     if(system_sleep_status == 0)
     {
    //    GPIO_DEBUG_TRIGER(0x5);
        clrf_PMU_Reg0x14_sleep_sel;
       // set_SYS_Reg0x1_gotosleep(1);
      //  set_SYS_Reg0x1_gotosleep(0);    
        setf_SYS_Reg0x1_CPU_PWD;  
     }
  //      
}

extern void CLK32K_AutoCali_init(void);
extern volatile uint32_t XVR_ANALOG_REG_BAK[32];

void mcu_clk_switch(uint8_t clk)
{
    system_clk = clk;
    switch(clk)
    {
        case MCU_CLK_16M:
        {
            set_SYS_Reg0x2_core_div(0x01);
            set_SYS_Reg0x2_core_sel(0x01);            
          
        }break;
        
        case MCU_CLK_32M:
        {
            
            XVR_ANALOG_REG_BAK[9]|= (0x01 << 20);
            XVR_ANALOG_REG_BAK[9]&= ~(0x01 << 17);
            XVR_ANALOG_REG_BAK[9]|= (0x01 << 16);
            addXVR_Reg0x9 = XVR_ANALOG_REG_BAK[9];
            clrf_SYS_Reg0x17_CLK96M_PWD;
            setf_SYS_Reg0xd_PLL_PWR_sel;
            set_SYS_Reg0x2_core_div(0x2);            
            set_SYS_Reg0x2_core_sel(0x03);
                                           
        }break;
        
        case MCU_CLK_48M:
        {
            XVR_ANALOG_REG_BAK[9]|= (0x01 << 20);
            XVR_ANALOG_REG_BAK[9]|= (0x01 << 17);
            XVR_ANALOG_REG_BAK[9]|= (0x01 << 16);
            addXVR_Reg0x9 = XVR_ANALOG_REG_BAK[9];
            clrf_SYS_Reg0x17_CLK96M_PWD;
            setf_SYS_Reg0xd_PLL_PWR_sel;
            set_SYS_Reg0x2_core_div(0x2);         
            set_SYS_Reg0x2_core_sel(0x03);
        
        }break;
        
        case MCU_CLK_64M:
        {
            XVR_ANALOG_REG_BAK[9]|= (0x01 << 20);
            XVR_ANALOG_REG_BAK[9]&= ~(0x01 << 17);
            XVR_ANALOG_REG_BAK[9]|= (0x01 << 16);
            addXVR_Reg0x9 = XVR_ANALOG_REG_BAK[9];
            clrf_SYS_Reg0x17_CLK96M_PWD;
            setf_SYS_Reg0xd_PLL_PWR_sel;
            set_SYS_Reg0x2_core_div(0x1);            
            set_SYS_Reg0x2_core_sel(0x03);
        
        }break;
        
        case MCU_CLK_80M:
        {
            XVR_ANALOG_REG_BAK[9]&= ~(0x01 << 20);
            XVR_ANALOG_REG_BAK[9]&= ~(0x01 << 17);
            XVR_ANALOG_REG_BAK[9]&= ~(0x01 << 16);
            addXVR_Reg0x9 = XVR_ANALOG_REG_BAK[9];
            clrf_SYS_Reg0x17_CLK96M_PWD;
            setf_SYS_Reg0xd_PLL_PWR_sel;
            set_SYS_Reg0x2_core_div(0x1);            
            set_SYS_Reg0x2_core_sel(0x03);
        
        }break;
        
        default:break;
    }
    


}

uint8_t rw_ip_sleep_test_flag = 0;


uint8_t get_rw_ip_sleep_test_flag(void)
{
   return rw_ip_sleep_test_flag;
}


void rw_ip_deep_sleep_test(void)
{   
    
    Delay_ms(3000);
  //  xvr_reg_initial();
   // setf_SYS_Reg0x17_enb_busrt_reg;
 //   setf_SYS_Reg0x17_enb_busrt_sel;
    addXVR_Reg0x9 = 0x7093220C  ;XVR_ANALOG_REG_BAK[9] = 0x7093220C;
    set_PMU_Reg0x4_gotosleep(0x3633);
    Delay_ms(3000);
    while(1);
}



void Delay_us(int num)
{
    int x, y,z;
    
     switch(system_clk)
     {
        case MCU_CLK_16M:
        {
            z = 2;
        }break;
            
         case MCU_CLK_32M:
        {
            z = 5;
        }break;
        case MCU_CLK_48M:
        {
            z = 9;
        }break;
         case MCU_CLK_64M:
        {
            z = 11;
        }break;
        case MCU_CLK_80M:
        {
            z = 13;
        }break;
        
        default:
        {
            z =  3;
        }
        break;
     }
    y = num;
    while(y--)
    {
        x = z;
        while(x--);
    }
}

void Delay_ms(int num)
{
    int x, y,z;
     switch(system_clk)
     {
        case MCU_CLK_16M:
        {
            z = 4000;
        }break;
            
         case MCU_CLK_32M:
        {
            z = 8000;
        }break;
        case MCU_CLK_48M:
        {
            z = 12000;
        }break;
         case MCU_CLK_64M:
        {
            z = 3260;
        }break;
        case MCU_CLK_80M:
        {
            z = 3260;
        }break;
        
        default:
        {
            z = 3260;
        }
        break;
     }
    y = num;
    while(y--)
    {
        x = z;
        while(x--);
    }
}


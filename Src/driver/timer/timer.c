/**
****************************************************************************************
*
* @file timer.c
*
* @brief timer initialization and specific functions
*
* Copyright (C) Beken 2019-2022
*
* $Rev: $
*
****************************************************************************************
*/

/**
****************************************************************************************
* @addtogroup TIME
* @ingroup TIME
* @brief TIME
*
* This is the driver block for TIME
* @{
****************************************************************************************
*/


#include <stddef.h>     // standard definition
#include "rwip_config.h"
#include "rwip.h"
#include "../header/BK3633_RegList.h"
#include "../header/BK3633_Config.h"
#include "timer.h"
#include "reg_ipcore.h"
#include "icu.h"      // timer definition
#include "gpio.h"
#include "uart.h"

void (*p_TIM0_Int_Handler[TIMER_NUMBER_MAX])(unsigned char ucTIM_channel);
void (*p_TIM1_Int_Handler[TIMER_NUMBER_MAX])(unsigned char ucTIM_channel);

TMR_DRV_DESC tmr0_env ,tmr1_env;
uint8_t test_cnt = 0;
void timer0_0_isr_handler(unsigned char ucChannel)
{
    bk_printf("timer0_0_isr_handler:%d,test_cnt:%d\r\n",ucChannel,test_cnt);
   
    test_cnt++;
    if(test_cnt == 3)
    {
        timer0_set(1,0,5000);
    }
    if(test_cnt == 10)
    {
        timer0_set(1,1,2000);
    }
    
    
}

void timer0_1_isr_handler(unsigned char ucChannel)
{
    bk_printf("timer0_1_isr_handler:%d\r\n",ucChannel);

}
void timer0_2_isr_handler(unsigned char ucChannel)
{
    bk_printf("timer0_2_isr_handler:%d\r\n",ucChannel);
    

}

void timer0_init_test(void)
{
    timer0_init(0,1,1,1000,timer0_0_isr_handler);
    timer0_init(1,4,1,2000,timer0_1_isr_handler);
}
__RAM_CODE static void time0_env_init(void)
{
    static uint8_t init_flag = 0;
    if(init_flag == 0)
    {
        memset(&tmr0_env,sizeof(TMR_DRV_DESC),0);
        init_flag = 1;
    }
}
__RAM_CODE void timer0_init(uint8_t ch, uint8_t clk_div,uint8_t restart,uint32_t timer,void (*p_Int_Handler)(unsigned char ucChannel))//timer uints 1ms
{
   // bk_printf("timer0_init[%d],%d \r\n",ch,timer);
    uint32_t timer_val;
    
   // timer_val = timer * 16000;
    
    timer_val = timer * 8; // units 0.5 us
    if (ch >= TIMER_NUMBER_MAX)
    {
        return;
    }
    if (clk_div > 0Xf)
    {
        return;
    }
    time0_env_init();
       
    clrf_SYS_Reg0x3_tim0_pwd;
    set_SYS_Reg0x4_tim0_sel(1);
    if(!(tmr0_env.init_en & 0x07))
    {
        set_TIMER0_Reg0x3_clk_div(clk_div); 
        tmr0_env.clk_div = clk_div;
    }
    timer_val =  timer_val /(tmr0_env.clk_div + 1); 
      
    tmr0_env.init_en |=(0x01 << ch);   
    tmr0_env.restart[ch] = restart;
    tmr0_env.p_Int_Handler[ch] = p_Int_Handler;
    tmr0_env.timer_set_val[ch] = timer_val;
    tmr0_env.init_en |= (0x01 << (ch + 3));
    switch(ch)
    {
        case 0:
        {
            addTIMER0_Reg0x0 = timer_val;
            setf_TIMER0_Reg0x3_timer0_en;
            
        }break;
        case 1:
        {
             addTIMER0_Reg0x1 = timer_val;
            setf_TIMER0_Reg0x3_timer1_en;
        }break;
        case 2:
        {
            addTIMER0_Reg0x2 = timer_val;
            setf_TIMER0_Reg0x3_timer2_en;
        }break;
        
        default:
            break;
    }
    setf_SYS_Reg0x10_int_timer0_en;  
}

void timer0_deinit(uint8_t ch)//timer uints 1ms
{
    bk_printf("timer0_deinit[%d] \r\n",ch);
   
    if (ch >= TIMER_NUMBER_MAX)
    {
        return;
    }
    switch(ch)
    {
        case 0:
        {          
            clrf_TIMER0_Reg0x3_timer0_en;           
        }break;
        case 1:
        {      
            clrf_TIMER0_Reg0x3_timer1_en;
        }break;
        case 2:
        {
          
            clrf_TIMER0_Reg0x3_timer2_en;
        }break;
        
        default:
            break;
    }  
    tmr0_env.init_en &= ~(0x01 << ch);
    tmr0_env.init_en &= ~(0x01 << (ch + 3));
    if(!(tmr0_env.init_en & 0x07))
    {
        setf_SYS_Reg0x3_tim0_pwd;
        clrf_SYS_Reg0x10_int_timer0_en;  
    }
    tmr0_env.p_Int_Handler[ch] = NULL;  
    bk_printf("tmr0_env.init_en:%x\r\n",tmr0_env.init_en);
}


void timer0_set(uint8_t ch,uint8_t restart,uint32_t timer)
{
    bk_printf("timer0_set[%d],%d \r\n",ch,timer);
    if (ch >= TIMER_NUMBER_MAX)
    {
        return;
    }
    if(!(tmr0_env.init_en & (0x01 << ch)))
    {
        bk_printf("tmr0_[%d] not init\r\n",ch);
        return;
    }
    uint32_t timer_val;
 //   timer_val = timer * 16000;
    timer_val = timer * 32;
    timer_val =  timer_val /(tmr0_env.clk_div + 1);  
    bk_printf("timer_val:%d\r\n",timer_val);
    tmr0_env.restart[ch] = restart;
     switch(ch)
    {
        case 0:
        {   
            clrf_TIMER0_Reg0x3_timer0_en; 
            addTIMER0_Reg0x0 = timer_val; 
            setf_TIMER0_Reg0x3_timer0_en;            
        }break;
        case 1:
        {      
            clrf_TIMER0_Reg0x3_timer1_en;
            addTIMER0_Reg0x1 = timer_val;
            setf_TIMER0_Reg0x3_timer1_en;
        }break;
        case 2:
        {
            clrf_TIMER0_Reg0x3_timer2_en;
            addTIMER0_Reg0x2 = timer_val;
            setf_TIMER0_Reg0x3_timer2_en;
        }break;
        
        default:
            break;
    }  

}

void timer0_en(uint8_t ch)
{
    addTIMER0_Reg0x3 &= (0x01 << ch);
}

void timer0_disen(uint8_t ch)
{
    addTIMER0_Reg0x3 &= (0x01 << ch);
}

void timer0_isr(void)
{
    int i;
    unsigned long ulIntStatus;

    ulIntStatus = (addTIMER0_Reg0x3 >> 7)& 0x7;;
  //  bk_printf("ulIntStatus:0x%x\r\n",ulIntStatus);
    for (i=0; i<TIMER_NUMBER_MAX; i++)
    {
        if (ulIntStatus & (0x01<<i))
        {
            if (!tmr0_env.restart[i])
            {
                addTIMER0_Reg0x3 &= ~(0x01 << i);
                //bk_printf("addTIMER0_Reg0x3:0x%x\r\n",addTIMER0_Reg0x3);
            }
            if (tmr0_env.p_Int_Handler[i] != NULL)
            {
                (void)tmr0_env.p_Int_Handler[i]((unsigned char)i);
            }
        }
    }    
    do
    {
        addTIMER0_Reg0x3 |= (ulIntStatus >> 7);
    } while ((addTIMER0_Reg0x3 >> 7) & ulIntStatus & 0x7);   // delays
}


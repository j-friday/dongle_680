/**
 ****************************************************************************************
 *
 * @file intc.c
 *
 * @brief Definition of the Interrupt Controller (INTCTRL) API.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup INTC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if defined(CFG_BT) || defined(CFG_BLE)
#include "rwip_config.h"
#endif// defined(CFG_BT) || defined(CFG_BLE)
#include "compiler.h"        // for inline functions
#include "arch.h"            // arch definition
#include "co_math.h"         // math library definition

#if defined(CFG_BT)
#include "rwip_config.h"     // stack configuration
#include "rwbt.h"            // rwbt core
#endif //CFG_BT

#if defined(CFG_BLE)
#if (BLE_EMB_PRESENT)
#include "rwble.h"           // rwble core
#endif // (BLE_EMB_PRESENT)
#endif //CFG_BLE

#if defined(CFG_BT) && defined(CFG_BLE)
#include "rwip.h"            // rw IP core driver
#endif // #if defined(CFG_BT) && defined(CFG_BLE)

#include "intc.h"            // interrupt controller
#include "reg_intc.h"        // intc registers
#if PLF_UART
#include "uart.h"            // uart definitions
#endif //PLF_UART

#ifdef CFG_CPU_BK3633
#include "../header/BK3633_Config.h"
#endif
#if(ADC_DRIVER)
#include "adc.h"
#endif
#if (USB_DRIVER)
#include "driver_usb.h"
#endif
#include "BK3633_Reglist.h"		////
#include "icu.h"
#include "gpio.h"
#include "pwm.h"
#include "timer.h"
#if(BEKEN_EVENT_2_4G)
//#include "driver_rf24.h"
#endif
/*
 * DEFINES
 ****************************************************************************************
 */

#define RWBT_INT      CO_BIT(INTC_BT)
#define RWBTDM_INT    CO_BIT(INTC_BTDM)
#define UART_INT      CO_BIT(INTC_UART)
#define UART_2_INT    CO_BIT(INTC_UART_2)

#define RWBLE_INT     CO_BIT(INTC_BLE)
#define RWCOMMON_INT  CO_BIT(INTC_COMMON)
#define TIMER_INT     CO_BIT(INTC_TIMER)


// enable the supported interrupts
#define PLF_INT     (UART_INT | UART_2_INT | TIMER_INT)

#if defined(CFG_BT)
#define BT_INT      (RWBT_INT)
#else
#define BT_INT       0
#endif // #if defined(CFG_BT)

#if defined(CFG_BLE)
#if (BLE_EMB_PRESENT)
#define BLE_INT     (RWBLE_INT)
#else
#define BLE_INT      0
#endif // (BLE_EMB_PRESENT)
#else
#define BLE_INT      0
#endif // #if defined(CFG_BLE)

#if defined(CFG_BT) && defined(CFG_BLE)
#define BTDM_INT    (RWBTDM_INT)
#else
#define BTDM_INT    0
#endif // #if defined(CFG_BT) && defined(CFG_BLE)


#define INT_FLAG_BITCHECK  // Beken platform used
//charles move from BK3633prj
#define        INT_FLAG_PWM0          (0x1 << 0  )
#define        INT_FLAG_PWM1          (0x1 << 1  )
#define        INT_FLAG_TIMER0        (0x1 << 2  )
#define        INT_FLAG_TIMER1        (0x1 << 3  )
#define        INT_FLAG_UART0         (0x1 << 4  )
#define        INT_FLAG_UART1         (0x1 << 5  )
#define        INT_FLAG_SPI           (0x1 << 6  )
#define        INT_FLAG_I2C           (0x1 << 7  )
#define        INT_FLAG_SADC          (0x1 << 8  )
#define        INT_FLAG_GPIO          (0x1 << 9  )
#define        INT_FLAG_RTC           (0x1 << 10 )
#define        INT_FLAG_I2S           (0x1 << 11 )
#define        INT_FLAG_AON_RTC       (0x1 << 12 )
#define        INT_FLAG_USB           (0x1 << 17 )
#define        INT_FLAG_DMA           (0x1 << 18 )
#define        INT_FLAG_BK24          (0x1 << 19 )
#define        INT_FLAG_RWBLE         (0x1 << 20 )
#define        INT_FLAG_RWBT          (0x1 << 21 )
#define        INT_FLAG_RWDM          (0x1 << 22 )
//charles end


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
/// locally define this type to be able to qualify the array.

/*
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */
void intc_spurious(void);
__RAM_CODE void UART0ISR(void);
__RAM_CODE void UART1ISR(void);
/*
 * CONSTANT DATA DEFINITIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void intc_spurious(void)
{
    // force error
    ASSERT_ERR(0);
}

void intc_init(void)
{
    bk_printf("SYS_Reg0x10:%x,SYS_Reg0x11:%x\r\n",addSYS_Reg0x10,addSYS_Reg0x11);
    addSYS_Reg0x10 = 0; // int enable 0:disable 1::enable
    addSYS_Reg0x11 = 0; // priority; 0: irq  1:fiq
    #if (!USB_DRIVER)
    setf_SYS_Reg0x10_int_uart0_en; //enable uart_int irq
    //setf_SYS_Reg0x10_int_adc_en;
    #endif
    setf_SYS_Reg0x10_int_timer0_en; //enable timer_int irq
    setf_SYS_Reg0x10_int_uart1_en;//enable uart1_int irq
    //clrf_SYS_Reg0x10_int_timer0_en;
    setf_SYS_Reg0x10_int_rwble_en; //enable int rwble
    setf_SYS_Reg0x10_int_rwdm_en; //enable int rwdm
    //setf_SYS_Reg0x10_int_bk24_en;//enable int bk24
    setf_SYS_Reg0x10_int_rwbt_en; //enable  rwbt
    
    clrf_SYS_Reg0x11_int_rwbt_pri; //0 :irq
    //clrf_SYS_Reg0x11_int_rwble_pri; //0 :irq
    setf_SYS_Reg0x11_int_rwble_pri; // 1:fiq
    setf_SYS_Reg0x11_int_rwdm_pri; // 1:fiq
    setf_SYS_Reg0x11_int_rwbt_pri; // 1:fiq
    
    setf_SYS_Reg0x11_int_timer0_pri; // 1:fiq
}

void intc_stat_clear(void)
{
#if 0	////
    // Clear all interrupts
    intc_fiqenableclear_set(INTC_FIQACK_MASK);
    intc_irqenableclear_set(INTC_IRQACK_MASK);
#endif
}

__RAM_CODE void intc_irq(void)
{

    uint32_t IntStat;
	uint32_t irq_status = 0;
//    cpu_wakeup();
	IntStat = intc_status_get();
//    bk_printf("irq IntStat:%x\r\n",IntStat);
    //GPIO_DEBUG_TRIGER(0x7);
    if(IntStat & INT_STATUS_RWDM_bit)
	{
		irq_status |= INT_STATUS_RWDM_bit;
        rwip_isr();
	}
    #if (BT_DUAL_MODE || BLE_STD_MODE)
    if(IntStat & INT_STATUS_RWBLE_bit)
	{
		irq_status |= INT_STATUS_RWBLE_bit;
		rwble_isr();
	}
    #endif //(BT_DUAL_MODE || BLE_STD_MODE) 
    #if (BT_DUAL_MODE || BT_STD_MODE) 
	if(IntStat & INT_STATUS_RWBT_bit)
	{
		irq_status |= INT_STATUS_RWBT_bit;
        rwbt_isr();
	} 
    #endif //(BT_DUAL_MODE || BT_STD_MODE)
    
   	// call the function handler
    if(IntStat & INT_STATUS_PWM0_bit)
	{
		irq_status |= INT_STATUS_PWM0_bit;
		pwm0_isr();
	}
#if (ADC_DRIVER)
    if(IntStat & INT_STATUS_ADC_bit)
    {
        irq_status |= INT_STATUS_ADC_bit;
        adc_isr();
    }
#endif   
    if(IntStat & INT_STATUS_PWM1_bit)
	{
		irq_status |= INT_STATUS_PWM0_bit;
		pwm1_isr();
	}
    
	if(IntStat & INT_STATUS_UART0_bit)
	{
		irq_status |= INT_STATUS_UART0_bit;
		UART0ISR();
	}
    if(IntStat & INT_STATUS_UART1_bit)
	{
		irq_status |= INT_STATUS_UART1_bit;
		UART1ISR();
	}
	if(IntStat & INT_STATUS_AON_GPIO_bit)
	{
		irq_status |= INT_STATUS_AON_GPIO_bit;
		gpio_isr();
	}
    
    if(IntStat & INT_STATUS_TMR0_bit)
	{
		irq_status |= INT_STATUS_TMR0_bit;
		timer0_isr();
	}
#if(USB_DRIVER)  
    if(IntStat & INT_STATUS_USB_bit)
    {
    	irq_status |= INT_STATUS_USB_bit;
    	USB_InterruptHandler();
    }
#endif
	intc_status_clear(irq_status);
    addSYS_Reg0x12 |= IntStat ; //clr flag

}

void intc_fiq(void)
{
    uint32_t IntStat;
	uint32_t fiq_status;
    
//    cpu_wakeup();

    //GPIO_DEBUG_TRIGER(0x3);
	IntStat = intc_status_get();
  //  bk_printf("fiq:%x\r\n",IntStat);
    if(IntStat & INT_STATUS_TMR0_bit)
	{
		fiq_status |= INT_STATUS_TMR0_bit;
		timer0_isr();
	}
    if(IntStat & INT_STATUS_RWDM_bit)
	{
		fiq_status |= INT_STATUS_RWDM_bit;
        rwip_isr();
	}

    #if (BT_DUAL_MODE || BLE_STD_MODE)
    if(IntStat & INT_STATUS_RWBLE_bit)
	{
		fiq_status |= INT_STATUS_RWBLE_bit;
		rwble_isr();
	}
    #endif //(BT_DUAL_MODE || BLE_STD_MODE) 
    #if (BT_DUAL_MODE || BT_STD_MODE) 
	if(IntStat & INT_STATUS_RWBT_bit)
	{
		fiq_status |= INT_STATUS_RWBT_bit;
        rwbt_isr();
	} 
    #endif //(BT_DUAL_MODE || BT_STD_MODE)
    
    if(IntStat & INT_STATUS_AON_GPIO_bit)
	{
		//fiq_status |= INT_STATUS_AON_GPIO_bit;
		//gpio_isr();
	}
	intc_status_clear(fiq_status);

	addSYS_Reg0x12 |= IntStat ; //clr flag

}

#pragma ARM//不要修改以下两个文件
__IRQ FAST_IRQ_ENTRY void SYSirq_IRQ_Handler(void)
{
    intc_irq();		
}

__FIQ FAST_FIQ_ENTRY void SYSirq_FIQ_Handler(void)
{
    intc_fiq();	 
}

/// @} INTC

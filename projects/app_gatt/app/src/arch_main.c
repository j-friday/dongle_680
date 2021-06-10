/**
 ****************************************************************************************
 *
 * @file arch_main.c
 *
 * @brief Main loop of the application.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */


/*
 * INCLUDES
 ****************************************************************************************
 */

#include "rwip_config.h" // RW SW configuration
#include "rwip_int.h"
#include "arch.h"      // architectural platform definitions
#include <stdlib.h>    // standard lib functions
#include <stddef.h>    // standard definitions
#include <stdint.h>    // standard integer definition
#include <stdbool.h>   // boolean definition
#include "boot.h"      // boot definition
#include "rwip.h"      // RW SW initialization
#include "syscntl.h"   // System control initialization
#include "emi.h"       // EMI initialization
#include "intc.h"      // Interrupt initialization
#if PLF_UART
  #include "uart.h"      // UART initialization
 #if PLF_UART2
  #include "uart2.h"      // UART2 initialization
 #endif // PLF_UART2
 
#endif //PLF_UART
#include "debug_uart.h"
#include "flash.h"     // Flash initialization
#include "led.h"       // Led initialization
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
  #include "rf.h"        // RF initialization
#endif // BLE_EMB_PRESENT || BT_EMB_PRESENT

#if (BLE_APP_PRESENT)
  #include "app.h"       // application functions
#endif // BLE_APP_PRESENT

#if (PLF_NVDS)
  #include "nvds.h"       // NVDS definitions 
#endif // PLF_NVDS

#include "reg_blecore.h"      // BLE Core registers
void CpuGotoIdle(void);
#if (BLE_HOST_PRESENT)
#include "rwble_hl.h"        // BLE HL definitions
#include "gapc.h"
#include "gapm.h"
#include "gattc.h"
#include "l2cc.h"
#endif //BLE_HOST_PRESENT
#include "dbg.h"
#if (BLE_OADS_SERVER)
#include "oads.h"
#endif
#include "icu.h"
#include "user_config.h"
#include "gpio.h"
#include "cli.h"
#if(USB_DRIVER)
#include "driver_usb.h"
#endif
#if(ADC_DRIVER)
#include "adc.h"
#endif
#include "user_func.h"
/**
 ****************************************************************************************
 * @addtogroup DRIVERS
 * @{
 *
 *
 * ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

extern void uart_stack_register(UART_DEBUG_CALLBACK_T cb);
/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

/// Description of unloaded RAM area content
struct unloaded_area_tag
{
    // status error
    uint32_t error;
};


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */


#if (PLF_DEBUG)
/// Variable to enable infinite loop on assert
volatile int dbg_assert_block = 1;
#endif //PLF_DEBUG

/// Pointer to access unloaded RAM area
struct unloaded_area_tag* unloaded_area;

/// Variable storing the reason of platform reset
uint32_t error = RESET_NO_ERROR;

uint32_t critical_sec_cnt = 0;
/*
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize unloaded RAM area
 *
 * The unloaded RAM area is a part of RAM data memory that is not loaded at platform boot.
 * Information written in this area is maintained until device power-off.
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if (PLF_DEBUG)
void assert_err(const char *condition, const char * file, int line)
{
    TRC_REQ_SW_ASS_ERR(file, line, 0, 0);

    // Trigger assert message
    rwip_assert(file, line, 0, 0, ASSERT_TYPE_ERROR);

    // Let time for the message transfer
    for(int i = 0; i<2000;i++){dbg_assert_block = 1;};

    asrt_line_set(line);
    asrt_addr_setf((uint32_t)file);
    asrt_trigg_setf(1);

    GLOBAL_INT_STOP();

    while(dbg_assert_block);
}

void assert_param(int param0, int param1, const char * file, int line)
{

    // Trigger assert message
    rwip_assert(file, line, param0, param1, ASSERT_TYPE_ERROR);

}

void assert_warn(int param0, int param1, const char * file, int line)
{
  
    // Trigger assert message
    rwip_assert(file, line, param0, param1, ASSERT_TYPE_WARNING);


}

void dump_data(uint8_t* data, uint16_t length)
{

}
#endif //PLF_DEBUG

uint16_t get_stack_usage(uint8_t stack_type)
{
#if 1 //@@
    switch(stack_type)
    {
        case 0:
        {
            uint8_t *ptr = (uint8_t*)(STACK_BASE_UNUSED);

            while(*ptr++ == BOOT_PATTERN_UNUSED);
            
            ptr--;
            bk_printf("STACK_BASE_UNUSED :0x%x\r\n",STACK_BASE_UNUSED);
            bk_printf("STACK_LEN_UNUSED :0x%x\r\n",STACK_LEN_UNUSED);
            bk_printf("(uint32_t)ptr :0x%x\r\n",(uint32_t)ptr);

            return (uint16_t)((uint32_t)STACK_BASE_UNUSED + (uint32_t)STACK_LEN_UNUSED - (uint32_t)ptr);
            
        }//break;
        
        case 1:
        {
            uint8_t *ptr = (uint8_t*)(STACK_BASE_SVC);

            while(*ptr++ == BOOT_PATTERN_SVC);
            ptr--;
            return (uint16_t)((uint32_t)STACK_BASE_SVC + (uint32_t)STACK_LEN_SVC - (uint32_t)ptr);
            
        }//break;
        
        case 2:
        {
            uint8_t *ptr = (uint8_t*)(STACK_BASE_IRQ);

            while(*ptr++ == BOOT_PATTERN_IRQ);
            ptr--;
            return (uint16_t)((uint32_t)STACK_BASE_IRQ + (uint32_t)STACK_LEN_IRQ - (uint32_t)ptr);
            
        }//break;
        
        case 3:
        {
            uint8_t *ptr = (uint8_t*)(STACK_BASE_FIQ);

            while(*ptr++ == BOOT_PATTERN_FIQ);
            ptr--;
            return (uint16_t)((uint32_t)STACK_BASE_FIQ + (uint32_t)STACK_LEN_FIQ - (uint32_t)ptr);
            
        }//break;
        
        default:
        {
            return 0;
        }
        //break;
    
    }
   
#else
    return 0;
#endif
}

void bdaddr_env_init(void)
{
	struct bd_addr co_bdaddr;
	flash_read_data(&co_bdaddr.addr[0],0x7E000,6);
	if(co_bdaddr.addr[0]!=0xff ||co_bdaddr.addr[1]!=0xff||
	        co_bdaddr.addr[2]!=0xff||co_bdaddr.addr[3]!=0xff||
	        co_bdaddr.addr[4]!=0xff||co_bdaddr.addr[5]!=0xff )
	{
		//memcpy(&co_default_bdaddr, &co_bdaddr, 6);        
        set_ble_mac((uint8_t *)&co_bdaddr);
	}    
}

void platform_reset(uint32_t error)
{
    void (*pReset)(void);

    // Disable interrupts
    GLOBAL_INT_STOP();

    #if PLF_UART
    // Wait UART transfer finished
    uart_finish_transfers();
    #if !(BLE_EMB_PRESENT) && !(BT_EMB_PRESENT)
    uart2_finish_transfers();
    #endif // !BLE_EMB_PRESENT && !(BT_EMB_PRESENT)
    #endif //PLF_UART

    // Store information in unloaded area
    unloaded_area->error = error;
    
    
    bk_printf("platform_reset error:0x%x\r\n",error);

    if(error == RESET_AND_LOAD_FW || error == RESET_TO_ROM)
    {
        // Not yet supported
    }
    else
    {
        // Restart FW
        pReset = (void * )(0x0);
        (*pReset)();
    }
    
    while(1);
}

static void rom_env_init(struct rom_env_tag *api)
{
    memset(&rom_env,0,sizeof(struct rom_env_tag));
    rom_env.prf_get_id_from_task = prf_get_id_from_task;
    rom_env.prf_get_task_from_id = prf_get_task_from_id;
    rom_env.prf_init = prf_init;
    rom_env.prf_create = prf_create;
    rom_env.prf_cleanup = prf_cleanup;
    rom_env.prf_add_profile = prf_add_profile;

    rom_env.rwip_reset = rwip_reset;
    rom_env.platform_reset = platform_reset;
    rom_env.appm_init = appm_init;
    rom_env.rf_init = rf_init;
    rom_env.rwip_driver_init = rwip_driver_init;

}

/*
 * MAIN FUNCTION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief RW main function.
 *
 * This function is called right after the booting process has completed.
 *
 * @return status   exit status
 ****************************************************************************************
 */


/// Heap definitions - use uint32 to ensure that memory blocks are 32bits aligned.
/// Memory allocated for environment variables
uint32_t rwip_ke_heap_env[RWIP_CALC_HEAP_LEN(RWIP_HEAP_ENV_SIZE)];

#if (BLE_HOST_PRESENT)
/// Memory allocated for Attribute database
uint32_t rwip_ke_heap_db[RWIP_CALC_HEAP_LEN(RWIP_HEAP_DB_SIZE)];

#endif // (BLE_HOST_PRESENT)
/// Memory allocated for kernel messages
uint32_t rwip_ke_heap_msg[RWIP_CALC_HEAP_LEN(RWIP_HEAP_MSG_SIZE)];

/// Non Retention memory block
uint32_t rwip_ke_heap_non_ret[RWIP_CALC_HEAP_LEN(RWIP_HEAP_NON_RET_SIZE)];

void rw_main(void)
{
    /*
     ************************************************************************************
     * Platform initialization
     ************************************************************************************
     */
    bk_printf("rw_main start\r\n");
    
    xvr_reg_initial();

    uart_stack_register((UART_DEBUG_CALLBACK_T)bk_printf);

    bk_printf("xvr_reg_init ok\r\n");

    // Initialize random process
   // srand(1);
    // Initialize the exchange memory interface
    // Initialize timer module
//    timer_init();
    // Initialize the Interrupt Controller
    intc_init();
    gpio_init();
    // Initialize Flash component
    flash_init();
    
    user_init();//

    struct rwip_param_api param;
    param.get = nvds_get;
    param.set = nvds_put;
    param.del = nvds_del;
    rwip_param_init(param);
    #if (PLF_NVDS)
    // Initialize NVDS module
    nvds_init();
    #endif // PLF_NVDS

    rom_env_init(&rom_env);
    
    bdaddr_env_init();
    // Initialize UART component
    #if PLF_UART
    
    #if !(BLE_EMB_PRESENT) && !(BT_EMB_PRESENT)
    uart2_init();
    #endif // !BLE_EMB_PRESENT && !(BT_EMB_PRESENT)
    #endif //PLF_UART
   
	
    // Initialize System Control module

    //addAON_GPIO_Reg0x2 ^= 0x02 ; //trigger GPIO2 OUTPUT

    // Initialize Led module

    //addAON_GPIO_Reg0x2 ^= 0x02 ; //trigger GPIO2 OUTPUT
    /*
     ************************************************************************************
     * RW SW stack initialization
     ************************************************************************************
     */
    
    // Initialize RW SW stack
    rwip_ke_mem_init(rwip_ke_heap_env,rwip_ke_heap_db,rwip_ke_heap_msg,rwip_ke_heap_non_ret);
    
    bk_printf("rwip_ke_mem_init OK\r\n");
    struct rwip_eif_api api;
    api.flow_off = uart_flow_off;
    api.flow_on = uart_flow_on;
    api.read = uart_read;
    api.write =   uart_write;
    rwip_eif_api_init(api);

    rwip_init(0);
    bk_printf("rwip_init OK\r\n");
    mcu_clk_switch(MCU_CLK_16M);//(MCU_CLK_80M);
    
    aos_cli_init();
    
    #if (USB_DRIVER)
    usb_init(1);
    #else
    #if(ADC_DRIVER) 
    adc_init(1, 1);
    //adc_init(2, 1);
    #endif
    #endif
    // finally start interrupt handling
    GLOBAL_INT_START();
    #if(ADC_CALIB)
    calib_adc();
    #endif
    /*
     ************************************************************************************
     * Main loop
     ************************************************************************************
     */
    while(1)
    {
        // schedule all pending events
        rwip_schedule();
        user_task();   
        // Checks for sleep have to be done with interrupt disabled
        GLOBAL_INT_DISABLE();
        #if (BLE_OADS_SERVER)
        oad_updating_user_section_pro();
        #endif        
#if 1
        if(get_dev_mode())
        {
        // Check if the processor clock can be gated
        switch(rwip_sleep())
        {
            case RWIP_DEEP_SLEEP:
            {
                // add here platform specific deep sleep code                     
                cpu_reduce_voltage_sleep(0x00);
                cpu_wakeup();
            }
             break;
            // no break
            case RWIP_CPU_SLEEP:
            {
                // Wait for interrupt                
                WFI();                                
                cpu_idle_sleep();
            }
            break;
            case RWIP_ACTIVE:
            default:
            {
                // nothing to do.                
            } break;
        }
		}
#endif
        // Checks for sleep have to be done with interrupt disabled
        GLOBAL_INT_RESTORE();
        #if (VIRTUAL_UART_H4TL == 1)
            uart_h4tl_data_switch();
        #endif        
    }
}

/// @} DRIVERS

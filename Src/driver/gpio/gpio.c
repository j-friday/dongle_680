/**
****************************************************************************************
*
* @file gpio.c
*
* @brief icu initialization and specific functions
*
* Copyright (C) Beken 2019-2022
*
* $Rev: $
*
****************************************************************************************
*/

/**
****************************************************************************************
* @addtogroup GPIO
* @ingroup GPIO
* @brief GPIO
*
* This is the driver block for GPIO
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
#include "gpio.h"
#include "uart.h"


static GPIO_INT_CALLBACK_T gpio_int_cb = NULL; 

void gpio_init(void)
{
	//gpio_config(0x14,OUTPUT,PULL_HIGH);
   //gpio_config(0x17,INPUT,PULL_HIGH);
    gpio_config(0x04,OUTPUT,PULL_NONE);
    gpio_config(0x05,OUTPUT,PULL_NONE);
    gpio_config(0x06,OUTPUT,PULL_NONE);
    gpio_config(0x07,OUTPUT,PULL_NONE);
}
void gpio_config(uint8_t gpio, Dir_Type dir, Pull_Type pull)
{
    uint32_t  gpio_temp=0;
    uint8_t port = ((gpio&0xf0)>>4);
    uint8_t  pin = (gpio&0xf);

    switch(dir)
    {
        case OUTPUT:	       
            gpio_temp &= ~(1<<GPIO_INPUT_EN);
            gpio_temp &= ~(1<<GPIO_OUTPUT_EN);
            break;        
        case INPUT:
            gpio_temp |= (1<<GPIO_OUTPUT_EN);
            gpio_temp |= (1<<GPIO_INPUT_EN);
            break;
    	case FLOAT:		
    		gpio_temp &= ~(1<<GPIO_INPUT_EN);
            gpio_temp |= (1<<GPIO_OUTPUT_EN);
            break;
        case SC_FUN:		
    		gpio_temp |= (1<<GPIO_2FUN_EN);
            break;
    }

    switch(pull)
    {
    case PULL_HIGH:        
        gpio_temp |= (1<<GPIO_PULL_EN);
        gpio_temp |= (1<<GPIO_PULL_MODE);
        break;
    case PULL_LOW:
        gpio_temp |= (1<<GPIO_PULL_EN);
        gpio_temp &= ~(1<<GPIO_PULL_MODE);
        break;
    case PULL_NONE:
        gpio_temp &= ~(1<<GPIO_PULL_EN);
        break;
    }
    *((volatile unsigned long *) (BASEADDR_GPIO+4*(port*8+pin)))=gpio_temp;
    
}

uint8_t gpio_get_input(uint8_t gpio)
{
    uint32_t temp=0;
    uint8_t port = ((gpio&0xf0)>>4);
    uint8_t  pin = (gpio&0xf);

    temp = *((volatile unsigned long *) (BASEADDR_GPIO+4*(port*8+pin)));
	
    return (temp&(1<<GPIO_INPUT_VA));
}


void gpio_set(uint8_t gpio, uint8_t val)
{
    uint32_t temp=0;
    uint8_t port = ((gpio&0xf0)>>4);
    uint8_t  pin = (gpio&0xf);

    temp = *((volatile unsigned long *) (BASEADDR_GPIO+4*(port*8+pin)));
    if(val)
    {
        temp |= (1<<GPIO_OUTPUT_VA);
    }
    else
    {
        temp &= ~(1<<GPIO_OUTPUT_VA);
    }
    *((volatile unsigned long *) (BASEADDR_GPIO+4*(port*8+pin))) = temp;
    
}

void gpio_triger(uint8_t gpio)
{
	gpio_set(gpio, 1);
	gpio_set(gpio, 0);
}


void gpio_cb_register(GPIO_INT_CALLBACK_T cb)
{
	if(cb)
	{
		gpio_int_cb = cb;
	}
}

void gpio_test_init(void)
{
    
    set_AON_GPIO_Reg0x8_GPIO8_Config(0x3c);
    
    addAON_GPIO_Reg0x30 |= (0x3 << 16);
    
    addAON_GPIO_Reg0x33 |= (0x1 << 8);
    
  //  setf_PMU_Reg0x1_direct_wake_enable;
    
   // addPMU_Reg0x1 |= (0x1 << 12);
    setf_SYS_Reg0x10_int_aon_gpio_en;
    
		
	
}

__RAM_CODE void gpio_isr(void)
{
	uint32_t state = addAON_GPIO_Reg0x35;
    if(state != 0)
    {   
       // addAON_GPIO_Reg0x33 = 0;
        bk_printf("gpio_isr:%x\r\n",state);
       // addAON_GPIO_Reg0x33 = 0;
       // addAON_GPIO_Reg0x33 |= (0x1 << 8);
        addAON_GPIO_Reg0x35 = state;  
        if(gpio_int_cb)
        {
            (*gpio_int_cb)();
        }
        
    }
   
}





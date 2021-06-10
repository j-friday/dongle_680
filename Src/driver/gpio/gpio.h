/**
 ****************************************************************************************
 *
 * @file gpio.h
 *
 * @brief gpio Driver for gpio operation.
 *
 * Copyright (C) Beken 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef _GPIO_H_
#define _GPIO_H_



#include <stdbool.h>          // standard boolean definitions
#include <stdint.h>           // standard integer functions
#include "arch.h"
/**
 ****************************************************************************************
 * @defgroup GPIO 
 * @ingroup DRIVERS
 * @brief GPIO driver
 *
 * @{
 *
 ****************************************************************************************
 */

 
#define GPIOA_0     0x00
#define GPIOA_1     0x01
#define GPIOA_2     0x02
#define GPIOA_3     0x03
#define GPIOA_4     0x04
#define GPIOA_5     0x05
#define GPIOA_6     0x06
#define GPIOA_7     0x07

#define GPIOB_0     0x10
#define GPIOB_1     0x11
#define GPIOB_2     0x12
#define GPIOB_3     0x13
#define GPIOB_4     0x14
#define GPIOB_5     0x15
#define GPIOB_6     0x16
#define GPIOB_7     0x17

#define GPIOC_0     0x20
#define GPIOC_1     0x21
#define GPIOC_2     0x22
#define GPIOC_3     0x23
#define GPIOC_4     0x24
#define GPIOC_5     0x25
#define GPIOC_6     0x26
#define GPIOC_7     0x27

#define GPIOD_0     0x30
#define GPIOD_1     0x31
#define GPIOD_2     0x32
#define GPIOD_3     0x33
#define GPIOD_4     0x34
#define GPIOD_5     0x35
#define GPIOD_6     0x36
#define GPIOD_7     0x37

#define BASEADDR_GPIO                                           0x00800A00

#define GPIO_INPUT_VA   0
#define GPIO_OUTPUT_VA  1
#define GPIO_INPUT_EN   2
#define GPIO_OUTPUT_EN  3
#define GPIO_PULL_MODE  4
#define GPIO_PULL_EN    5
#define GPIO_2FUN_EN    6
 
 typedef enum
{
    INPUT,
    OUTPUT,
    FLOAT,
    SC_FUN,
}Dir_Type;

typedef enum
{
    PULL_HIGH,
    PULL_LOW,
    PULL_NONE
}Pull_Type;

typedef void (*GPIO_INT_CALLBACK_T)(void);   

void gpio_cb_register(GPIO_INT_CALLBACK_T cb);


void gpio_init(void);
void gpio_config(uint8_t gpio, Dir_Type dir, Pull_Type pull);
uint8_t gpio_get_input(uint8_t gpio);
void gpio_set(uint8_t gpio, uint8_t val);
void gpio_target(uint8_t gpio);
__RAM_CODE void gpio_isr(void);
void gpio_sleep(void);
void gpio_wakeup(void); 
void gpio_triger(uint8_t gpio);
void DEBUG_MSG(uint8_t x);

#define  GPIO_DEBUG  1

#if (GPIO_DEBUG)
#define  GPIO_DEBUG_TRIGER(GPIO)         \
    { \
        addAON_GPIO_Reg##GPIO = ((addAON_GPIO_Reg##GPIO & (~0x3FF)) | ((2) << 0));\
        addAON_GPIO_Reg##GPIO = ((addAON_GPIO_Reg##GPIO & (~0x3FF)) | ((0) << 0));\
    }

#else 
#define  GPIO_TRIGER(H_GPIO,GPIO)
#endif
 
#endif // _GPIO_H_

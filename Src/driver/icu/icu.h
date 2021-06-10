#ifndef _ICU_H_
#define _ICU_H_

#include "stdint.h"



enum sleep_type
{
	RW_NO_SLEEP = 0,
	RW_MCU_IDLE_SLEEP = (0x01 << 0), //cpu idle,clk run	
    RW_MCU_REDUCE_VO_SLEEP = (0x01 << 1), //cpu idle,16M,PLL STOP,reduce cpu voltage
    RW_MCU_DEEP_SLEEP = (0x01 << 2), //cpu idle,16M,PLL STOP
	
};


enum reduce_vol_sleep_level{
	REDUCE_VOL_V1_SLEEP = 0x06, // 1.2v
	REDUCE_VOL_V2_SLEEP = 0x05, //
    REDUCE_VOL_V3_SLEEP = 0x04, 
    REDUCE_VOL_V4_SLEEP = 0x03,
	
};


enum system_run_mode
{
	RW_NO_MODE = 0,
	RW_DUT_MODE = (0x01 << 0),
	RW_FCC_MODE = (0x01 << 1),
	RW_PN9_MODE = (0x01 << 2),
};


#define MCU_CLK_16M   1
#define MCU_CLK_32M	  2
#define MCU_CLK_48M   3
#define MCU_CLK_64M	  4
#define MCU_CLK_80M   5

#define REDUCE_VOL_SLEEP		1
#define REDUCE_VOL_SLEEP_LEVEL		REDUCE_VOL_V1_SLEEP

#define MCU_DEFAULT_CLK 	MCU_CLK_16M


void system_sleep_init(void);


void icu_init(void);

uint8_t get_sleep_mode(void);

void cpu_reduce_voltage_sleep(uint8_t level);

void cpu_idle_sleep(void);

uint8_t icu_get_sleep_mode(void);

void icu_set_sleep_mode(uint8_t v);

void cpu_wakeup(void);

void mcu_clk_config(void);

void udi_wdt_enable(uint16_t wdt_cnt);

void mcu_clk_switch(uint8_t clk);

void bk3435_singleWaveCfg(uint8_t freq, uint8_t power_level);

void bk3435_set_power(uint8_t power_level);
void rw_ip_sleep_test(uint32_t time);

 
uint8_t get_rw_ip_sleep_test_flag(void);
void Delay_us(int num);

void Delay_ms(int num);
#endif


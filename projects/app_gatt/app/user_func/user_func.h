#ifndef USER_FUNC_H_
#define USER_FUNC_H_
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include "gap.h"
#include "gapm_task.h"

#if USB_DRIVER
#define LED_G    0x11    //output default:1
#define SW_PIN   0x37    //output default:0

#define VAUX_DTH 0x32    //input
#define CABLE_SE 0x31    //input
#else
#define ADC1_PIN 0x31    //input
#define ADC2_PIN 0x32    //input
#define ADC_CS   0x13    //output default:0

#define STA_PIN1 0x10    //output default:0
#define STA_PIN2 0x11    //output default:0
#define STA_PIN3 0x33    //output default:0

#define ADC1_STATUS_BIT  	0x02
#define CONNECT_STATUS_BIT  0x01
#define ADAPTER_STATUS_BIT  0x02
#define DEFAULT_STATUS   	0x00
#define ADC1_ABNORMAL    	0x01
#define ADC2_ABNORMAL    	0x01
#define CONNECT_SUCCESS  	0x00
#define CONNECT_ABNORMAL 	0x01
#define ADAPTER_IN       	0x01
#define ADAPTER_OUT			0x00

#define ADV_REPORT_DEV_NUM    20
#endif

#define USER_AT_CMD_SAVE_ADDR 0x7E000
//#define BLE_RX_BUF_MAX        4096
extern uint8_t dmo_channel;
extern uint8_t *ble_data_buf;
//extern uint8_t ble_data_buf[];
extern uint16_t ble_rx_data_len;


typedef struct 
{
    uint8_t version[6];
    uint8_t time[15];
    uint8_t sn[10];
    uint8_t uuid[13];
    uint8_t name[32];
}dev_info_t;

typedef struct 
{
    uint8_t wdt_state;
    uint16_t wdt_time;
    uint8_t sleep_mode;
    uint32_t sleep_interval;
}dev_state_t;
#if !USB_DRIVER
typedef struct 
{
//    uint8_t  adc_en;
    uint16_t threshold_value[2][2];
    uint16_t adc_interval;
    uint16_t adc_count;
}adc_state_t;
#endif
#if USB_DRIVER
typedef struct 
{
    uint8_t pin;
    uint8_t val;
    uint8_t led;
    uint8_t mode;
}gpio_state_t;
#endif
typedef struct 
{
    uint16_t con_interval;//1.25
    uint16_t latency;//
    uint16_t timeout;//10ms   timeout >= con_interval x 1.25 x (latency + 1)x 6
}ble_parm_t;

typedef struct 
{
    uint8_t ble_mac[6];
    uint8_t ble_name[32];    
    
    uint8_t ble_power;
    uint8_t pincodelen;
    uint8_t pincode[16];
    uint8_t auto_conn_mac[6];
    uint8_t auto_conn_en;    
    uint32_t auto_conn_time;    
    uint16_t ble_buf_size;
    uint32_t baudrate[2];
    ble_parm_t ble_parm;
	uint8_t dis_reason;
}ble_state_t;

typedef struct 
{
    ble_state_t ble_state;    
    dev_info_t dev_info;
    dev_state_t dev_state;
    #if !USB_DRIVER
    adc_state_t adc_state;
    #endif
    #if USB_DRIVER
    gpio_state_t gpio_state;
    #endif
    uint8_t used_flag[13];
}user_info_t;

void user_init(void);
void user_task(void);

void app_wdt_feed(void);
void user_info_need_save(void);
void reverse(uint8_t *src, uint8_t len);
uint8_t mac_search(uint8_t *bdaddr);
int8_t channel_search(uint8_t *mac);
uint8_t free_channel_search(void);
void hexstr2bin(const char *macstr, uint8_t *mac, int len, uint8_t endian);

uint8_t get_at_rsp_ch(void);
void set_at_rsp_ch(uint8_t ch);
void set_at_mode(uint8_t mode);
uint8_t get_at_mode(void);
void set_data_mode(uint8_t mode);
uint8_t get_data_mode(void);

void set_fw_version(uint8_t *ver);
void get_fw_version(uint8_t *ver);
void set_version_time(uint8_t *ver);
void get_version_time(uint8_t *time);
void set_sn(uint8_t *sn);
void get_sn(uint8_t *sn);
void set_uuid(uint8_t *uuid);//void set_uuid(uint16_t uuid);
void get_uuid(uint8_t *uuid);//uint16_t get_uuid(void);
void set_product_name(uint8_t *name, uint8_t len);
uint8_t get_product_name(uint8_t *name);
void set_wdt_state(uint8_t flag);
uint8_t get_wdt_state(void);
void set_wdt_time(uint16_t time);
uint16_t get_wdt_time(void);
void set_dev_mode(uint8_t flag);
uint8_t get_dev_mode(void);
void set_sleep_interval(uint16_t time);
uint16_t get_sleep_interval(void);

uint8_t get_adc_en_status(void);
void set_adc_func(uint8_t flag);
uint8_t get_adc_func(void);
void set_adc_threshold(uint8_t ch, uint16_t *threshold);
void get_adc_threshold(uint8_t ch, uint16_t *threshold);
void set_adc_ch_en(uint8_t flag);
uint8_t get_adc_ch_status(void);
void set_adc_interval(uint16_t time);
uint16_t get_adc_interval(void);
void set_adc_count(uint16_t count);
uint16_t get_adc_count(void);
uint16_t get_adc_val(uint8_t ch);

void set_gpio_value(uint8_t pin, uint8_t value);
void set_gpio_led(uint8_t pin, uint8_t mode);
uint8_t get_gpio_led(void);
void set_ble_buf_size(uint16_t size);
uint16_t get_ble_buf_size(void);
enum appm_error set_ble_name(uint8_t len, uint8_t *name);
uint8_t get_ble_name(uint8_t *name);
uint8_t get_ble_state(void);
enum appm_error set_ble_mac(uint8_t *mac);//void set_ble_mac(uint8_t *mac);
enum appm_error get_ble_mac(uint8_t *mac);
void set_ble_power(uint8_t level);
uint8_t get_ble_power(void);
int8_t get_ble_rssi(void);
void set_ble_pincode(uint8_t len, uint8_t *pincode);
uint8_t get_ble_pincode(uint8_t *pincode);
void user_remove_bond(void);
void set_ble_erroinfo(uint8_t reason);
uint8_t get_ble_errorinfo(void);

#if !USB_DRIVER
void set_ble_auto_connect(uint8_t flag);//, uint8_t *mac
uint8_t get_ble_auto_connect(void);
void set_auto_connect_parm(uint8_t *mac, uint32_t duration_time);
uint32_t get_auto_connect_parm(uint8_t *mac);    
void set_connect_start(struct gap_bdaddr bdaddr);
void set_stop_connect(void);
bool appm_scan_adv_repor(void);
void appm_adv_report_filter_free(void);
uint8_t appm_add_adv_report_to_filter(struct gapm_ext_adv_report_ind *param);///(struct adv_report const *param)
#endif
uint8_t set_ble_parm(uint16_t interval, uint16_t latency, uint16_t timeout);
void get_ble_parm(ble_parm_t *parm);///ble_parm_t get_ble_parm(void);

uint8_t baudrate_check(uint32_t baudrate);
enum appm_error set_baudrate(uint8_t uart_id, uint32_t baudrate);
uint32_t get_baudrate(uint8_t uart_id);
void soft_reset(void);
uint8_t app_connect_device_info_get(void);

uint8_t app_send_at_cmd(uint8_t device_id,uint16_t packet_len,uint8_t* packet);///,uint8_t operation
uint8_t app_send_ble_data(uint8_t device_id,uint16_t packet_len,uint8_t* packet);
void app_send_keyboad_data(void);
void adc_task(void);
void set_gpio_status(uint8_t mode, uint8_t status);
void gpio_task(void);

void set_adc_status(uint8_t status);
#endif

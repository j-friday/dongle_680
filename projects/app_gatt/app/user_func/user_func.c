#include <string.h>
#include "rwip.h"
#include "llm_int.h"
#include "co_utils.h"

#include "ke_timer.h"

#include "app_init.h"
#include "app_fee0.h"
#include "app_task.h"
#include "app.h"

#include "user_func.h"

#include "icu.h"
#include "flash.h"
#include "uart.h"
#include "adc.h"
#include "gpio.h"
#include "wdt.h"
#include "rf.h"

static user_info_t user_info =
{
#if USB_DRIVER
	{"\x11\x22\x33\x44\x55\x66", "beken_slave", 4, 6, "123456", "\xFF\xFF\xFF\xFF\xFF\xFF", 0, 600, 2048, 115200, 115200, 24, 0, 600, 0},
#else
    {"\x11\x22\x33\x44\x55\x66", "beken_master", 4, 6, "123456", "\xBF\x06\x39\x69\x23\x0C", 1, 600, 2048, 115200, 115200, 24, 0, 600, 0},
#endif
    {"v01.05", "t20210623180609", "1234567890", "3CDE216050058" , "BLE_SCANN_GUN"},
    {0, 0xffff, 0, 100},
    #if !USB_DRIVER
    {0x500, 0x1000, 0x500, 0x1000, 1000, 256},
    #endif
    #if USB_DRIVER
    {0, 0, 0, 0},
    #endif
    {"UserInfo_v0.2"},
};
uint8_t dmo_channel = 0xFF;
uint8_t *ble_data_buf;
uint16_t ble_rx_data_len=0;
static uint8_t save_user_info_flag = 0;
static uint8_t at_mode_val = 1;
static uint8_t data_mode_val = 1;
#if USB_DRIVER
uint16_t usb_send_totail=0;
static uint8_t vaux_detech_v;
static uint8_t cable_sel_v;
#else
//bit0:gpio10 bit1:gpio11 bit2:gpio33
uint8_t gpio_status_value = DEFAULT_STATUS;

//bit7:ADC EN 
//bit0:CH1 EN 
//bit1:CH2 EN
static uint8_t ADC_EN_FLAG = 0x83;
static uint16_t adc_value_buf[2];
#endif
static void user_gpio_init(void)
{
#if USB_DRIVER
    //LED_G    0x11    //output default:1
    //SW_PIN   0x37    //output default:0
    gpio_config(LED_G, OUTPUT, PULL_NONE);
    gpio_config(SW_PIN, OUTPUT, PULL_NONE);
    
    gpio_set(LED_G, 1);
    gpio_set(SW_PIN, 0);
    
    //VAUX_DTH 0x32    //input
    //CABLE_SE 0x31    //input
    gpio_config(VAUX_DTH, INPUT, PULL_LOW);
    gpio_config(CABLE_SE, INPUT, PULL_LOW);
#else
    //ADC1_PIN 0x31    //adc1 input
    //ADC2_PIN 0x32    //adc2 input
    //ADC_CS   0x13    //output default:0
    gpio_config(ADC1_PIN, FLOAT, PULL_NONE);
    gpio_config(ADC2_PIN, FLOAT, PULL_NONE);
    gpio_config(ADC_CS,  OUTPUT, PULL_NONE);
    
    gpio_set(ADC_CS, 0);
    
    //STA_PIN1 0x10    //output default:0
    //STA_PIN2 0x11    //output default:0
    //STA_PIN3 0x33    //output default:0
    gpio_config(STA_PIN1, OUTPUT, PULL_NONE);
    gpio_config(STA_PIN2, OUTPUT, PULL_NONE);
    gpio_config(STA_PIN3, OUTPUT, PULL_NONE);
    
    gpio_set(STA_PIN1, 0);
    gpio_set(STA_PIN2, 0);
    gpio_set(STA_PIN3, 0);
#endif    
}

void user_info_need_save(void)
{
    save_user_info_flag = 1;
}

void erase_user_info(void)
{
    user_info_t device_type_temp;
    flash_read_data(&device_type_temp.ble_state.ble_mac[0],USER_AT_CMD_SAVE_ADDR,sizeof(device_type_temp));
    //if(memcmp(device_type_temp.used_flag, "UserInfo_v0.1", 13) == 0)
    {
        flash_erase(0, USER_AT_CMD_SAVE_ADDR, 0x1000, NULL);
    }
}

void save_user_info_to_flash(uint8_t *data, uint16_t len, uint32_t addr)
{
    if(save_user_info_flag == 1)
    {
        save_user_info_flag = 0;
        flash_erase(0, addr, 0x1000, NULL);
        flash_write(0, addr, len, data, NULL);
    }
}

void app_wdt_feed(void)
{
    if(user_info.dev_state.wdt_state == 1)
    {
        wdt_feed();
    }  
}

void copy_ble_parm(void)
{
    app_env.conn_intv = user_info.ble_state.ble_parm.con_interval;
    app_env.conn_latency = user_info.ble_state.ble_parm.latency;
    app_env.conn_super_to = user_info.ble_state.ble_parm.timeout;
}

void user_init(void)
{
    user_info_t tempbuf;//[sizeof(user_info_t)]
    //read flash for default parm
    flash_read_data(&tempbuf.ble_state.ble_mac[0], USER_AT_CMD_SAVE_ADDR, sizeof(user_info_t));
    //check flash validity
    if(memcmp(tempbuf.used_flag, user_info.used_flag, 13) == 0)
    {
        memcpy(&user_info.ble_state.ble_mac[0], &tempbuf.ble_state.ble_mac[0], sizeof(user_info_t));
    }
    //gpio init
    user_gpio_init();
    
    //status init
    copy_ble_parm();
    
    set_ble_buf_size(user_info.ble_state.ble_buf_size);
    #if (!USB_DRIVER)
	UartDbgInit(get_baudrate(0));
    #endif
    Uart1DbgInit(get_baudrate(1));
    
    if(user_info.dev_state.wdt_state == 1)
    {
        wdt_enable(user_info.dev_state.wdt_time);
    }
}

void user_task(void)
{
    //gpio task
    gpio_task();
    
    #if USB_DRIVER
    app_send_keyboad_data();
    #else
    //adc task
    adc_task();
	auto_conn_task();
    #endif
  
    //
    app_wdt_feed();
    //user info save
    save_user_info_to_flash(&user_info.ble_state.ble_mac[0], sizeof(user_info_t), USER_AT_CMD_SAVE_ADDR);
}

void reverse(uint8_t *src, uint8_t len)
{
    int i;
    int x; 
    for(i = 0; i < len/2; i++)
    {
        x = src[i];
        src[i] = src[len -i-1];
        src[len -i-1] = x;
    }
}

uint8_t mac_search(uint8_t *bdaddr)
{
    uint8_t num = 0;
    for(int i = 0;i < APP_IDX_MAX;i++)
    {
        
        if((ke_state_get(KE_BUILD_ID(TASK_APP,i)) == APPC_LINK_CONNECTED) ||(ke_state_get(KE_BUILD_ID(TASK_APP,i)) == APPC_SDP_DISCOVERING)\
            ||(ke_state_get(KE_BUILD_ID(TASK_APP,i)) == APPC_SERVICE_CONNECTED) )
        {
            num += 0x10;
            if(memcmp((const void *)&(app_env.con_dev_addr[i].addr.addr[0]),(const void *)&(bdaddr[0]),6)==0)
            {
                num++; 
                break;
            } 
        }        
    }
    return num;
} 

int8_t channel_search(uint8_t *mac)
{
	for(int i = 0;i < APP_IDX_MAX;i++)
	{			
		if((ke_state_get(KE_BUILD_ID(TASK_APP,i)) == APPC_LINK_CONNECTED) ||(ke_state_get(KE_BUILD_ID(TASK_APP,i)) == APPC_SDP_DISCOVERING)\
				||(ke_state_get(KE_BUILD_ID(TASK_APP,i)) == APPC_SERVICE_CONNECTED) )
		{
            bk_printf("channel_search: %02x%02x%02x%02x%02x%02x  ",\
            app_env.con_dev_addr[i].addr.addr[0],app_env.con_dev_addr[i].addr.addr[1],\
            app_env.con_dev_addr[i].addr.addr[2],app_env.con_dev_addr[i].addr.addr[3],\
            app_env.con_dev_addr[i].addr.addr[4],app_env.con_dev_addr[i].addr.addr[5]);
			bk_printf("mac: %02x%02x%02x%02x%02x%02x\r\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
            if(memcmp(mac,app_env.con_dev_addr[i].addr.addr,GAP_BD_ADDR_LEN)==0)
			{
				//bk_printf("i:%x\r\n",i);
				return i; 
			}
		}
	}
	return -1;
}

uint8_t free_channel_search(void)
{
    uint8_t num = 0;   
    for(int i = 0;i < APP_IDX_MAX;i++)
    {
        
        if((ke_state_get(KE_BUILD_ID(TASK_APP,i)) == APPC_LINK_CONNECTED) ||(ke_state_get(KE_BUILD_ID(TASK_APP,i)) == APPC_SDP_DISCOVERING)\
            ||(ke_state_get(KE_BUILD_ID(TASK_APP,i)) == APPC_SERVICE_CONNECTED) )
        {
            num++; 
        }
        else break;
    }
    return num;
}

static uint8_t hex(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'a' && c <= 'z')
        return c - 'a' + 10;
    if (c >= 'A' && c <= 'Z')
        return c - 'A' + 10;
    return 0;
}

void hexstr2bin(const char *macstr, uint8_t *mac, int len, uint8_t endian)
{
    int i;
    if(endian)
    {
        for (i = 0; i < len && macstr[2 * i]; i++)
        {
            mac[len - 1 - i] = hex(macstr[2 * i]) << 4;
            mac[len - 1 - i] |= hex(macstr[2 * i + 1]);
        }
    }
    else
    {
        for (i=0; i < len && macstr[2 * i]; i++) 
        {
            mac[i] = hex(macstr[2 * i]) << 4;
            mac[i] |= hex(macstr[2 * i + 1]);
        }
    }
}
#include "cli.h"
static uint8_t at_rsp_ch = 0;//0:uart 1:ble
void set_at_rsp_ch(uint8_t ch)
{
    at_rsp_ch = ch;
}

uint8_t get_at_rsp_ch(void)
{
    return at_rsp_ch;
}

void set_at_mode(uint8_t mode)
{
    at_mode_val = mode;
    //if(mode == 2)cliexit = 1;
}

uint8_t get_at_mode(void)
{
    return at_mode_val;
}

void set_data_mode(uint8_t mode)
{
    data_mode_val = mode;
}

uint8_t get_data_mode(void)
{
    return data_mode_val;
}

void set_fw_version(uint8_t *ver)
{
    memcpy(user_info.dev_info.version, ver, 6);
}

void get_fw_version(uint8_t *ver)
{
    memcpy(ver, user_info.dev_info.version, 6);
}

void set_version_time(uint8_t *ver)
{
    memcpy(user_info.dev_info.time, ver, 15);
}

void get_version_time(uint8_t *time)
{
    memcpy(time, user_info.dev_info.time, 15);
}

void set_sn(uint8_t *sn)
{
    memcpy(user_info.dev_info.sn, sn, 10);
}

void get_sn(uint8_t *sn)
{    
    memcpy(sn, user_info.dev_info.sn, 10);
}

void set_uuid(uint8_t *uuid)
{
    memcpy(user_info.dev_info.uuid, uuid, 13);
}

void get_uuid(uint8_t *uuid)
{
    memcpy(uuid, user_info.dev_info.uuid, 13);
}

void set_product_name(uint8_t *name, uint8_t len)
{
    memset(user_info.dev_info.name, 0x00, sizeof(user_info.dev_info.name));
    memcpy(user_info.dev_info.name, name, len);
}

uint8_t get_product_name(uint8_t *name)
{
    uint8_t len;
    len = strlen((const char*)user_info.dev_info.name);
    memcpy(name, user_info.dev_info.name, len);
    return len;
}

void set_wdt_state(uint8_t flag)///1: enable 0: disable
{
    user_info.dev_state.wdt_state = flag;
    if(user_info.dev_state.wdt_state == 1)
    {
        wdt_enable(user_info.dev_state.wdt_time);
    }
    else
    {
        wdt_disable();
    }
}

uint8_t get_wdt_state(void)
{
    return user_info.dev_state.wdt_state;
}

void set_wdt_time(uint16_t time)///count
{
    user_info.dev_state.wdt_time = time;
    if(user_info.dev_state.wdt_state == 1)
    {
        wdt_enable(user_info.dev_state.wdt_time);
    }
}

uint16_t get_wdt_time(void)
{
    return user_info.dev_state.wdt_time;
}

void set_dev_mode(uint8_t flag)
{
    user_info.dev_state.sleep_mode = flag;
}

uint8_t get_dev_mode(void)
{
    return user_info.dev_state.sleep_mode;
}

void set_sleep_interval(uint16_t time)
{
    user_info.dev_state.sleep_interval = time;
}

uint16_t get_sleep_interval(void)
{
    return user_info.dev_state.sleep_interval;
}
#if !USB_DRIVER
void set_adc_func(uint8_t flag)///1: open 0: close
{
    //ADC_EN_FLAG &= 0x7F;
    //ADC_EN_FLAG |= (flag & 0x01) << 7;
    ADC_EN_FLAG = flag;
    if(ADC_EN_FLAG & 0x80)
    {
        #if(ADC_DRIVER) 
        adc_init(1, 1);
        adc_init(2, 1);
        #endif
    }
}

uint8_t get_adc_en_status(void)
{
    return ADC_EN_FLAG;
}

uint8_t get_adc_func(void)///1: open 0: close
{
    return ((get_adc_en_status() & 0x80) >> 7);
}

//ch:0~1;
void set_adc_threshold(uint8_t ch, uint16_t *threshold)
{
    user_info.adc_state.threshold_value[ch][0] = threshold[0];
    user_info.adc_state.threshold_value[ch][1] = threshold[1];
}

//ch:0~1;
void get_adc_threshold(uint8_t ch, uint16_t *threshold)
{
    threshold[0] = user_info.adc_state.threshold_value[ch][0];
    threshold[1] = user_info.adc_state.threshold_value[ch][1];
}

void set_adc_interval(uint16_t time)
{
    user_info.adc_state.adc_interval = time;
}

uint16_t get_adc_interval(void)
{
    return user_info.adc_state.adc_interval;
}

void set_adc_count(uint16_t count)
{
    user_info.adc_state.adc_count = count;
}

uint16_t get_adc_count(void)
{
    return user_info.adc_state.adc_count;
}

uint16_t get_adc_val(uint8_t ch)
{
    return adc_value_buf[ch-1];
}

#endif
#if USB_DRIVER
#include "fee0s_task.h"

void set_gpio_value(uint8_t pin, uint8_t value)
{
    gpio_config(pin, OUTPUT, PULL_NONE);
    gpio_set(pin, value);
}

void set_gpio_led(uint8_t pin, uint8_t mode)
{
    user_info.gpio_state.led = pin;
    user_info.gpio_state.mode = mode;
    ke_timer_set(APP_LED_MODE_PROCESS_HANDLER, TASK_APP, 2);
}

uint8_t get_gpio_led(void)
{
    return user_info.gpio_state.mode;
}

#endif
void set_ble_buf_size(uint16_t size)
{
    user_info.ble_state.ble_buf_size = size;
    if(ble_data_buf != NULL)
    {
        free(ble_data_buf);
    }

    ble_data_buf = malloc(size);
}

uint16_t get_ble_buf_size(void)
{
    return user_info.ble_state.ble_buf_size;
}

void set_ble_erroinfo(uint8_t reason)
{
	user_info.ble_state.dis_reason = reason;
}

uint8_t get_ble_errorinfo(void)
{
	return user_info.ble_state.dis_reason;
}

enum appm_error set_ble_name(uint8_t len, uint8_t *name)
{
    if(rwip_param.set(PARAM_ID_DEVICE_NAME, len, (uint8_t *)&name[0]) != PARAM_OK)
    {
        return APPM_ERROR_STATE;
    }
    memset(user_info.ble_state.ble_name, 0x00, 32);
    memcpy(user_info.ble_state.ble_name, name, len);
    
    return APPM_ERROR_NO_ERROR;
}

uint8_t get_ble_name(uint8_t *name)
{
    uint8_t len;
    //len = APP_DEVICE_NAME_MAX_LEN;
    if (rwip_param.get(PARAM_ID_DEVICE_NAME, &len, name) != PARAM_OK)
    {        
        memcpy(name, user_info.ble_state.ble_name, APP_DEVICE_NAME_MAX_LEN);
    }
    len = strlen((const char*)name);
    return len;
}

uint8_t get_ble_state(void)
{
    uint8_t state;
#if USB_DRIVER    
    if(ke_state_get(KE_BUILD_ID(TASK_APP,0)) == APPC_LINK_CONNECTED)
#else
    if(ke_state_get(KE_BUILD_ID(TASK_APP,dmo_channel)) == APPC_SERVICE_CONNECTED)
#endif    
    {
        state = 1;
    }
    else
    {
        state = 0;
    }
    return state;
}

enum appm_error set_ble_mac(uint8_t *mac)
{  
    struct bd_addr b_addr;
    
    //bk_printf("\r\nsetmac: %02x%02x%02x%02x%02x%02x\r\n",
    //        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    if(rwip_param.set(PARAM_ID_BD_ADDRESS,6,(uint8_t *)&mac[0]) != PARAM_OK)
    {            
        return APPM_ERROR_STATE;
    }
    memcpy(llm_env.local_pub_addr.addr,mac,6);      
    memcpy(b_addr.addr,mac,6);
    memcpy(&co_default_bdaddr, mac, 6);
    memcpy(user_info.ble_state.ble_mac, mac, 6);
    
    return APPM_ERROR_NO_ERROR;
}

enum appm_error get_ble_mac(uint8_t *mac)
{
    struct bd_addr ble_addr = co_default_bdaddr;//llm_env.local_pub_addr;
    uint8_t mac_len = 6;
    if(rwip_param.get(PARAM_ID_BD_ADDRESS,&mac_len,(uint8_t *)&mac[0]) != PARAM_OK)
    {            
        mac[5] = ble_addr.addr[5];
        mac[4] = ble_addr.addr[4];
        mac[3] = ble_addr.addr[3];
        mac[2] = ble_addr.addr[2];
        mac[1] = ble_addr.addr[1];
        mac[0] = ble_addr.addr[0];
        bk_printf("PARAM_unOK\r\n");
    }
   
    //bk_printf("\r\n+MAC: %02x-%02x-%02x-%02x-%02x-%02x\r\n",
    //        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
   
    return APPM_ERROR_NO_ERROR;
}

void set_ble_power(uint8_t level)
{
    rf_power_set(level);
    user_info.ble_state.ble_power = level;
}

uint8_t get_ble_power(void)
{
    return user_info.ble_state.ble_power;//rf_power_get();
}
#include "llm.h"
extern int8_t lld_con_rssi_get(uint8_t link_id);
int8_t get_ble_rssi(void)
{
    //return lld_con_rssi_get(0) + (llm_rx_path_comp_get()/10); 
	return get_rf_rssi();
}

void set_ble_pincode(uint8_t len, uint8_t *pincode)
{
    user_info.ble_state.pincodelen = len;
    memset(user_info.ble_state.pincode, 0x00, 16);
    memcpy(user_info.ble_state.pincode, pincode, len);
}

uint8_t get_ble_pincode(uint8_t *pincode)
{
    memcpy(pincode, user_info.ble_state.pincode, user_info.ble_state.pincodelen);
    return user_info.ble_state.pincodelen;
}
#include "app_sec.h"
#include "app_adv.h"
void user_remove_bond(void)
{
    app_sec_remove_bond();

    if((ke_state_get(TASK_APP) == APPC_LINK_CONNECTED) || (ke_state_get(TASK_APP) == APPC_SERVICE_CONNECTED)) //连接状态
    {
        appm_disconnect(0); 
    }
}
#if !USB_DRIVER
//1:en 0:disen
void set_ble_auto_connect(uint8_t flag)
{
    user_info.ble_state.auto_conn_en = flag;

    if(flag)
    {
        struct gap_bdaddr bdaddr;
        memcpy(bdaddr.addr.addr, user_info.ble_state.auto_conn_mac, 6);
        bdaddr.addr_type = ((bdaddr.addr.addr[5] & 0xC0) == 0xC0) ? ADDR_RAND : ADDR_PUBLIC;
        set_connect_start(bdaddr);
        ke_timer_set(APP_BLE_AUTO_CONN_HANDLER, TASK_APP, user_info.ble_state.auto_conn_time);
    }
}

void set_ble_auto_connect_status(uint8_t flag)
{
	user_info.ble_state.auto_conn_en = flag;
}

uint8_t get_ble_auto_connect(void)
{
    return user_info.ble_state.auto_conn_en;
}

void set_auto_connect_parm(uint8_t *mac, uint32_t duration_time)
{
    memcpy(user_info.ble_state.auto_conn_mac, mac, 6);
    user_info.ble_state.auto_conn_time = duration_time;
}

uint32_t get_auto_connect_parm(uint8_t *mac)
{
    memcpy(mac, user_info.ble_state.auto_conn_mac, 6);
    return user_info.ble_state.auto_conn_time;
}

void set_connect_mac(uint8_t *mac)
{
	memcpy(user_info.ble_state.auto_conn_mac, mac, 6);
}

void set_connect_start(struct gap_bdaddr bdaddr)
{
    appm_set_connect_dev_addr(bdaddr);
    appm_update_init_state(1);
}

void set_stop_connect(void)
{
    appm_stop_connencting();
    appm_update_init_state(false);    
}

#include "app_scan.h"
extern struct appm_env_tag appm_env;
bool appm_scan_adv_repor(void)
{
    bool ret = true;
//    uint8_t scan_dev_num = scanned_device_num;///0;
    uint8_t str_name;
    uint8_t cursor = 0;
    uint8_t str_buf[31];

    memset(str_buf,0,31);
    bd_addr_t default_addr = {{0,0,0,0,0,0}};
    // check that filter array exists
    if(appm_env.scan_filter != NULL)
    {
        // Find provided address in filter
        while(((memcmp(&(appm_env.scan_filter[cursor].adv_addr), &default_addr,sizeof(bd_addr_t)) != 0 )//while(((memcmp(&(appm_env.scan_filter[cursor].rsp_data[2]), slave_device.serv_uuid, 16) != 0)//
                && (cursor <  ADV_REPORT_DEV_NUM)))
        {
            //UART_PRINTF("");
            cursor++;
        }
        aos_cli_printf("+SCAN:\r\n{\r\n");
        bk_printf("appm_env.scan_filter[%d]\r\n", cursor);
        for(int i = 0; i < cursor;i++)
        {
            aos_cli_printf("+INQ:%d ", i); 
            for(int j = 0; j < 6;j++)
            {
                aos_cli_printf("%02X",appm_env.scan_filter[i].adv_addr.addr[5-j]);
            }
            str_name = appm_adv_data_decode_name(appm_env.scan_filter[i].adv_data_len,appm_env.scan_filter[i].adv_data,str_buf);
            if(str_name)
            {
                aos_cli_printf(",%s",str_buf);
                memset(str_buf,0,31);
            }
            else
            {
                str_name = appm_adv_data_decode_name(appm_env.scan_filter[i].rsp_data_len,appm_env.scan_filter[i].rsp_data,str_buf);            
                if(str_name)
                {
                    aos_cli_printf(",%s",str_buf);
                    memset(str_buf,0,31);                                       
                }
                else aos_cli_printf(", ");               
            }
            aos_cli_printf(",%d",appm_env.scan_filter[i].rssi);
            aos_cli_printf("\r\n");
        }
        aos_cli_printf("}\r\n");
    }
    return ret;
}

void appm_adv_report_filter_free(void)
{
    if(appm_env.scan_filter != NULL)
    {
        ke_free(appm_env.scan_filter);
        appm_env.scan_filter = NULL;
		bk_printf("%s\r\n",__func__);
    }
}

uint8_t appm_add_adv_report_to_filter(struct gapm_ext_adv_report_ind *param)///(struct adv_report const *param)
{
    bd_addr_t default_addr = {{0,0,0,0,0,0}};
    uint8_t cursor = 0;

    // allocate scan filtered device list if needed.
    if(appm_env.scan_filter == NULL)
    {
        appm_env.scan_filter =
                (struct adv_rsp_report*) ke_malloc(sizeof(struct adv_rsp_report) * ADV_REPORT_DEV_NUM, KE_MEM_NON_RETENTION);

        memset(appm_env.scan_filter, 0, sizeof(struct adv_rsp_report) * ADV_REPORT_DEV_NUM);
        //UART_PRINTF("######### malloc appm_env.scan_filter ######### \r\n");
    }

    if((param->info & GAPM_REPORT_INFO_REPORT_TYPE_MASK) & (0x01 << GAPM_REPORT_TYPE_ADV_LEG))//scan rsp report.//if(param->evt_type != LL_SCAN_RSP) // adv report LL_SCAN_RSP GAPM_REPORT_INFO_COMPLETE_BIT
    {
        //USER_PRINTF("rsp report\r\n");
        while((memcmp(&(appm_env.scan_filter[cursor].adv_addr.addr[0]),&param->trans_addr.addr.addr[0], sizeof(bd_addr_t)) != 0) /*&& (appm_env.scan_filter[cursor].addr_type != addr_type)*/
            && (cursor < ADV_REPORT_DEV_NUM))
        {
            cursor++;
			//UART_PRINTF("# rsp report param->info %x cursor:%d\r\n", param->info, cursor);
        }
        if (cursor < ADV_REPORT_DEV_NUM)
        {
            memcpy(appm_env.scan_filter[cursor].rsp_data, param->data, param->length);
            appm_env.scan_filter[cursor].rsp_data_len = param->length;  
            appm_env.scan_filter[cursor].rssi = param->rssi;
        }
        else
        {
            cursor = 0;
        } 
    }
    else// else adv report
    { 
        // find first available space in array
        //UART_PRINTF("memcmp1:%d, memcmp2:%d\r\n",memcmp(&(appm_env.scan_filter[cursor].adv_addr), &default_addr, sizeof(bd_addr_t)),memcmp(&(appm_env.scan_filter[cursor].adv_addr),&param->trans_addr.addr, sizeof(bd_addr_t)));
        while((memcmp(&(appm_env.scan_filter[cursor].adv_addr.addr), &default_addr, sizeof(bd_addr_t)) != 0) 
					  && (memcmp(&(appm_env.scan_filter[cursor].adv_addr.addr),&param->trans_addr.addr, sizeof(bd_addr_t)) != 0) /*&& (appm_env.scan_filter[cursor].addr_type != addr_type)*/
            && (cursor <  ADV_REPORT_DEV_NUM))
        {
            cursor++;
			//UART_PRINTF("memcmp1:%d, memcmp2:%d\r\n",memcmp(&(appm_env.scan_filter[cursor].adv_addr), &default_addr, sizeof(bd_addr_t)),memcmp(&(appm_env.scan_filter[cursor].adv_addr),&param->trans_addr.addr, sizeof(bd_addr_t)));
        	//UART_PRINTF("cursor:%d\r\n",cursor);
        }
        
        // copy provided device address in array
        if (cursor < ADV_REPORT_DEV_NUM)
        {
            memcpy(&(appm_env.scan_filter[cursor].adv_addr.addr), &param->trans_addr.addr, sizeof(bd_addr_t));
            memcpy(appm_env.scan_filter[cursor].adv_data, param->data, param->length);
            appm_env.scan_filter[cursor].adv_data_len = param->length;
            appm_env.scan_filter[cursor].adv_addr_type = param->trans_addr.addr_type;
            appm_env.scan_filter[cursor].rssi = param->rssi;
        }        
    }	
    //cursor_max = MAX(cursor_max, cursor);
	//UART_PRINTF("appm_add_adv_report_to_filter cursor:%d\r\n",cursor);
    return cursor;
}

#endif
//interval:1.25ms/unit 
//latency:1 unit
//timeout:10ms/unit
uint8_t set_ble_parm(uint16_t interval, uint16_t latency, uint16_t timeout)
{    
    if(rwip_param.set(NVDS_TAG_CONN_INTV, 2, (uint8_t*) &interval) != PARAM_OK)
    {
        bk_printf("set:NVDS_TAG_CONN_INTV err\r\n");
        return 1;
    }
    if(rwip_param.set(NVDS_TAG_CONN_LAT, 2, (uint8_t*) &latency) != PARAM_OK)
    {
        bk_printf("set:NVDS_TAG_CONN_LAT err\r\n");
        return 1;
    }
    if(rwip_param.set(NVDS_TAG_CONN_SUP_TO, 2, (uint8_t*) &timeout) != PARAM_OK)
    {
        bk_printf("set:NVDS_TAG_CONN_SUP_TO err\r\n");
        return 1;
    }
    
    user_info.ble_state.ble_parm.con_interval = interval;
    user_info.ble_state.ble_parm.latency      = latency;
    user_info.ble_state.ble_parm.timeout      = timeout;
    copy_ble_parm();
    return 0;
}

void get_ble_parm(ble_parm_t *parm)
{
    memcpy(parm, (uint8_t *)&(user_info.ble_state.ble_parm), sizeof(ble_parm_t));
}

uint8_t baudrate_check(uint32_t baudrate)
{
    uint8_t err_code = 0x00;
    if((4800 <= baudrate) && (baudrate <= 2000000))
	{
		switch(baudrate)
		{
		case 4800:
		case 9600:
		case 19200:
		case 38400:
		case 57600:
		case 115200:
        case 128000:
        case 230400:
        case 256000:
        case 460800:
        case 500000:
        case 512000:
        case 600000:
        case 750000:
        case 921600:            
        case 1000000:
        case 1500000:
        case 2000000:
            break;
		default:
			err_code = 0x01;
            break;
		}			
	}
	else
	{
		err_code = 0x01;
	}
    return err_code;
}
//uart_id 0:uart1 1:uart2
enum appm_error set_baudrate(uint8_t uart_id, uint32_t baudrate)
{
    bk_printf("baudrate %d\r\n",baudrate);
      
	//if(rwip_param.set(PARAM_ID_UART_BAUDRATE,sizeof(baudrate),(uint8_t *)&baudrate) != PARAM_OK)
	//{		
	//	return APPM_ERROR_STATE;
	//}
    #if (!USB_DRIVER)
    if(uart_id == 0)
    {
        UartDbgInit(baudrate);
    }
    else
    #endif
    if(uart_id == 1)
    {
        Uart1DbgInit(baudrate);
    }
    else return APPM_ERROR_STATE; 
    user_info.ble_state.baudrate[uart_id] = baudrate;
    return APPM_ERROR_NO_ERROR;    
}
//uart_id 0:uart1 1:uart2
uint32_t get_baudrate(uint8_t uart_id)
{
    if(baudrate_check(user_info.ble_state.baudrate[uart_id]))
    {
        return 115200;
    }
    else
    {
        return user_info.ble_state.baudrate[uart_id];
    }
}

void soft_reset(void)
{
    wdt_enable(100);
}

uint8_t app_connect_device_info_get(void)
{
    uint8_t num = 0;
    
    aos_cli_printf("\r\n+CHINFO{\r\n");
    for(int i = 0;i < APP_IDX_MAX;i++)
    {     
        if((ke_state_get(KE_BUILD_ID(TASK_APP,i)) == APPC_LINK_CONNECTED) ||(ke_state_get(KE_BUILD_ID(TASK_APP,i)) == APPC_SDP_DISCOVERING)\
            ||(ke_state_get(KE_BUILD_ID(TASK_APP,i)) == APPC_SDP_DISCOVERING) || (ke_state_get(KE_BUILD_ID(TASK_APP,i)) == APPC_SERVICE_CONNECTED) )
        {
            num++;
            for(int j = 0; j < GAP_BD_ADDR_LEN;j ++)
            {
                aos_cli_printf("%02x",app_env.con_dev_addr[i].addr.addr[j]);
            }aos_cli_printf(",%x,3\r\n",app_env.con_dev_addr[i].addr_type); 
        }
    }
    aos_cli_printf("+CHINFO}\r\n\r\nOK\r\n");
    
    return num;
}
uint8_t app_send_at_cmd(uint8_t device_id,uint16_t packet_len,uint8_t* packet)///,uint8_t operation
{
    if((device_id >= BLE_CONNECTION_MAX))
        goto EXIT;

	if(app_env.role[device_id] == ROLE_SLAVE)
    {
        bk_printf("ROLE_SLAVE\r\n");
        if(app_fee5_send_ind(device_id,packet_len,packet) != APPM_ERROR_NO_ERROR)
            goto EXIT;       
    }
    else 
    {      
        bk_printf("ROLE_MASTER\r\n");        
        if(appc_write_service_data_req(device_id,app_fee0_env.svc_fee3_write_handle[device_id],packet_len,packet)!= APPM_ERROR_NO_ERROR)
            goto EXIT;        
    }
    
	return APPM_ERROR_NO_ERROR;
    
EXIT:	
    bk_printf("send error %x\r\n", device_id);
	return APPM_ERROR_STATE;    
}

uint8_t app_send_ble_data(uint8_t device_id,uint16_t packet_len,uint8_t* packet)///,uint8_t operation
{
	if((device_id >= BLE_CONNECTION_MAX))
        goto EXIT;

	if(app_env.role[device_id] == ROLE_SLAVE)
    {        
        if(app_fee4_send_ntf(device_id,packet_len,packet) != APPM_ERROR_NO_ERROR)
            goto EXIT;       
    }
    else 
    {     
        if(appc_write_service_data_req(device_id,app_fee0_env.svc_fee2_write_handle[device_id],packet_len,packet)!= APPM_ERROR_NO_ERROR)
            goto EXIT;        
    }
    
	return APPM_ERROR_NO_ERROR;
    
EXIT:
	return APPM_ERROR_STATE;
}

#if USB_DRIVER
#include "driver_usb.h"
struct rev_ntf_data notify_data;
extern void test_usb_device(void);

uint8_t hexvalue[94]=
{
    0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,
    0x61,0x62,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6A,
    0x6B,0x6C,0x6D,0x6E,0x6F,0x70,0x71,0x72,0x73,0x74,
    0x75,0x76,0x77,0x78,0x79,0x7A,
    0x2D,0x3D,0x5B,0x5D,0x5C,0x3B,0x27,0x2C,0x2E,0x2F,0x60,
    /*以下是上面所有对应按键的shift大写*/
    0x29,0x21,0x40,0x23,0x24,0x25,0x5E,0x26,0x2A,0x28,
    0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4A,
    0x4B,0x4C,0x4D,0x4E,0x4F,0x50,0x51,0x52,0x53,0x54,
    0x55,0x56,0x57,0x58,0x59,0x5A,
    0x5F,0x2B,0x7B,0x7D,0x7C,0x3A,0x22,0x3C,0x3E,0x3F,0x7E
};	
//0-9,a-z,-=[]\;',./` hex

uint8_t keyvalue[47]=
{
    0x27,0x1E,0x1F,0x20,0x21,0x22,0x23,0x24,0x25,0x26,
    0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,
    0x0E,0x0F,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,
    0x18,0x19,0x1A,0x1B,0x1C,0x1D,
    0x2D,0x2E,0x2F,0x30,0x31,0x33,0x34,0x36,0x37,0x38,0x35
};
//0-9,a-z,-=[]\;',./ keyvalue

void app_send_keyboad_data(void)
{
    uint8_t keybuf[8];
    if(get_data_mode() != 1) return;
    if(ble_rx_data_len > 0)
    {
        if(cable_sel_v == 0)
        {
            if((usb_send_totail < ble_rx_data_len) && (check_b_isTRxing() == FALSE))//for(uint16_t i = 0; i< ble_rx_data_len; i++)
            {
                //bk_printf("%02x ",ble_data_buf[i]);
                //uart_clean_printf("%02x ",ble_data_buf[i]); 
                memset(keybuf,0x0,8);        
                for(uint8_t j = 0; j< 94; j++)
                {
                    if((ble_data_buf[usb_send_totail] == hexvalue[j]) && (j < 47))/*正常按键*/
                    {
                        keybuf[2] = keyvalue[j];
                        notify_data.notify_standard_key_status=1;
                        memcpy(&notify_data.notify_standard_key[1],keybuf,sizeof(keybuf));//按下
                        test_usb_device();
                        //Delay_us(1000);
                        
                        memset(keybuf,0x0,8);
                        notify_data.notify_standard_key_status=1;
                        memcpy(&notify_data.notify_standard_key[1],keybuf,sizeof(keybuf));//释放
                        //test_usb_device();                        
                        
                        //Delay_us(1000);
                        break;
                    }
                    else if((ble_data_buf[usb_send_totail] == hexvalue[j]) && (j >= 47))/*shift组合按键*/
                    {
                        //keybuf[0] = 0x02;
                        //notify_data.notify_standard_key_status=1;
                        //memcpy(&notify_data.notify_standard_key[1],keybuf,sizeof(keybuf));//left shift按下
                        //test_usb_device();
                        
                        //Delay_us(400);
                        memset(keybuf,0x0,8);
                        keybuf[0] = 0x02;
                        keybuf[2] = keyvalue[j-47];
                        notify_data.notify_standard_key_status=1;
                        memcpy(&notify_data.notify_standard_key[1],keybuf,sizeof(keybuf));//按下
                        test_usb_device();                        
                        //Delay_us(1000);                        
                        memset(keybuf,0x0,8);
                        notify_data.notify_standard_key_status=1;
                        memcpy(&notify_data.notify_standard_key[1],keybuf,sizeof(keybuf));//释放
                        //test_usb_device();   
                        //Delay_us(1000);   
                        break;
                    }
                }
                usb_send_totail++;
                if(usb_send_totail == ble_rx_data_len)
                {
                    usb_send_totail = 0;
                    ble_rx_data_len = 0;
                }
            }
        }
        else
        {
            uart1_send(ble_data_buf, ble_rx_data_len);
            ble_rx_data_len = 0;
        }        
    }    
}

#else
void auto_conn_task(void)
{
	static rwip_time_t current_time;
	static rwip_time_t pre_time = {0, 0};

	if(get_ble_auto_connect())
	{
		current_time = rwip_time_get();
		if((current_time.hs -  pre_time.hs) > (user_info.ble_state.auto_conn_time * 10 * 1000 + 3000000) / HALF_SLOT_SIZE * 2)
		{
			if(ke_state_get(KE_BUILD_ID(TASK_APP,dmo_channel)) != APPC_SERVICE_CONNECTED)
			{
				set_ble_auto_connect(1);
			}
			pre_time = current_time;
		}
	}
}

void adc_task(void)
{
    static uint32_t cnt = 0;
    static rwip_time_t current_time;
    static rwip_time_t pre_time = {0, 0};  
    
    if(get_adc_func() == 1)
    {
        current_time = rwip_time_get();
        if((current_time.hs -  pre_time.hs) > user_info.adc_state.adc_interval)
        {
            uint8_t ch_en;
            uint16_t count;
            ch_en = get_adc_en_status() & 0x03;
            count = get_adc_count();
            //bk_printf("rwip_schedule :%d,t:%d,gpio:%d\r\n",cnt,current_time.hs,gpio_get_input(0x17));
            if((count > 0) && (count >= cnt))
            {
                cnt++;
                #if(ADC_DRIVER) 
                if(ch_en & 0x01)
                {                    
                    adc_value_buf[0] = adc_get_value(1, 1);
					if(adc_value_buf[0] > 235 || adc_value_buf[0] < 200)
					{
						set_gpio_status(ADC1_STATUS_BIT, 1);
					}
					else
					{
						set_gpio_status(ADC1_STATUS_BIT, 0);
					}
                }
                if(ch_en & 0x02)
                {        
                    adc_value_buf[1] = adc_get_value(2, 1);
                }
                #endif
            }
            else
            {
                cnt = 0;
                set_adc_func(get_adc_en_status() & 0x03);
            } 
            pre_time = current_time;
        }
    }    
}

void set_gpio_status(uint8_t mode, uint8_t status)
{
	if(status)
	{	
		gpio_status_value |= (0x1 << mode);
	}
	else
	{
		gpio_status_value &= ~(0x1 << mode);
	}
}

void gpio_status_display(uint8_t status)
{
    gpio_set(STA_PIN1, status & 0x01);
    gpio_set(STA_PIN2, (status >> 1) & 0x01);
    gpio_set(STA_PIN3, (status >> 2) & 0x01);
}

void set_adc_status(uint8_t status)
{
	if(status)
	{
		gpio_set(STA_PIN1, 0x01);
	}
	else
	{
		gpio_set(STA_PIN1, 0x00);
	}
}
#endif

void gpio_task(void)
{
#if USB_DRIVER
    static uint8_t vaux_detech_vback;
    vaux_detech_v = gpio_get_input(VAUX_DTH);
    if(vaux_detech_vback != vaux_detech_v)
    {
        uint8_t str[20];
        vaux_detech_vback = vaux_detech_v;
        snprintf((char *)str, 20,"vaux_detech_value:%d", vaux_detech_v);
        app_send_ble_data(0, 19, str);//通道需要注意，从机不能确认主机端是什么设备，需要想办法确定；
    }
    cable_sel_v = gpio_get_input(CABLE_SE);
#else
    gpio_status_display(gpio_status_value);
#endif
}





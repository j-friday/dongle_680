
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include "reg_blecore.h"
#include "app.h"
#include "llm_int.h"

#include "nvds.h"
#include "gpio.h"
#include "icu.h"
#include "cli_api.h"
#include "app_scan.h"
#include "app_adv.h"
#include "wdt.h"
#include "user_func.h"

#if CLI_CONSOLE
#include "cli.h"

static void help_cmd(char *buf, int len, int argc, char **argv)
{
    int i, n;
    uint32_t build_in_count = sizeof(built_ins) / sizeof(struct cli_command);

    aos_cli_printf( "====Build-in Commands====\r\n" );
    for (i = 0, n = 0; i < MAX_COMMANDS && n < cli->num_commands; i++) {
        if (cli->commands[i]->name) {
            aos_cli_printf("%s: %s\r\n", cli->commands[i]->name,
                           cli->commands[i]->help ?
                           cli->commands[i]->help : "");
            n++;
            if ( n == build_in_count - USER_COMMANDS_NUM ) {
                aos_cli_printf("\r\n");
                aos_cli_printf("====User Commands====\r\n");
            }
        }
    }
}

static void at_mode(char *buf, int len, int argc, char **argv)
{
    uint8_t mode;
    if(argc == 1)
    {
        aos_cli_printf("\r\n+ATMODE:%d\r\n", get_at_mode());
    }
    else if((argc == 2)&&(strlen(argv[1]) == 1))
    {
        mode = atoi(argv[1]);
        if(mode < 3)
        {
            set_at_mode(mode);
            if(mode != 1)cliexit = 1;
            aos_cli_printf("\r\nOK\r\n");
        }
        else goto EXIT;
    }
    return;
EXIT:
    aos_cli_printf("\r\nERROR\r\n");  
}

static void data_mode(char *buf, int len, int argc, char **argv)
{
    uint8_t mode;
    if(argc == 1)
    {        
        aos_cli_printf("\r\n+DATAMODE:%d\r\n", get_data_mode());
    }
    else if((argc == 2)&&(strlen(argv[1]) == 1))
    {
        mode = atoi(argv[1]);
        if(mode < 2)
        {
            set_data_mode(mode);
            aos_cli_printf("\r\nOK\r\n");
        }
        else goto EXIT;
    }
    return;
EXIT:
    aos_cli_printf("\r\nERROR\r\n");     
}

static void version_cmd(char *buf, int len, int argc, char **argv)
{    
    if(argc == 1)
    {
        uint8_t ver[7];
        uint8_t time[16];
        memset(ver, 0x00, 7);
        memset(time, 0x00, 16);
        get_fw_version(ver);
        get_version_time(time);
        aos_cli_printf("\r\n+VERSION:%s,%s\r\n", ver, time);
    }
    else if((argc == 3) && (strlen(argv[1]) == 6) && (strlen(argv[2]) == 15))
    {        
        set_fw_version((uint8_t *)argv[1]);
        set_version_time((uint8_t *)argv[2]);
        user_info_need_save();
        aos_cli_printf("\r\nOK\r\n");
    }
    else
    {
        aos_cli_printf("\r\nERROR\r\n");
    }
}

static void sn_cmd(char *buf, int len, int argc, char **argv)
{
    uint8_t sn[11];
    memset(sn, 0x00, 11);
    if(argc == 1)
    {
        get_sn(sn);
        aos_cli_printf("\r\n+SN:%s\r\n", sn);
    }
    else if((argc == 2)&&(strlen(argv[1]) == 10))
    {
        set_sn((uint8_t *)argv[1]);
        user_info_need_save();
        aos_cli_printf("\r\nOK\r\n");
    }
    else
    {
        aos_cli_printf("\r\nERROR\r\n");
    }
}

static void uuid_cmd(char *buf, int len, int argc, char **argv)
{
    uint8_t uuid[14];
    memset(uuid, 0x00, 14);
    if(argc == 1)
    {        
        get_uuid(uuid);
        aos_cli_printf("\r\n+UUID:%s\r\n", uuid);
    }
    else if((argc == 2)&&(strlen(argv[1]) == 13))
    {
        //hexstr2bin(argv[1], (uint8_t *)&uuid, 2, 1);
        set_uuid((uint8_t *)argv[1]);
        user_info_need_save();
        aos_cli_printf("\r\nOK\r\n");
    }
    else
    {
        aos_cli_printf("\r\nERROR\r\n");
    }
}

static void dev_name_cmd(char *buf, int len, int argc, char **argv)
{
    uint8_t name[32];
    memset(name, 0x00, 32);
    if(argc == 1)
    {
        get_product_name(name);
        aos_cli_printf("\r\n+DEVNAME:%s\r\n", name);
    }
    else if((argc == 2) && (strlen(argv[1]) <= 31))
    {
        set_product_name((uint8_t *)argv[1], strlen(argv[1]));
        user_info_need_save();
        aos_cli_printf("\r\nOK\r\n");
    }
    else
    {
        aos_cli_printf("\r\nERROR\r\n");
    }
}

static void reboot_cmd(char *buf, int len, int argc, char **argv)
{
	aos_cli_printf("\r\nOK\r\n");
	soft_reset();
    while(1);
}
//status, time
static void wdt_cmd(char *buf, int len, int argc, char **argv)
{
    uint8_t wdt_en;
    uint16_t time;
    
    if(argc == 1)
    {
        aos_cli_printf("\r\n+WDT:%d, 0x%04X\r\n", get_wdt_state(), get_wdt_time());
    }
    else if(argc == 3) 
    {
        wdt_en = atoi(argv[1]);
        
        set_wdt_state(wdt_en);
        
        hexstr2bin(argv[2], (uint8_t *)&time, 4, 0);
        set_wdt_time(time);
        user_info_need_save();
        aos_cli_printf("\r\nOK\r\n");
    }
    else
    {
        aos_cli_printf("\r\nERROR\r\n");
    }
}

static void sleep_cmd(char *buf, int len, int argc, char **argv)
{
    if(argc == 1)
    {
        aos_cli_printf("\r\n+SLEEP:%d\r\n", get_dev_mode());
    }
    else if(argc == 2)
    {
        set_dev_mode(atoi(argv[1]));
        user_info_need_save();
        aos_cli_printf("\r\nOK\r\n");
    }
    else
    {
        aos_cli_printf("\r\nERROR\r\n");
    }
}

#if !USB_DRIVER
static void adc_en_cmd(char *buf, int len, int argc, char **argv)
{
    uint8_t en_flag;
    if(argc == 1)
    {
        aos_cli_printf("\r\n+ADCEN:%s CH1:%s CH2:%s\r\n", get_adc_func()?"ON":"OFF", 
        (get_adc_en_status()&0x01)?"ON":"OFF", (get_adc_en_status()&0x02)?"ON":"OFF");
    }
    else if(argc == 2)
    {
        hexstr2bin(argv[1], (uint8_t *)&en_flag, 2, 0);
        if((en_flag & 0x7C) == 0)//~0x83
        {
            bk_printf("en_flag:%02x\r\n", en_flag);
            set_adc_func(en_flag);
            //user_info_need_save();
            aos_cli_printf("\r\nOK\r\n");
        }
        else aos_cli_printf("\r\nERROR\r\n");
    }
    else
    {
        aos_cli_printf("\r\nERROR\r\n");
    }
}

//threshold 1 2 3 4
//interval
//count
//AT+ADCPARM=500,1000,40,900,8000,100
static void adc_parm_cmd(char *buf, int len, int argc, char **argv)
{
    uint16_t threshold[2];
    if(argc == 1)
    {
        get_adc_threshold(0, threshold);
        aos_cli_printf("\r\n+ADCPARM:%d %d ", threshold[0], threshold[1]);
        get_adc_threshold(1, threshold);
        aos_cli_printf("%d %d %d %d\r\n", threshold[0], threshold[1], get_adc_interval(), get_adc_count());
    }
    else if(argc == 7)
    {
        threshold[0] = atoi(argv[1]);
        threshold[1] = atoi(argv[2]);
        //need Check validity
        set_adc_threshold(0 , threshold);//
        
        threshold[0] = atoi(argv[3]);
        threshold[1] = atoi(argv[4]);
        //need Check validity
        set_adc_threshold(1 , threshold);//        
        
        //need Check validity
        set_adc_interval(atoi(argv[5]));
        
        //need Check validity
        set_adc_count(atoi(argv[6]));
        
        user_info_need_save();
        aos_cli_printf("\r\nOK\r\n");
    }
    else
    {
        aos_cli_printf("\r\nERROR\r\n");
    }
    
}

static void adc_val_cmd(char *buf, int len, int argc, char **argv)
{
    uint8_t ch;
    if(argc == 2)
    {
        ch = atoi(argv[1]);
        if((ch > 0) && (ch < 3))
        {
            aos_cli_printf("\r\n+ADCVAL:%d\r\n", get_adc_val(ch));
        }
    }
    else
    {
        aos_cli_printf("\r\nERROR\r\n");
    }
}
#endif

#if USB_DRIVER
//gpio_pin, value
static void gpio_cmd(char *buf, int len, int argc, char **argv)
{
    uint8_t pin;
    uint8_t val;
    if(argc == 2)
    {
        hexstr2bin(argv[1], (uint8_t *)&pin, 2, 0);
        if(pin > 0x37)
        {
            goto EXIT;
        }
        aos_cli_printf("\r\n+GPIO:%02x\r\n",*((volatile unsigned long *) (BASEADDR_GPIO + 4 * (((pin >> 4) & 0x0F) * 8 + pin & 0x0F))));
    }
    else if(argc == 3)    
    {
        hexstr2bin(argv[1], (uint8_t *)&pin, 2, 0);        
        val = argv[2][0] - 0x30;
        if((pin > 0x37) || (val > 1))
        {
            goto EXIT;
        }
        set_gpio_value(pin, val);
        aos_cli_printf("\r\nOK\r\n");
    }
    return;
EXIT:
    aos_cli_printf("\r\nERROR\r\n"); 
}

static void led_cmd(char *buf, int len, int argc, char **argv)
{
    uint8_t pin;
    uint8_t mode;
    
    if(argc == 2)
    {
        hexstr2bin(argv[1], (uint8_t *)&pin, 2, 0);
        aos_cli_printf("\r\n+LED:%d\r\n", get_gpio_led());
    }
    else if(argc == 3)
    {
        hexstr2bin(argv[1], (uint8_t *)&pin, 2, 0);
        mode = argv[2][0] - 0x30;
        if((pin > 0x37) || (mode > 2))
        {
            goto EXIT;
        }
        set_gpio_led(pin, mode);
        aos_cli_printf("\r\nOK\r\n");
        
    }
    return;
EXIT:
    aos_cli_printf("\r\nERROR\r\n");      
}
#endif
static void bufs_cmd(char *buf, int len, int argc, char **argv)
{
    uint16_t size;
    if(argc == 1)
    {
        aos_cli_printf("\r\n+BLEBUFS:%d\r\n", get_ble_buf_size());
    }
    else if(argc == 2)
    {
        size = atoi(argv[1]);
        if(size < 20480)
        {
            set_ble_buf_size(size);
        }
        else goto EXIT;
    }
    else goto EXIT;
    aos_cli_printf("\r\nOK\r\n");
    return;
EXIT:
    aos_cli_printf("\r\nERROR\r\n");    
}

static void name_cmd(char *buf, int len, int argc, char **argv)
{	
    uint8_t name[APP_DEVICE_NAME_MAX_LEN];
    uint8_t name_len;
	if(argc > 2)
	{
		goto EXIT;
	}
    memset(name, 0x00, APP_DEVICE_NAME_MAX_LEN);
    if(argc == 2)
    {
        //aos_cli_printf("strlen(argv[1] :%d\r\nOK\r\n",strlen(argv[1]));
        memcpy(name, argv[1], strlen(argv[1]));
        name_len = strlen(argv[1]);
        if(set_ble_name(name_len, name))
        {
            goto EXIT;
        }   
        
        user_info_need_save();
    }
	else
    {
        get_ble_name(name);        
    }
    aos_cli_printf("\r\n+NAME=%s\r\nOK\r\n", name);	
	return;
    
EXIT:
    aos_cli_printf("\r\nERROR\r\n");    
}

static void ble_status_cmd(char *buf, int len, int argc, char **argv)
{
    if (argc == 1)
    {
        aos_cli_printf("\r\n%s\r\n", get_ble_state()?"CONNECTED":"DISCONNECTED");
    }
    else aos_cli_printf("\r\nERROR\r\n");   
}

static void mac_cmd(char *buf, int len, int argc, char **argv)
{
    uint8_t mac[6];
    bk_printf("%s argc %d\r\n",__func__,argc);
    if (argc == 1)
    {    
        get_ble_mac(mac);
        aos_cli_printf("\r\n+MAC: %02X%02X%02X%02X%02X%02X\r\n",
            mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
    }
    else if(argc == 2)
    { 
        hexstr2bin(argv[1], mac, 6, 0);
        reverse((uint8_t *)mac, 6);
        if(set_ble_mac(mac) != APPM_ERROR_NO_ERROR) 
        {
            goto EXIT;
        }
        user_info_need_save();
    }
    else
    {      
        goto EXIT;
    }
    aos_cli_printf("\r\nOK\r\n");
	return;
EXIT:
    aos_cli_printf("\r\nERROR\r\n");
}

static void power_cmd(char *buf, int len, int argc, char **argv)
{
    uint8_t level;
    if (argc == 1)
    {
        aos_cli_printf("\r\n+POWER:%X\r\n", get_ble_power());
    }
    else if(argc == 2)
    {
        for(uint8_t i = 0; i < strlen(argv[1]); i++)
        {
            if((argv[1][i] > 0x39) || (argv[1][i] < 0x30))
            {
                goto EXIT;
            }
        }
        level = atoi(argv[1]);
        if(level <= 0x0F)
        {
            set_ble_power(level);
            user_info_need_save();
        }
        else
        {
            goto EXIT;
        }
    }
    aos_cli_printf("\r\nOK\r\n");
	return;
EXIT:
    aos_cli_printf("\r\nERROR\r\n");    
}

static void rssi_cmd(char *buf, int len, int argc, char **argv)
{
    if (argc == 1)
    {
        aos_cli_printf("\r\n+RSSI:%ddBm\r\n", get_ble_rssi());
    }
    else
    {      
        aos_cli_printf("\r\nERROR\r\n");//goto EXIT;
    }
    
}

static void pincode_cmd(char *buf, int len, int argc, char **argv)
{
    uint8_t pincode[16];
    memset(pincode, 0x00, 16);
    if(argc == 1)
    {
        get_ble_pincode(pincode);
        aos_cli_printf("\r\n+PINCODE:%s\r\n", pincode);
    }
    else if((argc == 2) && (strlen(argv[1]) <= 16))
    {
        
        set_ble_pincode(strlen(argv[1]), (uint8_t *)argv[1]);
        user_info_need_save();
        aos_cli_printf("\r\nOK\r\n");
    }
    else
    {
        aos_cli_printf("\r\nERROR\r\n");
    }
}

static void remove_bond_cmd(char *buf, int len, int argc, char **argv)
{
    if(argc == 1)
    {
        user_remove_bond();
        aos_cli_printf("\r\nOK\r\n");
    }
    else
    {
        aos_cli_printf("\r\nERROR\r\n");
    }
}

#if !USB_DRIVER
static void auto_conn_cmd(char *buf, int len, int argc, char **argv)
{
    uint8_t auto_en;
    if (argc == 1)
    {
        aos_cli_printf("\r\n+AUTOCONN:%d\r\n", get_ble_auto_connect());
    }
    else if(argc == 2)
    {
        auto_en = atoi(argv[1]);
        if(auto_en < 2)
        {
            set_ble_auto_connect(auto_en);
            aos_cli_printf("\r\nOK\r\n");
        }
        else aos_cli_printf("\r\nERROR\r\n");
    }
}
//set_auto_connect_parm
static void auto_parm_cmd(char *buf, int len, int argc, char **argv)
{
    struct gap_bdaddr bdaddr;
    uint32_t duration_time;
    if(argc == 1)
    {
        duration_time = get_auto_connect_parm(bdaddr.addr.addr);
        aos_cli_printf("\r\n+AUTOPARM:%02X%02X%02X%02X%02X%02X,%d", bdaddr.addr.addr[5], bdaddr.addr.addr[4], bdaddr.addr.addr[3], bdaddr.addr.addr[2], 
                bdaddr.addr.addr[1], bdaddr.addr.addr[0], duration_time);
    }
    else if ((argc == 3) && (strlen(argv[1]) == 12))
    {    
        hexstr2bin(argv[1], bdaddr.addr.addr, GAP_BD_ADDR_LEN, 0);
        reverse((uint8_t *)bdaddr.addr.addr, 6);  
        bdaddr.addr_type = (bdaddr.addr.addr[5] & 0xC0) ? ADDR_RAND : ADDR_PUBLIC;
        bk_printf("con address: %02X%02X%02X%02X%02X%02X addr_type%d\r\n",
                bdaddr.addr.addr[0], bdaddr.addr.addr[1], bdaddr.addr.addr[2], bdaddr.addr.addr[3], bdaddr.addr.addr[4], bdaddr.addr.addr[5], bdaddr.addr_type);
        duration_time = atoi(argv[2]);
        set_auto_connect_parm(bdaddr.addr.addr, duration_time);//set_connect_start(bdaddr);
        user_info_need_save();
        aos_cli_printf("\r\n+GATTSTAT=%d,2\r\n",free_channel_search());
    }
    else goto EXIT;
	return;
EXIT:
    aos_cli_printf("\r\nERROR\r\n");    
}

static void connect(char *buf, int len, int argc, char **argv)
{      
    struct gap_bdaddr bdaddr;
    
    if ((argc != 2) || (strlen(argv[1]) != 12))goto EXIT;
    
    hexstr2bin(argv[1], bdaddr.addr.addr, GAP_BD_ADDR_LEN, 0);
    reverse((uint8_t *)bdaddr.addr.addr, 6);  
	bdaddr.addr_type = (bdaddr.addr.addr[5] & 0xC0) ? ADDR_RAND : ADDR_PUBLIC;
    bk_printf("con address: %02x-%02x-%02x-%02x-%02x-%02x addr_type%d\r\n",
            bdaddr.addr.addr[0], bdaddr.addr.addr[1], bdaddr.addr.addr[2], bdaddr.addr.addr[3], bdaddr.addr.addr[4], bdaddr.addr.addr[5], bdaddr.addr_type);
    set_connect_start(bdaddr);
    
    aos_cli_printf("\r\n+GATTSTAT=%d,2\r\n",free_channel_search());
 
	return;
EXIT:
    aos_cli_printf("\r\nERROR\r\n");
}
#include "ke_timer.h"
static void scan_cmd(char *buf, int len, int argc, char **argv)
{    
    if (argc == 1)
    {       uint8_t on_off = 0;
            if(app_env.scan_state == APP_SCAN_STATE_STARTED) 
            {
                on_off = 1;
            }
            bk_printf("Usage: Scan on/off. Scan is currently %s\r\n",
                       on_off ? "On" : "Off");
    }else if(argc == 2)
    {
        if (!strcmp(argv[1], "1")) 
        {
            if(app_env.scan_state == APP_SCAN_STATE_STARTED)
            {
                bk_printf("scan currently is on\r\n");
            }else
            {
                bk_printf("scan set on\r\n");
                bk_printf("scan set on cur_state:%d\r\n",app_env.scan_state);
                appm_update_scan_state(1);
                if (!ke_timer_active(APPM_SCAN_TIMEOUT_TIMER, TASK_APP))
                {
                    ke_timer_set(APPM_SCAN_TIMEOUT_TIMER, TASK_APP,500);            
                }
            }       
        } 
        else if (!strcmp(argv[1], "0"))
        {
            if(app_env.scan_state != APP_SCAN_STATE_STARTED)
            {
                bk_printf("scan currently is off\r\n");
            }else
            {           
                appm_update_scan_state(1);
                bk_printf("scan set off cur_state:%d\r\n",app_env.scan_state);
            }         
        }
    
    }else
    {
        goto EXIT;
    }
   
	return;
EXIT:
		aos_cli_printf("\r\nERROR\r\n");		
}
#endif

static void disconn_cmd(char *buf, int len, int argc, char **argv)
{
	int8_t device_id;
	uint8_t mac[6];
    
	if((argc != 2)||((strlen(argv[1]) != 1)&&(strlen(argv[1]) != 12)))
	{
		//bk_printf("argv[1]len:%x\r\n",strlen(argv[1]));
		goto EXIT;
	}   
	if(strlen(argv[1]) == 1)
	{
		device_id  = atoi(argv[1]);///-'0';
	}
	else
	{
		hexstr2bin(argv[1], mac, 6, 0);
        reverse((uint8_t *)mac, 6);
		device_id = channel_search(mac);
		if(device_id == -1)goto EXIT;
	}

	//add disconnect ble device function
	appm_disconnect(device_id);
    
	aos_cli_printf("\r\nOK\r\n");
	
	return;
EXIT:
	aos_cli_printf("\r\nERROR\r\n");

}

static void ble_parm_cmd(char *buf, int len, int argc, char **argv)
{
    ble_parm_t parm;
    uint16_t conn_intv;
    uint16_t conn_latency;
    uint16_t conn_sup_to;

    if(argc == 1)
    {
        get_ble_parm(&parm);
        aos_cli_printf("\r\n+BLEPARM:%d,%d,%d\r\n", parm.con_interval, parm.latency, parm.timeout);
    }
    else if(argc == 4)
    {
        
        conn_intv = atoi(argv[1]);
        conn_latency = atoi(argv[2]);
        conn_sup_to = atoi(argv[3]);

        bk_printf("conn_intv:%d,conn_latency:%d,conn_sup_to:%d\r\n",conn_intv ,conn_latency,conn_sup_to);
        if((conn_intv < CON_INTERVAL_MIN )|| (conn_intv > CON_INTERVAL_MAX )) //Allowed range is 7.5ms to 4s.
        {
            goto EXIT;
        }
        if((conn_latency > CON_LATENCY_MAX ))
        {
            goto EXIT;
        }
        if((conn_sup_to < CON_SUP_TO_MIN )|| (conn_sup_to > CON_SUP_TO_MAX ))//Allowed range is 100ms to 32s
        {
            goto EXIT;
        }
        if((conn_sup_to * 10) < (((1 + conn_latency) * conn_intv * 5 + 1) >> 1))
        {
            goto EXIT;
        }
        
        if(set_ble_parm(conn_intv, conn_latency, conn_sup_to))
        {        
            goto EXIT;
        }
        user_info_need_save();
        aos_cli_printf("\r\nOK\r\n");
    }
    return;
EXIT:
	aos_cli_printf("\r\nERROR\r\n");       
}

static void connected_info_cmd(char *buf, int len, int argc, char **argv)
{	
    if(argc == 1)
    {
        app_connect_device_info_get();	 
    }
    else
        aos_cli_printf("\r\nERROR\r\n");      
}
//AT+BDBAUD=0,115200
static void baud_cmd(char *buf, int len, int argc, char **argv)
{
    uint8_t uart_id;
	uint32_t baudrate=0;
	
	if(argc == 2)
	{
        uart_id = argv[1][0] - 0x30;
        if(uart_id > 1)
        {
            goto EXIT;
        }
        aos_cli_printf("\r\n+BAUD=%d\r\n", get_baudrate(uart_id));
		
	}
    else if(argc == 3)
    {
        uart_id = argv[1][0] - 0x30;
        baudrate = atoi(argv[2]); 
        
        bk_printf("baudrate=%d\r\n",baudrate);
        if((uart_id > 1) || baudrate_check(baudrate))// || (set_baudrate(uart_id, baudrate) != APPM_ERROR_NO_ERROR)))
        {		
            goto EXIT;
        }        
        aos_cli_printf("\r\n+BAUD=%s\r\n\r\nOK\r\n",argv[1]);
        user_info_need_save();
        set_baudrate(uart_id, baudrate);
    }
    else
    {
        goto EXIT;
    }
	return;
EXIT:
	aos_cli_printf("\r\nERROR\r\n");	
}

static void adv_en(char *buf, int len, int argc, char **argv)
{       
    if (argc == 1)
    {       
        uint8_t on_off = 0;
        if(app_env.adv_state == APP_ADV_STATE_STARTED)
        {
            on_off = 1;
        }
        bk_printf("Usage: Adv on/off. Adv is currently %s\r\n",
            on_off ? "On" : "Off");
        goto EXIT;
    }
    else if(argc == 2)
    {
        if ((!strcmp(argv[1], "on"))||(!strcmp(argv[1], "ON"))) 
        {
            if(app_env.adv_state != APP_ADV_STATE_STARTED)
            {
                appm_update_adv_state(true);
            } 
        } 
        else if ((!strcmp(argv[1], "off"))||(!strcmp(argv[1], "OFF")))
        {
            if(app_env.adv_state == APP_ADV_STATE_STARTED)            
            {
                appm_update_adv_state(false);
            }     
        }
    }
    else goto EXIT;
    aos_cli_printf("\r\nOK\r\n");
	return;
EXIT:
    aos_cli_printf("\r\nERROR\r\n");
}

const struct cli_command built_ins[BUILD_INS_NUM] = {
	
    {"AT+HELP","     help",              help_cmd},
    
    {"AT+ATMODE","   get/set at mode",   at_mode},
    {"AT+DATAMODE"," get/set data mode", data_mode},
        
    {"AT+VERSION","  get/set ver",       version_cmd},
    {"AT+SN","       get/set SN",        sn_cmd},
    {"AT+UUID","     get/set UUID",      uuid_cmd},
    {"AT+DEVNAME","  get/set name",      dev_name_cmd},
    
    {"AT+REBOOT","   reboot system",     reboot_cmd},
    {"AT+WDT","      get/set wdt",       wdt_cmd},
    {"AT+SLEEP","    get/set sleep mode",sleep_cmd},
    
#if !USB_DRIVER    
    {"AT+ADCEN","    set adc on/off",    adc_en_cmd},//on/off
    {"AT+ADCPARM","  get/set adc parm",  adc_parm_cmd},//thld11,thld12,thld21,thld22,interval,count
    {"AT+ADCVAL","   get adc value ",    adc_val_cmd},//chn
#endif
#if USB_DRIVER    
    {"AT+GPIO","     ctrl gpio value",   gpio_cmd},//pin,value
    {"AT+LED","      set led state",     led_cmd},
#endif 
    {"AT+BLEBUFS","  get/set ble b size",bufs_cmd},
    {"AT+BLENAME","  get/set ble name",  name_cmd},
    {"AT+BLESTATU"," get/set ",          ble_status_cmd},
    {"AT+MAC","      get/set mac",       mac_cmd},    
    {"AT+POWER","    get/set power",     power_cmd},
    {"AT+RSSI","     get/set rssi",      rssi_cmd},
    {"AT+PINCODE","  get/set pincode",   pincode_cmd},
    {"AT+REBOND","   remove bond",       remove_bond_cmd},
#if !USB_DRIVER    
    {"AT+AUTOCONN"," get/set auto conn", auto_conn_cmd},
    {"AT+AUTOPARM"," get/set auto parm", auto_parm_cmd},
    {"AT+CONN","     connect device",    connect},
    {"AT+SCAN","     scan device",       scan_cmd},
#endif    
    {"AT+DISC","     disconnect device", disconn_cmd},
    {"AT+BLEPARM","  get/set ble parm",  ble_parm_cmd},//interval, latecy, timeout
    {"AT+CONNINFO"," connected dev info",connected_info_cmd},
    
    {"AT+BDBAUD","   get/set baudrate",  baud_cmd},
        
    {"AT+ADVEN","    adv on/off",        adv_en},    
   
};

#endif

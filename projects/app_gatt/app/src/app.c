/**
 ****************************************************************************************
 *
 * @file app.c
 *
 * @brief Application entry point
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"             // SW configuration

#if (BLE_APP_PRESENT)
#include "rwapp_config.h"
#include <string.h>
#include "reg_blecore.h"
#include "rwip.h"
#include "app_task.h"                // Application task Definition
#include "app.h"                    
#include "app_init.h"
#include "app_scan.h"
#include "app_adv.h"
#include "gap.h"                     // GAP Definition
#include "gapm_task.h"               // GAP Manager Task API
#include "gapc_task.h"               // GAP Controller Task API
#include "gattc_int.h"
#include "co_bt.h"                   // Common BT Definition
#include "co_math.h"                 // Common Maths Definition

#if (BLE_APP_SEC)
#include "app_sec.h"                 // Application security Definition
#endif // (BLE_APP_SEC)

#if (BLE_APP_FEE0S)
#include "app_fee0.h"                  // USER define FEE0S Application Definitions
#endif //(BLE_APP_FEE0S)

#if (BLE_APP_FCC0S)
#include "app_fcc0.h"                  // USER define FCC0S Application Definitions
#endif //(BLE_APP_FCC0S)

#if (BLE_APP_HT)
#include "app_ht.h"                  // Health Thermometer Application Definitions
#endif //(BLE_APP_HT)

#if (BLE_APP_DIS)
#include "app_dis.h"                 // Device Information Service Application Definitions
#endif //(BLE_APP_DIS)

#if (BLE_APP_BATT)
#include "app_batt.h"                // Battery Application Definitions
#endif //(BLE_APP_DIS)

#if (BLE_APP_HID)
#include "app_hid.h"                 // HID Application Definitions
#endif //(BLE_APP_HID)

#if (DISPLAY_SUPPORT)
#include "app_display.h"             // Application Display Definition
#endif //(DISPLAY_SUPPORT)

#if (BLE_APP_AM0)
#include "app_am0.h"                 // Audio Mode 0 Application
#endif //(BLE_APP_AM0)
#if(BLE_APP_OADS)
    #include "app_oads.h"                 // Application oads Definition
#endif
#if (NVDS_SUPPORT)
#include "nvds.h"                    // NVDS Definitions
#endif //(NVDS_SUPPORT)
#include "user_func.h"
#include "sdp_service.h"
#include "uart.h"
/*
 * DEFINES
 ****************************************************************************************
 */





/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

typedef void (*appm_add_svc_func_t)(void);

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// List of service to add in the database


/*
 * LOCAL VARIABLES DEFINITIONS
 ****************************************************************************************
 */

/// Application Task Descriptor
extern const struct ke_task_desc TASK_DESC_APP;

/// List of functions used to create the database
static const appm_add_svc_func_t appm_add_svc_func_list[APPM_SVC_LIST_STOP] =
{
    #if (BLE_APP_HT)
    (appm_add_svc_func_t)app_ht_add_hts,
    #endif //(BLE_APP_HT)
    #if (BLE_APP_DIS)
    (appm_add_svc_func_t)app_dis_add_dis,
    #endif //(BLE_APP_DIS)
    #if (BLE_APP_BATT)
    (appm_add_svc_func_t)app_batt_add_bas,
    #endif //(BLE_APP_BATT)
    #if (BLE_APP_HID)
    (appm_add_svc_func_t)app_hid_add_hids,
    #endif //(BLE_APP_HID)
    #if (BLE_APP_AM0)
    (appm_add_svc_func_t)app_am0_add_has,
    #endif //(BLE_APP_AM0)
    #if (BLE_APP_FEE0S)
    (appm_add_svc_func_t)app_fee0_add_fee0s,
    #endif //(BLE_APP_FEE0S)    
    #if (BLE_APP_FCC0S)
    (appm_add_svc_func_t)app_fcc0_add_fcc0s,
    #endif //(BLE_APP_FCC0S)     
    #if (BLE_APP_OADS)
    (appm_add_svc_func_t)app_oad_add_oads,
    #endif
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Application Environment Structure
struct app_env_tag app_env;
#if !USB_DRIVER
struct appm_env_tag appm_env;
#endif
/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */



static void appm_send_gapm_reset_cmd(void)
{
    // Reset the stack
    struct gapm_reset_cmd *p_cmd = KE_MSG_ALLOC(GAPM_RESET_CMD,
                                                TASK_GAPM, TASK_APP,
                                                gapm_reset_cmd);

    p_cmd->operation = GAPM_RESET;

    ke_msg_send(p_cmd);
    
    bk_printf("appm_send_gapm_reset_cmd\r\n");
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void appm_init()
{
    // Reset the application manager environment
    memset(&app_env, 0, sizeof(app_env));

    // Create APP task
    ke_task_create(TASK_APP, &TASK_DESC_APP);

    // Initialize Task state
    ke_state_set(TASK_APP, APPM_INIT);

    // Get the Device Name to add in the Advertising Data (Default one or NVDS one)
    #if (NVDS_SUPPORT)
    app_env.dev_name_len = APP_DEVICE_NAME_MAX_LEN;
    if (rwip_param.get(PARAM_ID_DEVICE_NAME, &(app_env.dev_name_len), app_env.dev_name) != NVDS_OK)
    #endif //(NVDS_SUPPORT)
    {
        // Get default Device Name (No name if not enough space)
        #if 1
        app_env.dev_name_len = get_ble_name(app_env.dev_name);        
        #else
        memcpy(app_env.dev_name, APP_DFLT_DEVICE_NAME, APP_DFLT_DEVICE_NAME_LEN);
        app_env.dev_name_len = APP_DFLT_DEVICE_NAME_LEN;
        #endif
        // TODO update this value per profiles
    }
    app_env.dev_name_len = strlen((const char *)app_env.dev_name);

    /*------------------------------------------------------
     * INITIALIZE ALL MODULES
     *------------------------------------------------------*/

    // load device information:

    #if (DISPLAY_SUPPORT)
    app_display_init();
    #endif //(DISPLAY_SUPPORT)

    #if (BLE_APP_SEC)
    // Security Module
    app_sec_init();
    #endif // (BLE_APP_SEC)

    #if (BLE_APP_HT)
    // Health Thermometer Module
    app_ht_init();
    #endif //(BLE_APP_HT)

    #if (BLE_APP_DIS)
    // Device Information Module
    app_dis_init();
    #endif //(BLE_APP_DIS)

    #if (BLE_APP_HID)
    // HID Module
    app_hid_init();
    #endif //(BLE_APP_HID)

    #if (BLE_APP_BATT)
    // Battery Module
    app_batt_init();
    #endif //(BLE_APP_BATT)

    #if (BLE_APP_AM0)
    // Audio Mode 0 Module
    app_am0_init();
    #endif //(BLE_APP_AM0)
    #if (BLE_APP_FEE0S)
    app_fee0_init();
    #endif
    #if (BLE_APP_FCC0S)
    app_fcc0_init();
    #endif
    #if (BLE_APP_OADS)
    app_oads_init();
    #endif
    
    sdp_service_init();	
    // Reset the stack
    appm_send_gapm_reset_cmd();
}

bool appm_add_svc(void)
{
    // Indicate if more services need to be added in the database
    bool more_svc = false;

    // Check if another should be added in the database
    if (app_env.next_svc != APPM_SVC_LIST_STOP)
    {
        ASSERT_INFO(appm_add_svc_func_list[app_env.next_svc] != NULL, app_env.next_svc, 1);

        // Call the function used to add the required service
        appm_add_svc_func_list[app_env.next_svc]();

        // Select following service to add
        app_env.next_svc++;
        more_svc = true;
    }
    bk_printf("%s more_svc:%d\r\n",__func__,more_svc);
    return more_svc;
}

void appm_disconnect(uint8_t conidx)
{
    struct gapc_disconnect_cmd *cmd = KE_MSG_ALLOC(GAPC_DISCONNECT_CMD,
                                                   KE_BUILD_ID(TASK_GAPC, conidx), KE_BUILD_ID(TASK_APP, conidx),
                                                   gapc_disconnect_cmd);

    cmd->operation = GAPC_DISCONNECT;
    cmd->reason    = CO_ERROR_REMOTE_USER_TERM_CON;

    // Send the message
    ke_msg_send(cmd);
}
void rw_ip_sleep_test(uint32_t time);
void rw_ip_deep_sleep_test(void);



void appm_update_param(struct gapc_conn_param *conn_param)
{
    // Prepare the GAPC_PARAM_UPDATE_CMD message
    struct gapc_param_update_cmd *cmd = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_CMD,
                                                     KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                                                     gapc_param_update_cmd);

    cmd->operation  = GAPC_UPDATE_PARAMS;
    cmd->intv_min   = conn_param->intv_min;
    cmd->intv_max   = conn_param->intv_max;
    cmd->latency    = conn_param->latency;
    cmd->time_out   = conn_param->time_out;

    // not used by a slave device
    cmd->ce_len_min = 0xFFFF;
    cmd->ce_len_max = 0xFFFF;

    // Send the message
    ke_msg_send(cmd);
}

void appc_le_data_length_update_req(uint8_t conidx,uint16_t max_tx_octets,uint16_t max_tx_time)
{
    
    if((ke_state_get(KE_BUILD_ID(TASK_APP,conidx)) == APPC_LINK_CONNECTED) ||(ke_state_get(KE_BUILD_ID(TASK_APP,conidx)) == APPC_SDP_DISCOVERING)\
                ||(ke_state_get(KE_BUILD_ID(TASK_APP,conidx)) == APPC_SERVICE_CONNECTED) )
    {

        // Prepare the GAPC_SET_LE_PKT_SIZE_CMD message
        struct gapc_set_le_pkt_size_cmd *cmd = KE_MSG_ALLOC(GAPC_SET_LE_PKT_SIZE_CMD,
                                                     KE_BUILD_ID(TASK_GAPC, conidx),KE_BUILD_ID(TASK_APP,conidx) , //KE_BUILD_ID(TASK_APPM,conidx)
                                                     gapc_set_le_pkt_size_cmd);

        cmd->operation  = GAPC_SET_LE_PKT_SIZE;
        cmd->tx_octets   = max_tx_octets;
        cmd->tx_time   = max_tx_time;

        //UART_PRINTF("appc_le_data_length_update_req tx_octets = %d,tx_time = %d\r\n", max_tx_octets,max_tx_time);

        // Send the message
        ke_msg_send(cmd);
    }else
    {
        //UART_PRINTF("%s can't pro. cur state :%d\r\n",__func__,ke_state_get(KE_BUILD_ID(TASK_APP,conidx)));
    }
}

void appc_gatt_mtu_change(uint8_t conidx)
{
    if((ke_state_get(KE_BUILD_ID(TASK_APP,conidx)) == APPC_LINK_CONNECTED) ||(ke_state_get(KE_BUILD_ID(TASK_APP,conidx)) == APPC_SDP_DISCOVERING)\
                ||(ke_state_get(KE_BUILD_ID(TASK_APP,conidx)) == APPC_SERVICE_CONNECTED) )
    {
        struct gattc_exc_mtu_cmd *cmd = KE_MSG_ALLOC(GATTC_EXC_MTU_CMD,
                                    KE_BUILD_ID(TASK_GATTC, conidx), KE_BUILD_ID(TASK_APP, conidx),  
                                    gattc_exc_mtu_cmd);

        cmd->operation = GATTC_MTU_EXCH;

        cmd->seq_num = 0;
        
        ke_msg_send(cmd);

    }else
    {
        bk_printf("%s can't pro. cur state :%d\r\n",ke_state_get(KE_BUILD_ID(TASK_APP,conidx)));
    }
}
#endif //(BLE_APP_PRESENT)

/// @} APP

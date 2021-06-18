/**
 ****************************************************************************************
 *
 * @file appm_task.c
 *
 * @brief RW APP Task implementation
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APPTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"          // SW configuration


#if (BLE_APP_PRESENT)

#include "rwapp_config.h"
#include "app_task.h"             // Application Manager Task API
#include "app.h"                  // Application Manager Definition
#include "app_init.h"
#include "app_scan.h"
#include "app_adv.h"
#include "gapc_task.h"            // GAP Controller Task API
#include "gapm_task.h"            // GAP Manager Task API
#include "arch.h"                 // Platform Definitions
#include <string.h>
#include "co_utils.h"
#include "ke_timer.h"             // Kernel timer

#if (BLE_APP_FEE0S)
#include "app_fee0.h"              //  Module Definition
#endif //(BLE_APP_FEE0S)

#if (BLE_APP_FCC0S)
#include "app_fcc0.h"              //  Module Definition
#endif //(BLE_APP_FCC0S)
  
#if (BLE_APP_SEC)
#include "app_sec.h"              // Security Module Definition
#endif //(BLE_APP_SEC)

#if (BLE_APP_HT)
#include "app_ht.h"               // Health Thermometer Module Definition
#include "htpt_task.h"
#endif //(BLE_APP_HT)

#if (BLE_APP_DIS)
#include "app_dis.h"              // Device Information Module Definition
#include "diss_task.h"
#endif //(BLE_APP_DIS)

#if (BLE_APP_BATT)
#include "app_batt.h"             // Battery Module Definition
#include "bass_task.h"
#endif //(BLE_APP_BATT)

#if (BLE_APP_HID)
#include "app_hid.h"              // HID Module Definition
#include "hogpd_task.h"
#endif //(BLE_APP_HID)

#if (BLE_APP_AM0)
#include "app_am0.h"             // Audio Mode 0 Application
#endif //(BLE_APP_AM0)

#if (DISPLAY_SUPPORT)
#include "app_display.h"          // Application Display Definition
#endif //(DISPLAY_SUPPORT)

#include "cli_api.h"

#if (BLE_APP_OADS)
    #include "app_oads.h"
    #include "oads_task.h"
#endif
#include "gpio.h"
#include "user_func.h"
/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
#define  GAPM_OPERATION_NUM 55 
#define  GAPC_OPERATION_NUM 30 

uint8_t unknow_str[] = {"UNKNOW OPERATION"};
typedef struct key_str
{
    uint8_t key;
    unsigned char str[50];
}key_str_t;
const key_str_t gapm_operation_str[GAPM_OPERATION_NUM];
const key_str_t gapc_operation_str[GAPC_OPERATION_NUM];

uint8_t *gapm_operation_key2str(uint8_t value)
{
    
    for(int i = 0;i < GAPM_OPERATION_NUM;i++)
    {
        if(gapm_operation_str[i].key == value)
        {
            return (uint8_t *)gapm_operation_str[i].str;
        }
    }
    return unknow_str;
}
uint8_t *gapc_operation_key2str(uint8_t value)
{
    
    for(int i = 0;i < GAPC_OPERATION_NUM;i++)
    {
        if(gapc_operation_str[i].key == value)
        {
            return (uint8_t *)gapc_operation_str[i].str;
        }
    }
    return unknow_str;
}
const key_str_t gapm_operation_str[GAPM_OPERATION_NUM]=
{
    /* No Operation (if nothing has been requested)     */
    /* ************************************************ */
    /// No operation.
    {0x0,"GAPM_NO_OP"},
    

    /* Default operations                               */
    /* ************************************************ */
    /// Reset BLE subsystem: LL and HL.
    {0x01,"GAPM_RESET"},

    /* Configuration operations                         */
    /* ************************************************ */
    /// Set device configuration
    {0x03, "GAPM_SET_DEV_CONFIG"},
    /// Set device channel map
    {0x04,"GAPM_SET_CHANNEL_MAP"},

    /* Retrieve device information                      */
    /* ************************************************ */
    /// Get Local device version
    {0x05,"GAPM_GET_DEV_VERSION"},
    /// Get Local device BD Address
    {0x06,"GAPM_GET_DEV_BDADDR"},
    /// Get device advertising power level
    {0x07,"GAPM_GET_DEV_ADV_TX_POWER"},
    /// Get White List Size.
    {0x08,"GAPM_GET_WLIST_SIZE"},
    /// Retrieve Antenna information
    {0x09,"GAPM_GET_ANTENNA_INFO"},

    /* Security / Encryption Toolbox                    */
    /* ************************************************ */
    /// Resolve device address
    {0x17,"GAPM_RESOLV_ADDR"},
    /// Generate a random address
    {0x18,"GAPM_GEN_RAND_ADDR"},
    /// Use the controller's AES-128 block
    {0x19,"GAPM_USE_ENC_BLOCK"}, 
    /// Generate a 8-byte random number
    {0x1A,"GAPM_GEN_RAND_NB"},

    /* Profile Management                               */
    /* ************************************************ */
    /// Create new task for specific profile
    {0x1B,"GAPM_PROFILE_TASK_ADD"},
    /* DEBUG                                            */
    /* ************************************************ */
    /// Get memory usage
    {0x1C,"GAPM_DBG_GET_MEM_INFO"},
    /// Perform a platform reset
    {0x1D,"GAPM_PLF_RESET"}, 

    /* Data Length Extension                            */
    /* ************************************************ */
    /// Set Suggested Default LE Data Length
    {0x1E,"GAPM_SET_SUGGESTED_DFLT_LE_DATA_LEN"},
    /// Get Suggested Default LE Data Length
    {0x1F,"GAPM_GET_SUGGESTED_DFLT_LE_DATA_LEN"},
    /// Get Maximum LE Data Length
    {0x20,"GAPM_GET_MAX_LE_DATA_LEN"},

    /* Operation on Resolving List                      */
    /* ************************************************ */
    /// Get resolving address list size
    {0x21,"GAPM_GET_RAL_SIZE"},
    /// Get resolving local address
    {0x22,"GAPM_GET_RAL_LOC_ADDR"},
    /// Get resolving peer address
    {0x23,"GAPM_GET_RAL_PEER_ADDR"},

    /* Change current IRK                               */
    /* ************************************************ */
    /// Set IRK
    {0x28,"GAPM_SET_IRK"},

    /* LE Protocol/Service Multiplexer management       */
    /* ************************************************ */
    /// Register a LE Protocol/Service Multiplexer
    {0x29,"GAPM_LEPSM_REG"},
    /// Unregister a LE Protocol/Service Multiplexer
    {0x2A,"GAPM_LEPSM_UNREG"},

    /* LE Direct Test Mode                              */
    /* ************************************************ */
    /// Stop the test mode
    {0x2B,"GAPM_LE_TEST_STOP"},
    /// Start RX Test Mode
    {0x2C,"GAPM_LE_TEST_RX_START"}, 
    /// Start TX Test Mode
    {0x2D,"GAPM_LE_TEST_TX_START"},

    /* Secure Connection                                */
    /* ************************************************ */
    /// Generate DH_Key
    {0x2E,"GAPM_GEN_DH_KEY"},
    /// Retrieve Public Key
    {0x2F,"GAPM_GET_PUB_KEY"},

    /* List Management                                  */
    /* ************************************************ */
    /// Set content of white list
    {0x90,"GAPM_SET_WL"},
    /// Set content of resolving list
    {0x91,"GAPM_SET_RAL"},
    /// Set content of periodic advertiser list
    {0x92,"GAPM_SET_PAL"},
    /// Get periodic advertiser list size
    {0x95,"GAPM_GET_PAL_SIZE"},

    /* Air Operations                                   */
    /* ************************************************ */
    /// Create advertising activity
    {0xA0,"GAPM_CREATE_ADV_ACTIVITY"},
    /// Create scanning activity
    {0xA1,"GAPM_CREATE_SCAN_ACTIVITY"}, 
    /// Create initiating activity
    {0xA2,"GAPM_CREATE_INIT_ACTIVITY"}, 
    /// Create periodic synchronization activity
    {0xA3,"GAPM_CREATE_PERIOD_SYNC_ACTIVITY"}, 
    /// Start an activity
    {0xA4,"GAPM_START_ACTIVITY"}, 
    /// Stop an activity
    {0xA5,"GAPM_STOP_ACTIVITY"}, 
    /// Stop all activities
    {0xA6,"GAPM_STOP_ALL_ACTIVITIES"},
    /// Delete an activity
    {0xA7,"GAPM_DELETE_ACTIVITY"},
    /// Delete all activities
    {0xA8,"GAPM_DELETE_ALL_ACTIVITIES"}, 
    /// Set advertising data
    {0xA9,"GAPM_SET_ADV_DATA"}, 
    /// Set scan response data
    {0xAA,"GAPM_SET_SCAN_RSP_DATA"},
    /// Set periodic advertising data
    {0xAB,"GAPM_SET_PERIOD_ADV_DATA"},
    /// Get number of available advertising sets
    {0xAC,"GAPM_GET_NB_ADV_SETS"},
    /// Get maximum advertising data length supported by the controller
    {0xAD,"GAPM_GET_MAX_LE_ADV_DATA_LEN"},
    /// Get minimum and maximum transmit powers supported by the controller
    {0xAE,"GAPM_GET_DEV_TX_PWR"},
    /// Get the RF Path Compensation values used in the TX Power Level and RSSI calculation
    {0xAF,"GAPM_GET_DEV_RF_PATH_COMP"},
    /// Enable/Disable reception of periodic advertising report
    {0xB0,"GAPM_PER_ADV_REPORT_CTRL"}, 
    /// Enable / Disable IQ sampling
    {0xB1,"GAPM_PER_SYNC_IQ_SAMPLING_CTRL"}, 
    /// Enable / Disable CTE transmission
    {0xB2,"GAPM_PER_ADV_CTE_TX_CTL"},

    /* Debug Commands                                   */
    /* ************************************************ */
    /// Configure the Debug Platform I&Q Sampling generator
    {0x50,"GAPM_DBG_IQGEN_CFG"},

    /* Internal Operations                              */
    /* ************************************************ */
    /// Renew random addresses
    {0xF0,"GAPM_RENEW_ADDR"},
};
const key_str_t gapc_operation_str[GAPC_OPERATION_NUM]= 
{
    /*                 Operation Flags                  */
    /* No Operation (if nothing has been requested)     */
    /* ************************************************ */
    /// No operation
    {0x00,"GAPC_NO_OP"},

    /* Connection management */
    /// Disconnect link
    {0x01,"GAPC_DISCONNECT"},

    /* Connection information */
    /// Retrieve name of peer device.
    {0x02,"GAPC_GET_PEER_NAME"},
    /// Retrieve peer device version info.
    {0x03,"GAPC_GET_PEER_VERSION"},
    /// Retrieve peer device features.
    {0x04,"GAPC_GET_PEER_FEATURES"},
    /// Get Peer device appearance
    {0x05,"GAPC_GET_PEER_APPEARANCE"},
    /// Get Peer device Slaved Preferred Parameters
    {0x06,"GAPC_GET_PEER_SLV_PREF_PARAMS"},
    /// Retrieve connection RSSI.
    {0x07,"GAPC_GET_CON_RSSI"},
    /// Retrieve Connection Channel MAP.
    {0x08,"GAPC_GET_CON_CHANNEL_MAP"},

    /* Connection parameters update */
    /// Perform update of connection parameters.
    {0x09,"GAPC_UPDATE_PARAMS"},

    /* Security procedures */
    /// Start bonding procedure.
    {0x0A,"GAPC_BOND"},
    /// Start encryption procedure.
    {0x0B,"GAPC_ENCRYPT"},
    /// Start security request procedure
    {0x0C,"GAPC_SECURITY_REQ"},

    /* LE Ping*/
    /// get timer timeout value
    {0x12,"GAPC_GET_LE_PING_TO"},
    /// set timer timeout value
    {0x13,"GAPC_SET_LE_PING_TO"},

    /* LE Data Length extension*/
    /// LE Set Data Length
    {0x14,"GAPC_SET_LE_PKT_SIZE"},

    /* Central Address resolution supported*/
    {0x15,"GAPC_GET_ADDR_RESOL_SUPP"}, 

    /* Secure Connections */
    /// Request to inform the remote device when keys have been entered or erased
    {0x16,"GAPC_KEY_PRESS_NOTIFICATION"},

    /* PHY Management */
    /// Set the PHY configuration for current active link
    {0x17,"GAPC_SET_PHY"},
    /// Retrieve PHY configuration of active link
    {0x18,"GAPC_GET_PHY"},

    /* Channel Selection Algorithm */
    /// Retrieve Channel Selection Algorithm
    {0x19,"GAPC_GET_CHAN_SEL_ALGO"},

    /* Preferred slave latency */
    /// Set the preferred slave latency (for slave only, with RW controller)
    {0x1A,"GAPC_SET_PREF_SLAVE_LATENCY"},
    /// Set the preferred slave event duration (for slave only, with RW controller)
    {0x1B,"GAPC_SET_PREF_SLAVE_EVT_DUR"},

    /* Periodic Sync Transfer */
    /// Transfer periodic advertising sync information to peer device
    {0x1C,"GAPC_PER_ADV_SYNC_TRANS"},

    /* Constant Tone Extension */
    /// Constant Tone Extension Transmission configuration
    {0x20,"GAPC_CTE_TX_CFG"},
    /// Constant Tone Extension Reception configuration
    {0x21,"GAPC_CTE_RX_CFG"},
    /// Constant Tone Extension request control (enable / disable)
    {0x22,"GAPC_CTE_REQ_CTRL"},
    /// Constant Tone Extension Response control (enable / disable)
    {0x23,"GAPC_CTE_RSP_CTRL"}, 

    // ---------------------- INTERNAL API ------------------------
    /* Packet signature */
    /// sign an attribute packet
    {0xF0,"GAPC_SIGN_PACKET"},
    /// Verify signature or an attribute packet
    {0xF1,"GAPC_SIGN_CHECK"},
};

static uint8_t app_get_handler(const struct app_subtask_handlers *handler_list_desc,
                               ke_msg_id_t msgid,
                               void *param,
                               ke_task_id_t src_id)
{
    // Counter
    uint8_t counter;

    // Get the message handler function by parsing the message table
    for (counter = handler_list_desc->msg_cnt; 0 < counter; counter--)
    {
        struct ke_msg_handler handler
                = (struct ke_msg_handler)(*(handler_list_desc->p_msg_handler_tab + counter - 1));

        if ((handler.id == msgid) ||
            (handler.id == KE_MSG_DEFAULT_HANDLER))
        {
            // If handler is NULL, message should not have been received in this state
            ASSERT_ERR(handler.func);

            return (uint8_t)(handler.func(msgid, param, TASK_APP, src_id));
        }
    }

    // If we are here no handler has been found, drop the message
    return (KE_MSG_CONSUMED);
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles GAPM_ACTIVITY_CREATED_IND event
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_activity_created_ind_handler(ke_msg_id_t const msgid,
                                             struct gapm_activity_created_ind const *p_param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id)
{
    bk_printf("%s\r\n",__func__);
    if ((app_env.adv_state == APP_ADV_STATE_CREATING) && (p_param->actv_type == GAPM_ACTV_TYPE_ADV))
    {
        // Store the advertising activity index
        app_env.adv_actv_idx = p_param->actv_idx;
        bk_printf("adv_actv_idx:%d,tx_pwr:%d\r\n",app_env.adv_actv_idx,p_param->tx_pwr);
    }else if((app_env.scan_state == APP_SCAN_STATE_CREATING) && (p_param->actv_type == GAPM_ACTV_TYPE_SCAN))
    {
        // Store the scaning activity index
        app_env.scan_actv_idx = p_param->actv_idx;
        app_env.scan_state = APP_SCAN_STATE_CREATED;
        bk_printf("scan_actv_idx:%d,scan_state:%d\r\n",app_env.scan_actv_idx,app_env.scan_state);
    }
    else if((app_env.init_state == APP_INIT_STATE_CREATING) && (p_param->actv_type == GAPM_ACTV_TYPE_INIT))
    {
        // Store the scaning activity index
        app_env.init_actv_idx = p_param->actv_idx;
        app_env.init_state = APP_INIT_STATE_CREATED;
        bk_printf("init_actv_idx:%d\r\n",app_env.init_actv_idx);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles GAPM_ACTIVITY_STOPPED_IND event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_activity_stopped_ind_handler(ke_msg_id_t const msgid,
                                             struct gapm_activity_stopped_ind const *p_param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id)
{
    bk_printf("%s,reason:%x\r\n",__func__,p_param->reason);
   
    if ((app_env.adv_state == APP_ADV_STATE_STARTED) && (p_param->actv_type == GAPM_ACTV_TYPE_ADV))
    {
        // Act as if activity had been stopped by the application
        app_env.adv_state = APP_ADV_STATE_STOPPING;

        // Perform next operation
        appm_adv_fsm_next();

    }
    else if ((app_env.adv_state == APP_ADV_STATE_WAITING_DELETE) && (p_param->actv_type == GAPM_ACTV_TYPE_ADV))
    {
        // Act as if activity had been stopped by the application
       // app_env.adv_state = APP_ADV_STATE_DELETEING;

        // Perform next operation
        appm_adv_fsm_next();

    }
	else if((app_env.scan_state == APP_SCAN_STATE_STARTED) && (p_param->actv_type == GAPM_ACTV_TYPE_SCAN))
    {
        {
            // Act as if activity had been stopped by the application
            app_env.scan_state = APP_SCAN_STATE_STOPPING;
            // Perform next operation
            appm_scan_fsm_next();
        }

        
    }
    else if(((app_env.init_state == APP_INIT_STATE_STOPPING) ||(app_env.init_state == APP_INIT_STATE_CONECTTING) )&& (p_param->actv_type == GAPM_ACTV_TYPE_INIT))
    {     
        {
            // Act as if activity had been stopped by the application            
            // Perform next operation
            appm_init_fsm_next();
            #if !USB_DRIVER                    
            if(ke_timer_active(APPM_SCAN_TIMEOUT_TIMER, TASK_APP))
            {
                ke_timer_clear(APPM_SCAN_TIMEOUT_TIMER, TASK_APP);
                ke_msg_send_basic(APPM_SCAN_TIMEOUT_TIMER, TASK_APP, TASK_APP);
            }
            #endif
        }      
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles GAPM_PROFILE_ADDED_IND event
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_profile_added_ind_handler(ke_msg_id_t const msgid,
                                          struct gapm_profile_added_ind *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{ 
    // Current State
    ke_state_t state = ke_state_get(dest_id);
    uint8_t conidx = KE_IDX_GET(dest_id);
    bk_printf("%s prf_task_id:%x,prf_task_nb:%d,start_hdl:%d\r\n",__func__,param->prf_task_id, param->prf_task_nb,param->start_hdl);
    if(param->role == PRF_CLIENT)
    {
        if(sdp_add_profiles_num_get(conidx) == 0)
        {
            sdp_enable_all_server_ntf_ind(conidx,1);
        }
    }
    if (state == APPM_CREATE_DB)
    {
        switch (param->prf_task_id)
        {
            #if (BLE_APP_AM0)
            case (TASK_ID_AM0_HAS):
            {
                app_am0_set_prf_task(param->prf_task_nb);
            } break;
            #endif //(BLE_APP_AM0)

            default: /* Nothing to do */ break;
        }
    }
    else
    {
        ASSERT_INFO(0, state, src_id);
    }

    return KE_MSG_CONSUMED;
}

/**
 ****************************************************************************************
 * @brief Handles GAP manager command complete events.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gapm_cmp_evt const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    #if (NVDS_SUPPORT)
    uint8_t key_len = KEY_LEN;
    #endif //(NVDS_SUPPORT)
    uint8_t conidx = KE_IDX_GET(dest_id);
    bk_printf("%s conidx:%d,operation:0x%x,%s,status:%x\r\n",__func__,conidx,param->operation,gapm_operation_key2str(param->operation),param->status);
    switch(param->operation)
    {
        // Reset completed
        case (GAPM_RESET)://0
        {

            if(param->status == GAP_ERR_NO_ERROR)
            {
                #if (NVDS_SUPPORT)
                nvds_tag_len_t len;
                #endif //(NVDS_SUPPORT)
                #if (BLE_APP_HID)
                app_hid_start_mouse();
                #endif //(BLE_APP_HID)

                // Set Device configuration
                struct gapm_set_dev_config_cmd* cmd = KE_MSG_ALLOC(GAPM_SET_DEV_CONFIG_CMD,
                                                                   TASK_GAPM, TASK_APP,
                                                                   gapm_set_dev_config_cmd);
                // Set the operation
                cmd->operation = GAPM_SET_DEV_CONFIG;
                // Set the device role - Peripheral / central
                cmd->role      = GAP_ROLE_ALL;//GAP_ROLE_PERIPHERAL;
                
                #if (BLE_APP_SEC_CON)
                // The Max MTU is increased to support the Public Key exchange
                // HOWEVER, with secure connections enabled you cannot sniff the 
                // LEAP and LEAS protocols
                cmd->max_mtu = 160;
                cmd->pairing_mode = GAPM_PAIRING_SEC_CON | GAPM_PAIRING_LEGACY;
                #else // !(BLE_APP_SEC_CON)
                // Do not support secure connections
                cmd->pairing_mode = GAPM_PAIRING_LEGACY;
                #endif //(BLE_APP_SEC_CON)
                
                // Set Data length parameters
                cmd->sugg_max_tx_octets = LE_MAX_OCTETS;//LE_MIN_OCTETS;//LE_MAX_OCTETS;
                cmd->sugg_max_tx_time   = LE_MAX_TIME;//LE_MIN_TIME;//LE_MAX_TIME;
                
                cmd->max_mtu = 527;//ATT_DEFAULT_MTU;
               
                #if (BLE_APP_HID)
                // Enable Slave Preferred Connection Parameters present 
                cmd->att_cfg = GAPM_MASK_ATT_SLV_PREF_CON_PAR_EN;
                #endif //(BLE_APP_HID)

                // Host privacy enabled by default
                cmd->privacy_cfg = 0;
                

                #if (NVDS_SUPPORT)
                if (rwip_param.get(PARAM_ID_BD_ADDRESS, &len, &cmd->addr.addr[0]) == PARAM_OK)
                {
                    // Check if address is a static random address
                    if ((cmd->addr.addr[5] & 0xC0) == 0xC0)
                    {
                        // Host privacy enabled by default
                        cmd->privacy_cfg |= GAPM_PRIV_CFG_PRIV_ADDR_BIT;
                    }
                }
				#if 1
                else
                {
                    memcpy(&cmd->addr.addr[0],&co_default_bdaddr.addr[0],BD_ADDR_LEN);
                    if ((cmd->addr.addr[5] & 0xC0) == 0xC0)
                    {
                        // Host privacy enabled by default
                        cmd->privacy_cfg |= GAPM_PRIV_CFG_PRIV_ADDR_BIT;
                    }
                }
				#endif
                #endif //(NVDS_SUPPORT)

                #if (BLE_APP_AM0)
                cmd->audio_cfg   = GAPM_MASK_AUDIO_AM0_SUP;
                #endif //(BLE_APP_AM0)


                #if (NVDS_SUPPORT)
                if ((app_sec_get_bond_status()==true) &&
                    (nvds_get(NVDS_TAG_LOC_IRK, &key_len, app_env.loc_irk) == NVDS_OK))
                {
                    memcpy(cmd->irk.key, app_env.loc_irk, 16);
                }
                else
                #endif //(NVDS_SUPPORT)
                {
                    memset((void *)&cmd->irk.key[0], 0x00, KEY_LEN);
                }
                // Send message
                ke_msg_send(cmd);
            }
            else
            {
                ASSERT_ERR(0);
            }
        }
        break;

        case (GAPM_PROFILE_TASK_ADD)://0x1b
        {          
            // Add the next requested service
            if (!appm_add_svc())
            {
                // Go to the ready state
              //  ke_state_set(KE_BUILD_ID(TASK_APP,conidx), APPM_READY);

                // No more service to add, start advertising
                appm_update_adv_state(true);
                
                #if (BLE_OBSERVER || BLE_CENTRAL)
                // No more service to add, start scaning
              //  appm_update_scan_state(true);
                #endif
                
                #if (BLE_CENTRAL)
             //   appm_update_init_state(true);
                #endif
                             
              //  app_pwm0_0_capature_init();
             //   app_pwm1_0_pwm_init();
            }
            
        }
        break;

        case (GAPM_GEN_RAND_NB) ://0x1a
        {
            if (app_env.rand_cnt == 1)
            {
                // Generate a second random number
                app_env.rand_cnt++;
                struct gapm_gen_rand_nb_cmd *cmd = KE_MSG_ALLOC(GAPM_GEN_RAND_NB_CMD,
                                                                TASK_GAPM, TASK_APP,
                                                                gapm_gen_rand_nb_cmd);
                cmd->operation = GAPM_GEN_RAND_NB;
                ke_msg_send(cmd);
            }
            else
            {
                struct gapm_set_irk_cmd *cmd = KE_MSG_ALLOC(GAPM_SET_IRK_CMD,
                                                        TASK_GAPM, TASK_APP,
                                                        gapm_set_irk_cmd);
                app_env.rand_cnt=0;
                ///  - GAPM_SET_IRK
                cmd->operation = GAPM_SET_IRK;
                memcpy(&cmd->irk.key[0], &app_env.loc_irk[0], KEY_LEN);
                ke_msg_send(cmd);
            }
        }
        break;

        case (GAPM_SET_IRK):
        {
            // ASSERT_INFO(param->status == GAP_ERR_NO_ERROR, param->operation, param->status);

            #if (BLE_APP_SEC)
            // If not Bonded already store the generated value in NVDS
            //if (app_sec_get_bond_status()==false)
            if (nvds_get(NVDS_TAG_LOC_IRK, &key_len, app_env.loc_irk) != NVDS_OK)
            {
                #if (NVDS_SUPPORT)
                if (nvds_put(NVDS_TAG_LOC_IRK, KEY_LEN, (uint8_t *)&app_env.loc_irk) != NVDS_OK)
                #endif //(NVDS_SUPPORT)
                {
                    ASSERT_INFO(0, 0, 0);
                }
            }
            #endif //(BLE_APP_SEC)
            app_env.rand_cnt = 0;
             
            // Go to the create db state
            ke_state_set(TASK_APP, APPC_CREATE_DB);

            // Add the first required service in the database
            // and wait for the PROFILE_ADDED_IND
            appm_add_svc();
                   
        }
        break;

        // Device Configuration updated
        case (GAPM_SET_DEV_CONFIG):
        {
            ASSERT_INFO(param->status == GAP_ERR_NO_ERROR, param->operation, param->status);

            #if (BLE_APP_SEC)
            if (app_sec_get_bond_status()==true) 
            {
                #if (NVDS_SUPPORT)
                // If Bonded retrieve the local IRK from NVDS
                if (nvds_get(NVDS_TAG_LOC_IRK, &key_len, app_env.loc_irk) == NVDS_OK)
                {
                    // Set the IRK in the GAP
                    struct gapm_set_irk_cmd *cmd = KE_MSG_ALLOC(GAPM_SET_IRK_CMD,
                                                                TASK_GAPM, TASK_APP,
                                                                gapm_set_irk_cmd);
                    ///  - GAPM_SET_IRK: 
                    cmd->operation = GAPM_SET_IRK;
                    memcpy(&cmd->irk.key[0], &app_env.loc_irk[0], KEY_LEN);
                    ke_msg_send(cmd);
                }
                else
                #endif //(NVDS_SUPPORT)
               
                {
                     // If cannot read IRK from NVDS ASSERT
                     ASSERT_ERR(0);
                }
            }
            else // Need to start the generation of new IRK
            #endif //(BLE_APP_SEC)
            {
                struct gapm_gen_rand_nb_cmd *cmd = KE_MSG_ALLOC(GAPM_GEN_RAND_NB_CMD,
                                                                TASK_GAPM, TASK_APP,
                                                                gapm_gen_rand_nb_cmd);
                cmd->operation   = GAPM_GEN_RAND_NB;
                app_env.rand_cnt = 1;
                ke_msg_send(cmd);
            }
           
        }
        break;
    #if (BLE_OBSERVER || BLE_CENTRAL)
        case (GAPM_CREATE_SCAN_ACTIVITY)://0xA1           
        {
            #if (BLE_OBSERVER || BLE_CENTRAL)
             if(param->status == GAP_ERR_NO_ERROR)
             {
                 appm_scan_fsm_next();
             }           
            #endif
        }break;
    #endif
        
    #if (BLE_CENTRAL )
        case (GAPM_CREATE_INIT_ACTIVITY)://0xA2
        {
            #if (BLE_CENTRAL)
            if(param->status == GAP_ERR_NO_ERROR)
            {
                 appm_init_fsm_next();
            }           
            #endif
        }break;
    #endif
     
        case (GAPM_CREATE_ADV_ACTIVITY)://0xA0              
        case (GAPM_SET_ADV_DATA)://0xA9
        case (GAPM_SET_SCAN_RSP_DATA)://0xAA
        {
            // Sanity checks
            ASSERT_INFO(app_env.adv_op == param->operation, app_env.adv_op, param->operation);
            ASSERT_INFO(param->status == GAP_ERR_NO_ERROR, param->status, app_env.adv_op);
            // Perform next operation
            if(param->status == GAP_ERR_NO_ERROR)
            {
                 appm_adv_fsm_next();   
            }  
  
        } break;
       
        case (GAPM_START_ACTIVITY)://0xA4
        {
            if(param->status == GAP_ERR_NO_ERROR)
            {
                if(app_env.adv_state == APP_ADV_STATE_STARTING)
                {
                    appm_adv_fsm_next(); 
                }
                #if (BLE_OBSERVER || BLE_CENTRAL)
                if(app_env.scan_state == APP_SCAN_STATE_STARTING)
                {
                    appm_scan_fsm_next();
                }
                #endif
                #if (BLE_CENTRAL)
                if(app_env.init_state == APP_INIT_STATE_WAIT_CONECTTING)
                {
                    appm_init_fsm_next(); 
                }
                #endif   
            }
            
        }break;
        case (GAPM_STOP_ACTIVITY)://0xA5
        {
            if(param->status == GAP_ERR_NO_ERROR)
            {
                if(app_env.adv_state == APP_ADV_STATE_STOPPING)
                {
                    appm_adv_fsm_next(); 
                }
                #if (BLE_OBSERVER || BLE_CENTRAL)
                if(app_env.scan_state == APP_SCAN_STATE_STOPPING)
                {
                    appm_scan_fsm_next(); 
                }
                #endif
            }
        }break;
        case (GAPM_DELETE_ACTIVITY)://0xA7
        {
            if(param->status == GAP_ERR_NO_ERROR)
            {
                if(app_env.adv_state == APP_ADV_STATE_DELETEING)
                {
                     appm_adv_fsm_next(); 
                
                }            
            }
        
        }break;
        

        case (GAPM_DELETE_ALL_ACTIVITIES) :
        {
            // Re-Invoke activty
            app_env.adv_state = APP_ADV_STATE_IDLE;
            #if (BLE_OBSERVER || BLE_CENTRAL )
            app_env.scan_state = APP_SCAN_STATE_IDLE;
            #endif
            
            #if (BLE_CENTRAL)
            app_env.init_state = APP_INIT_STATE_IDLE;
            #endif            

        } break;

        default:
        {
            // Drop the message
        }
        break;
    }

    return (KE_MSG_CONSUMED);
}

static int gapc_get_dev_info_req_ind_handler(ke_msg_id_t const msgid,
        struct gapc_get_dev_info_req_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    bk_printf("%s,req:0x%x\r\n",__func__,param->req);
    switch(param->req)
    {
        case GAPC_DEV_NAME:
        {
            struct gapc_get_dev_info_cfm * cfm = KE_MSG_ALLOC_DYN(GAPC_GET_DEV_INFO_CFM,
                                                    src_id, dest_id,
                                                    gapc_get_dev_info_cfm, APP_DEVICE_NAME_MAX_LEN);
            cfm->req = param->req;
            cfm->info.name.length = get_ble_name(cfm->info.name.value);//appm_get_dev_name

            bk_printf("length:%d,name:%s\r\n",cfm->info.name.length,cfm->info.name.value);
            // Send message
            ke_msg_send(cfm);
        } break;

        case GAPC_DEV_APPEARANCE:
        {
            // Allocate message
            struct gapc_get_dev_info_cfm *cfm = KE_MSG_ALLOC(GAPC_GET_DEV_INFO_CFM,
                                                             src_id, dest_id,
                                                             gapc_get_dev_info_cfm);
            cfm->req = param->req;
            // Set the device appearance
            #if (BLE_APP_HT)
            // Generic Thermometer - TODO: Use a flag
            cfm->info.appearance = 728;
            #elif (BLE_APP_HID)
            // HID Mouse
            cfm->info.appearance = 962;
            #else
            // No appearance
            cfm->info.appearance = 0;
            #endif

            // Send message
            ke_msg_send(cfm);
        } break;

        case GAPC_DEV_SLV_PREF_PARAMS:
        {
            // Allocate message
            struct gapc_get_dev_info_cfm *cfm = KE_MSG_ALLOC(GAPC_GET_DEV_INFO_CFM,
                    src_id, dest_id,
                                                            gapc_get_dev_info_cfm);
            cfm->req = param->req;
            // Slave preferred Connection interval Min
            cfm->info.slv_pref_params.con_intv_min = 6;
            // Slave preferred Connection interval Max
            cfm->info.slv_pref_params.con_intv_max = 10;
            // Slave preferred Connection latency
            cfm->info.slv_pref_params.slave_latency  = 0;
            // Slave preferred Link supervision timeout
            cfm->info.slv_pref_params.conn_timeout    = 300;  // 2s (500*10ms)

            // Send message
            ke_msg_send(cfm);
        } break;

        default: /* Do Nothing */ break;
    }


    return (KE_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles GAPC_SET_DEV_INFO_REQ_IND message.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_set_dev_info_req_ind_handler(ke_msg_id_t const msgid,
        struct gapc_set_dev_info_req_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    // Set Device configuration
    struct gapc_set_dev_info_cfm* cfm = KE_MSG_ALLOC(GAPC_SET_DEV_INFO_CFM, src_id, dest_id,
                                                     gapc_set_dev_info_cfm);
    // Reject to change parameters
    cfm->status = GAP_ERR_REJECTED;
    cfm->req = param->req;
    // Send message
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles connection complete event from the GAP. Enable all required profiles
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_connection_req_ind_handler(ke_msg_id_t const msgid,
                                           struct gapc_connection_req_ind const *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{
    app_env.conidx = KE_IDX_GET(src_id);
    bk_printf("%s conidx:%d,dest_id:0x%x\r\n",__func__,app_env.conidx,dest_id);
    
    // Check if the received Connection Handle was valid
    if (app_env.conidx != GAP_INVALID_CONIDX)
    {
        // Retrieve the connection info from the parameters
        app_env.conhdl = param->conhdl;
        
        /// Connection handle
        bk_printf("conhdl:%d \r\n",param->conhdl);
        bk_printf("con_interval:%.2f \r\n",param->con_interval * 1.25);
        bk_printf("con_latency:%d \r\n",param->con_latency );
        bk_printf("sup_to:%d \r\n",param->sup_to * 10 );
        bk_printf("peer_addr_type:%d \r\n",param->peer_addr_type);
        {
            bk_printf("peer_addr:0x");
            for(uint8_t i = 0;i < GAP_BD_ADDR_LEN;i++)
            {
                bk_printf("%02x",param->peer_addr.addr[i]);
                
            }bk_printf("\r\n");
        }
        app_env.con_dev_addr[app_env.conidx].addr_type = param->peer_addr_type;
        memcpy(app_env.con_dev_addr[app_env.conidx].addr.addr,param->peer_addr.addr,6);
        app_env.role[app_env.conidx] = param->role;
        bk_printf("role:%s \r\n",param->role ? "Slave" : "Master");
        // Send connection confirmation
        struct gapc_connection_cfm *cfm = KE_MSG_ALLOC(GAPC_CONNECTION_CFM,
                KE_BUILD_ID(TASK_GAPC, app_env.conidx), KE_BUILD_ID(TASK_APP,app_env.conidx),
                gapc_connection_cfm);

        #if(BLE_APP_SEC)
        cfm->auth      = app_sec_get_bond_status() ? GAP_AUTH_REQ_MITM_BOND : GAP_AUTH_REQ_NO_MITM_NO_BOND; // TODO [FBE] restore valid data
        #else // !(BLE_APP_SEC)
        cfm->auth      = GAP_AUTH_REQ_NO_MITM_NO_BOND;
        #endif // (BLE_APP_SEC)
        // Send the message
        ke_msg_send(cfm);
        
        if(param->role == ROLE_MASTER)
        {
            #if !USB_DRIVER
            dmo_channel = app_env.conidx;
            #endif
            //sdp_discover_all_service(app_env.conidx);
            
            //ke_timer_set(APP_CHANGE_MTU_SIZE_REQ, TASK_APP, 50);
           // app_sec_send_bond_cmd(app_env.conidx);
        }
        else
        {
            app_sec_send_security_req(app_env.conidx);
        }

        #if DISPLAY_SUPPORT
        // Update displayed information
        app_display_set_adv(false);
        app_display_set_con(true);
        #endif //(DISPLAY_SUPPORT)

        /*--------------------------------------------------------------
         * ENABLE REQUIRED PROFILES
         *--------------------------------------------------------------*/

        #if (BLE_APP_BATT)
        // Enable Battery Service
        app_batt_enable_prf(app_env.conidx);
        #endif //(BLE_APP_BATT)

        #if (BLE_APP_HID)
        // Enable HID Service
        app_hid_enable_prf(app_env.conhdl);
        #endif //(BLE_APP_HID)

        // We are now in connected State
        ke_state_set(KE_BUILD_ID(TASK_APP,app_env.conidx), APPC_LINK_CONNECTED);
        #if !USB_DRIVER
        set_gpio_status(CONNECT_STATUS_BIT, CONNECT_SUCCESS);
        #endif
        {
            uint8_t rsp_buff[20];
            uint8_t len;
            len = sprintf((char *)rsp_buff,"\r\n+GATTSTAT=%d,3\r\nOK\r\n", app_env.conidx);
            UART_SEND_AT(rsp_buff, len);
        }
        #if 1//(BLE_APP_SEC && !defined(BLE_APP_AM0))
        //if (app_sec_get_bond_status())
        {
            // Ask for the peer device to either start encryption
          //  app_sec_send_security_req(app_env.conidx);
        }
        #endif // (BLE_APP_SEC && !defined(BLE_APP_AM0))
        app_sec_env.bonded = false;
		app_sec_env.peer_pairing = false;
		app_sec_env.peer_encrypt = false;
        //phy_change_state=0;
        appm_update_adv_state(false);
        //ke_timer_set(APP_GATTC_EXC_MTU_CMD,TASK_APP,20);
    }
    else
    {
        // No connection has been established, restart advertising
        appm_update_adv_state(true);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles connection complete event from the GAP. Enable all required profiles
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_param_update_req_ind_handler(ke_msg_id_t const msgid,
                                           struct gapc_param_update_req_ind const *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{
    app_env.conidx = KE_IDX_GET(src_id);

    bk_printf("%s\r\n",__func__);
    // Check if the received Connection Handle was valid
    if (app_env.conidx != GAP_INVALID_CONIDX)
    {

        // Send connection confirmation
        struct gapc_param_update_cfm *cfm = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_CFM,
                KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                gapc_param_update_cfm);

        cfm->accept = true;
        cfm->ce_len_min = 0xffff;
        cfm->ce_len_max = 0xffff;

        // Send message
        ke_msg_send(cfm);

    }
    else
    {
        // No connection has been established, restart advertising
        appm_update_adv_state(true);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief  GAPC_PARAM_UPDATED_IND
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_param_updated_ind_handler (ke_msg_id_t const msgid, 
									const struct gapc_param_updated_ind  *param,
                 					ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(src_id);
    bk_printf("appc %s dest_id:0x%x,src_id:0x%x\r\n", __func__,dest_id,src_id);
    bk_printf("conidx:%#x\r\n",conidx);
	bk_printf("con_interval = %.2f ms\r\n",param->con_interval*1.25);
	bk_printf("con_latency = %d\r\n",param->con_latency);
	bk_printf("sup_to = %d\r\n",param->sup_to * 10);
	    
	return KE_MSG_CONSUMED;
}

/*******************************************************************************
 * Function: gapc_le_pkt_size_ind_handler
 * Description: GAPC_LE_PKT_SIZE_IND
 * Input: msgid   -Id of the message received.
 *		  param   -Pointer to the parameters of the message.
 *		  dest_id -ID of the receiving task instance
 *		  src_id  -ID of the sending task instance.
 * Return: If the message was consumed or not.
 * Others: void
*******************************************************************************/
static int gapc_le_pkt_size_ind_handler (ke_msg_id_t const msgid, 
									const struct gapc_le_pkt_size_ind  *param,
                 					ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(src_id);
   	bk_printf("%s msgid:0x%x,dest_id:0x%x,src_id:0x%x\r\n",__func__,msgid,dest_id,src_id);
    bk_printf("conidx:%x,",conidx);
	bk_printf("1max_rx_octets = %d\r\n",param->max_rx_octets);
	bk_printf("1max_rx_time = %d\r\n",param->max_rx_time);
	bk_printf("1max_tx_octets = %d\r\n",param->max_tx_octets);
	bk_printf("1max_tx_time = %d\r\n",param->max_tx_time);
	
	return KE_MSG_CONSUMED;
}

/**
 ****************************************************************************************
 * @brief Handles GAP controller command complete events.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gapc_cmp_evt const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(dest_id);
    bk_printf("%s conidx:%d,operation:0x%x,%s,status:%x\r\n",__func__,conidx,param->operation,gapc_operation_key2str(param->operation),param->status);
 
    switch(param->operation)
    {
        case (GAPC_UPDATE_PARAMS):
        {
            if (param->status != GAP_ERR_NO_ERROR)
            {
//                appm_disconnect();
            }
        } break;
        case (GAPC_SECURITY_REQ): //0x0c
		{
			if (param->status != GAP_ERR_NO_ERROR)
	        {
	            bk_printf("gapc security req fail !\r\n");
                appm_disconnect(conidx);
	        }
	        else
	        {
	            bk_printf("gapc security req ok !\r\n");
	        }
		}break;
		case (GAPC_BOND): // 0xa
    	{
	        if (param->status != GAP_ERR_NO_ERROR)
	        {
                appm_disconnect(conidx);
	            bk_printf("gapc bond fail !\r\n");
	        }
	        else
	        {
	            bk_printf("gapc bond ok !\r\n");
	        }
    	}break;
		case (GAPC_ENCRYPT): // 0xb
		{
			if (param->status != GAP_ERR_NO_ERROR)
			{
                appm_disconnect(conidx);
				bk_printf("gapc encrypt start fail !\r\n");
			}
			else
			{
				bk_printf("gapc encrypt start ok !\r\n");
			}
		}
		break;
        
        case (GAPC_GET_PEER_VERSION): // 0x3
		{
			if (param->status != GAP_ERR_NO_ERROR)
			{
				bk_printf("GAPC_GET_PEER_VERSION fail ! status:0x%x\r\n",param->status);
			}
			else
			{
				bk_printf("GAPC_GET_PEER_VERSION ok !\r\n");
			}
            //appc_get_peer_dev_info(conidx,GAPC_GET_PEER_FEATURES);
		}
		break;
        
         case (GAPC_GET_PEER_FEATURES): // 0x4
		{
			if (param->status != GAP_ERR_NO_ERROR)
			{
				bk_printf("GAPC_GET_PEER_FEATURES fail ! status:0x%x\r\n",param->status);
			}
			else
			{
				bk_printf("GAPC_GET_PEER_FEATURES ok !\r\n");
			}    

            app_sec_send_bond_cmd(conidx);  
            
		}
		break;
        case (GAPC_SET_LE_PKT_SIZE):
            if (param->status != GAP_ERR_NO_ERROR)
			{
				bk_printf("GAPC_SET_LE_PKT_SIZE fail ! status:0x%x\r\n",param->status);
			}
			else
			{
				bk_printf("GAPC_SET_LE_PKT_SIZE ok !\r\n");
			}
        break;
        case (GAPC_DISCONNECT):
        {
            appm_update_adv_state(true);
        }            
        default:
        {
        } break;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles disconnection complete event from the GAP.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_disconnect_ind_handler(ke_msg_id_t const msgid,
                                      struct gapc_disconnect_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(dest_id);
    bk_printf("%s,reason:0x%x\r\n",__func__,param->reason);
	set_ble_erroinfo(param->reason);
    // Go to the ready state
    ke_state_set(KE_BUILD_ID(TASK_APP, conidx), APPC_LINK_IDLE);

    #if (BLE_APP_HT)
    // Stop interval timer
    app_stop_timer();
    #endif //(BLE_APP_HT)

    #if (DISPLAY_SUPPORT)
    // Update Connection State screen
    app_display_set_con(false);
    #endif //(DISPLAY_SUPPORT)

    #if (BLE_ISO_MODE_0_PROTOCOL)
    app_env.adv_state = APP_ADV_STATE_CREATING;
    #endif //(BLE_ISO_MODE_0_PROTOCOL)

    #if (!BLE_APP_HID)
    // Restart Advertising
    appm_update_adv_state(true);
    #endif //(!BLE_APP_HID)
    #if !USB_DRIVER
    set_gpio_status(CONNECT_STATUS_BIT, CONNECT_ABNORMAL);
    #endif
    {
        uint8_t rsp_buff[20];
        uint8_t len;
        len = sprintf((char *)rsp_buff,"\r\n+GATTSTAT=%d,0\r\nOK\r\n", conidx);
        UART_SEND_AT(rsp_buff, len);
    }
    if(dmo_channel == conidx)dmo_channel = 0xFF;
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of all messages sent from the lower layers to the application
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int appm_msg_handler(ke_msg_id_t const msgid,
                            void *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id)
{
    // Retrieve identifier of the task from received message
    ke_task_id_t src_task_id = MSG_T(msgid);
    // Message policy
    uint8_t msg_pol = KE_MSG_CONSUMED;
    uint8_t found_flag = 1;
    switch (src_task_id)
    {
        case (TASK_ID_GAPC):
        {
            #if (BLE_APP_SEC)
            if ((msgid >= GAPC_BOND_CMD) &&
                (msgid <= GAPC_SECURITY_IND))
            {
                // Call the Security Module
                msg_pol = app_get_handler(&app_sec_handlers, msgid, param, src_id);
            }
            #endif //(BLE_APP_SEC)
            // else drop the message
        } break;

        case (TASK_ID_GATTC):
        {
            // Service Changed - Drop
        } break;

        #if (BLE_APP_FEE0S)
        case (TASK_ID_FEE0S):
        {
            // Call the app fee0s Module
            msg_pol = app_get_handler(&app_fee0_handler, msgid, param, src_id);
        } break;
        #endif //(BLE_APP_FEE0S)
            
        #if (BLE_APP_FCC0S)
        case (TASK_ID_FCC0S):
        {
            // Call the app fee0s Module
            msg_pol = app_get_handler(&app_fcc0_handler, msgid, param, src_id);
        } break;
        #endif //(BLE_APP_FCC0S)

        #if (BLE_APP_HT)
        case (TASK_ID_HTPT):
        {
            // Call the Health Thermometer Module
            msg_pol = app_get_handler(&app_ht_handlers, msgid, param, src_id);
        } break;
        #endif //(BLE_APP_HT)

        #if (BLE_APP_DIS)
        case (TASK_ID_DISS):
        {
            // Call the Device Information Module
            msg_pol = app_get_handler(&app_dis_handlers, msgid, param, src_id);
        } break;
        #endif //(BLE_APP_DIS)

        #if (BLE_APP_HID)
        case (TASK_ID_HOGPD):
        {
            // Call the HID Module
            msg_pol = app_get_handler(&app_hid_handlers, msgid, param, src_id);
        } break;
        #endif //(BLE_APP_HID)

        #if (BLE_APP_BATT)
        case (TASK_ID_BASS):
        {
            // Call the Battery Module
            msg_pol = app_get_handler(&app_batt_handlers, msgid, param, src_id);
        } break;
        #endif //(BLE_APP_BATT)

        #if (BLE_APP_AM0)
        case (TASK_ID_AM0):
        {
            // Call the Audio Mode 0 Module
            msg_pol = app_get_handler(&app_am0_handlers, msgid, param, src_id);
        } break;

        case (TASK_ID_AM0_HAS):
        {
            // Call the Audio Mode 0 Module
            msg_pol = app_get_handler(&app_am0_has_handlers, msgid, param, src_id);
        } break;
        #endif //(BLE_APP_AM0)
        
        #if(BLE_APP_OADS)
        case (TASK_ID_OADS):
        {
            // Call the Health Thermometer Module
            msg_pol = app_get_handler(&app_oads_handler, msgid, param, src_id);
        }
        break;
        #endif
        default:
        {
            #if (BLE_APP_HT)
            if (msgid == APP_HT_MEAS_INTV_TIMER)
            {
                msg_pol = app_get_handler(&app_ht_handlers, msgid, param, src_id);
            }
            #endif //(BLE_APP_HT)

            #if (BLE_APP_HID)
            if (msgid == APP_HID_MOUSE_TIMEOUT_TIMER)
            {
                msg_pol = app_get_handler(&app_hid_handlers, msgid, param, src_id);
            }
            #endif //(BLE_APP_HID)
            found_flag = 0;
        } break;
    }
    if(!found_flag)
    {
        bk_printf("%s,src_task_id:0x%x,dest_id:0x%x,src_id:0x%x,msgid:0x%x\r\n",__func__,src_task_id,dest_id,src_id,msgid);  
    }
 
    return (msg_pol);
}

/**
 ****************************************************************************************
 * @brief Handles reception of random number generated message
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_gen_rand_nb_ind_handler(ke_msg_id_t const msgid, struct gapm_gen_rand_nb_ind *param,
                                        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (app_env.rand_cnt==1)      // First part of IRK
    {
        memcpy(&app_env.loc_irk[0], &param->randnb.nb[0], 8);
    }
    else if (app_env.rand_cnt==2) // Second part of IRK
    {
        memcpy(&app_env.loc_irk[8], &param->randnb.nb[0], 8);
    }

    return KE_MSG_CONSUMED;
}
#if (BLE_OBSERVER || BLE_CENTRAL )

static int gapm_ext_adv_report_ind_handler(ke_msg_id_t const msgid, struct gapm_ext_adv_report_ind *param,
                                        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
#if !USB_DRIVER
    uint8_t filter =0;
    #if 0
    bk_printf("%s\r\n",__func__);
    bk_printf("actv_idx:%x\r\n",param->actv_idx);
    bk_printf("info:%x\r\n",param->info);
    bk_printf("ch:%d\r\n",param->channel_idx);
    bk_printf("rssi:%d\r\n",param->rssi);
    bk_printf("addr_type:%x\r\n",param->trans_addr.addr_type);
    bk_printf("addr");
    for(int i = 0;i < 6;i++)
    {
        bk_printf(":%02x",param->trans_addr.addr.addr[i]);
    }bk_printf("\r\n");
    
    bk_printf("data len:%d\r\n",param->length);
    bk_printf("data");
    for(int i = 0;i < param->length;i++)
    {
        bk_printf(":%02x",param->data[i]);
    }bk_printf("\r\n");
    #endif
    
    filter = appm_add_adv_report_to_filter(param);
    if(filter >= ADV_REPORT_DEV_NUM)//ADV_REPORT_DEV_NUM
    {
        //UART_PRINTF("adv_report_num:%d\r\n",filter);
        return KE_MSG_CONSUMED; 
    }
   
    if(filter == (ADV_REPORT_DEV_NUM - 1))
    {    
        if (ke_timer_active(APPM_SCAN_TIMEOUT_TIMER, TASK_APP))
        {
            ke_timer_set(APPM_SCAN_TIMEOUT_TIMER, TASK_APP,1);

        }  
       //return KE_MSG_CONSUMED; 
    }
#endif    
    //appm_adv_data_decode(param->length,param->data,NULL,0);
    
    //bk_printf("****************************************\r\n");
    
    return KE_MSG_CONSUMED;
}
#endif

#if (BLE_SDP_CLIENT)
uint16_t user_uuid = 0xfee0;
static int gattc_sdp_svc_ind_handler(ke_msg_id_t const msgid,
                                     struct gattc_sdp_svc_ind const *ind,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(src_id);
    bk_printf("appc %s msgid:0x%x,dest_id:0x%x,src_id:0x%x\r\n",__func__,msgid,dest_id,src_id);
   // struct prf_sdp_db_env *prf_db_env = NULL;
     
    bk_printf("ind uuid len:%d,uuid:",ind->uuid_len);
    for(int i = 0;i < ind->uuid_len;i++)
    {
        bk_printf("%02x ",ind->uuid[ind->uuid_len - i - 1]);
    }

    app_add_sdp_fee0_srv(conidx,2,ind);
   
   
    return (KE_MSG_CONSUMED);
}
static int app_gattc_event_ind_handler(ke_msg_id_t const msgid,
                                   struct gattc_event_ind const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    uint8_t conidx = KE_IDX_GET(src_id);
    bk_printf("gattc_event_ind conidx:%d\r\n",conidx);
    bk_printf("NOTIF RECIVE length:%d,value = \r\n",param->length);
    for(int i = 0; i< param->length; i++)
    {
        bk_printf("%02x ",param->value[i]);
    }bk_printf("\r\n");
    #if !USB_DRIVER
    if(memcmp("vaux_detech_value:1", param->value, 19) == 0)
    {
        set_gpio_status(ADAPTER_STATUS_BIT, ADAPTER_IN);
    }
	else if(memcmp("vaux_detech_value:0", param->value, 19) == 0)
	{
		set_gpio_status(ADAPTER_STATUS_BIT, ADAPTER_OUT);
	}
    #endif
   return (KE_MSG_CONSUMED);  
}

static int app_gattc_event_req_ind_handler(ke_msg_id_t const msgid,
                                       struct gattc_event_ind const *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    //uint8_t w_data[20];
    //UART_PRINTF("appm %s \r\n",__func__);
    //UART_PRINTF("type = 0x%x,length = 0x%x,handle = 0x%02x\r\n",param->type,param->length,param->handle);
/*    bk_printf("RECIVE value =  \r\n");
    for(int i = 0; i< param->length; i++)
    {
        bk_printf("%02x ",param->value[i]);
    }
    bk_printf("\r\n");
*/  
    UART_SEND_AT((void *)param->value, param->length);
   return (KE_MSG_CONSUMED);  
}
#endif

/**
 ****************************************************************************************
 * @brief  GATTC_MTU_CHANGED_IND
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int app_gattc_mtu_changed_ind_handler(ke_msg_id_t const msgid,
                                     struct gattc_mtu_changed_ind const *ind,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(src_id);
	bk_printf("%s msgid:0x%x,dest_id:0x%x,src_id:0x%x\r\n",__func__,msgid,dest_id,src_id);
    bk_printf("conidx:%x,",conidx);
	bk_printf("ind->mtu = %d,seq = %d\r\n",ind->mtu,ind->seq_num);
 	return (KE_MSG_CONSUMED);
}

static int gapm_scan_request_ind_handler(ke_msg_id_t const msgid,
                                 struct gapm_scan_request_ind const *p_ind,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    bk_printf("%s\r\n",__func__);
    bk_printf("actv_idx:%d,addr_type:%d\r\n",p_ind->actv_idx,p_ind->trans_addr.addr_type);
    bk_printf("addr ");
    for(int i = 0;i < 6;i++)
    {
        bk_printf(":%02x ",p_ind->trans_addr.addr.addr[i]);
    }bk_printf("\r\n");
    return (KE_MSG_CONSUMED);  
}


extern uint8_t send_data_enable_flag[BLE_CONNECTION_MAX];
static int gattc_cmp_evt_handler(ke_msg_id_t const msgid,
                                 struct gattc_cmp_evt const *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    
    uint8_t conidx = KE_IDX_GET(src_id);
    bk_printf("app %s dest_id = %x,conidx:%d\r\n",__func__,dest_id,conidx);
    bk_printf("operation = 0x%x,status = 0x%x,seq_num = 0x%x\r\n",param->operation,param->status,param->seq_num);
    
    if(((param->operation == GATTC_WRITE_NO_RESPONSE) || (param->operation == GATTC_WRITE)) && (param->seq_num != 0xa5))
	{ 	
        bk_printf("\r\n~~~~~~set send_data_enable_flag 1\r\n");

	}
    if((param->operation == GATTC_SDP_DISC_SVC_ALL))
	{
    	ke_state_set(KE_BUILD_ID(TASK_APP,conidx),APPC_SERVICE_CONNECTED);
        bk_printf("\r\nAPPC_SERVICE_CONNECTED\r\n");
        sdp_enable_all_server_ntf_ind(conidx,1);
	}
    
    if((param->operation == GATTC_WRITE) && (param->seq_num == 0xaa))
	{
    	
        bk_printf("\r\nGATTC_WRITE\r\n");
        sdp_enable_all_server_ntf_ind(conidx,0);
	}    
    	   
    return (KE_MSG_CONSUMED);
}

static int app_send_security_req_handler(ke_msg_id_t const msgid, 
										void const *param,
        								ke_task_id_t const dest_id, 
        								ke_task_id_t const src_id)
{   
	app_sec_send_security_req(app_env.conidx);
    
    return KE_MSG_CONSUMED;
}

static int app_user_change_mtu_size_handler(ke_msg_id_t const msgid,
                                     void const *ind,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(src_id);
    appc_le_data_length_update_req(conidx,BLE_MAX_OCTETS,ATT_MAX_VALUE);//appm_env.conidx
    appc_gatt_mtu_change(conidx);///appm_env.conidx
    return (KE_MSG_CONSUMED);
}

#if USB_DRIVER

static int app_led_mode_process_handler(ke_msg_id_t const msgid,
                                     void const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    uint8_t mode;
    static uint8_t flag = 0;
    
    mode = get_gpio_led();
    bk_printf("mode:%d, led:%d\r\n", mode, flag);
    if(mode < 2)
    {
        gpio_set(LED_G, mode);
    }
    else if(mode == 2)
    {
        gpio_set(LED_G, flag);
        flag ^= 1;
        bk_printf("led:%d\r\n", flag);
        ke_timer_set(APP_LED_MODE_PROCESS_HANDLER, TASK_APP, 20);
    }
    
    return (KE_MSG_CONSUMED);
}
#else
static int app_ble_auto_conn_handler(ke_msg_id_t const msgid,
                                     void const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    appm_stop_connencting();
    bk_printf("stop connect\r\n");
    return (KE_MSG_CONSUMED);
}

static int appm_scan_dev_timerout_handler(ke_msg_id_t const msgid,
                                          struct gapm_profile_added_ind *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
                                              
{
    if(app_env.scan_state == APP_SCAN_STATE_STARTED)
    {
        appm_update_scan_state(false);//stop scan
    }
	bk_printf("%s\r\n",__func__);

    appm_scan_adv_repor();
    appm_adv_report_filter_free();    
           
    return (KE_MSG_CONSUMED);
}
#endif
/*
 * GLOBAL VARIABLES DEFINITION
 ****************************************************************************************
 */

/* Default State handlers definition. */
KE_MSG_HANDLER_TAB(appm)
{
    // Note: first message is latest message checked by kernel so default is put on top.
    {KE_MSG_DEFAULT_HANDLER,    (ke_msg_func_t)appm_msg_handler},

    // GAPM messages
    {GAPM_PROFILE_ADDED_IND,    (ke_msg_func_t)gapm_profile_added_ind_handler},
    {GAPM_ACTIVITY_CREATED_IND, (ke_msg_func_t)gapm_activity_created_ind_handler},
    {GAPM_ACTIVITY_STOPPED_IND, (ke_msg_func_t)gapm_activity_stopped_ind_handler},
    {GAPM_CMP_EVT,              (ke_msg_func_t)gapm_cmp_evt_handler},
    {GAPM_GEN_RAND_NB_IND,      (ke_msg_func_t)gapm_gen_rand_nb_ind_handler},
    #if (BLE_BROADCASTER)    
    {GAPM_SCAN_REQUEST_IND,      (ke_msg_func_t)gapm_scan_request_ind_handler},
    #endif
    
    #if (BLE_OBSERVER || BLE_CENTRAL )
    {GAPM_EXT_ADV_REPORT_IND,   (ke_msg_func_t)gapm_ext_adv_report_ind_handler},
    #endif    

    // GAPC messages
    {GAPC_GET_DEV_INFO_REQ_IND, (ke_msg_func_t)gapc_get_dev_info_req_ind_handler},
    {GAPC_SET_DEV_INFO_REQ_IND, (ke_msg_func_t)gapc_set_dev_info_req_ind_handler},
    {GAPC_CONNECTION_REQ_IND,   (ke_msg_func_t)gapc_connection_req_ind_handler},
    {GAPC_PARAM_UPDATE_REQ_IND, (ke_msg_func_t)gapc_param_update_req_ind_handler},
    {GAPC_PARAM_UPDATED_IND,    (ke_msg_func_t)gapc_param_updated_ind_handler},
    {GAPC_LE_PKT_SIZE_IND,      (ke_msg_func_t)gapc_le_pkt_size_ind_handler},
    {GAPC_CMP_EVT,              (ke_msg_func_t)gapc_cmp_evt_handler},
    {GAPC_DISCONNECT_IND,       (ke_msg_func_t)gapc_disconnect_ind_handler},
     // GATTC messages
    #if (BLE_SDP_CLIENT)
    {GATTC_SDP_SVC_IND,         (ke_msg_func_t)gattc_sdp_svc_ind_handler},
    {GATTC_EVENT_IND,           (ke_msg_func_t)app_gattc_event_ind_handler},
    {GATTC_EVENT_REQ_IND,       (ke_msg_func_t)app_gattc_event_req_ind_handler},
    #endif
    {GATTC_MTU_CHANGED_IND,     (ke_msg_func_t)app_gattc_mtu_changed_ind_handler},
    {GATTC_CMP_EVT,             (ke_msg_func_t)gattc_cmp_evt_handler},
    {APP_SEND_SECURITY_REQ,     (ke_msg_func_t)app_send_security_req_handler},
    {APP_CHANGE_MTU_SIZE_REQ,   (ke_msg_func_t)app_user_change_mtu_size_handler},
    #if USB_DRIVER
    {APP_LED_MODE_PROCESS_HANDLER,	(ke_msg_func_t)app_led_mode_process_handler},
    #else
    {APP_BLE_AUTO_CONN_HANDLER, (ke_msg_func_t)app_ble_auto_conn_handler},
    {APPM_SCAN_TIMEOUT_TIMER,   (ke_msg_func_t)appm_scan_dev_timerout_handler},
    #endif
};

/* Defines the place holder for the states of all the task instances. */
ke_state_t appm_state[APP_IDX_MAX];

// Application task descriptor
const struct ke_task_desc TASK_DESC_APP = {appm_msg_handler_tab, appm_state, APP_IDX_MAX, ARRAY_LEN(appm_msg_handler_tab)};

#endif //(BLE_APP_PRESENT)

/// @} APPTASK

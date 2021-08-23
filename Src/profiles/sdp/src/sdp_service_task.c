/**
 ****************************************************************************************
 *
 * @file sdp_service_task.c
 *
 * @brief SDP Service Client Task implementation.
 *
 * Copyright (C) Beken 2016-2017
 *
 *
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @addtogroup SDPTASK
 * @{
 ****************************************************************************************
 */
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#include "rwprf_config.h"
#if (BLE_SDP_CLIENT)
#include "gap.h"
#include "attm.h"
#include "sdp_service.h"
#include "sdp_service_task.h"
#include "gattc_task.h"
#include "co_math.h"
#include "app.h"
#include "app_init.h"
#include "uart.h"
#include "cli_api.h"
//#include "user_profile.h"
//#include "app_698.h"
/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
/*
 * GLOBAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref BASC_ENABLE_REQ message.
 * The handler enables the NTF Service Client Role.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int sdp_enable_req_handler(ke_msg_id_t const msgid,
                                  struct sdp_enable_req const *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    // Status
    int msg_status = KE_MSG_CONSUMED;
    return (msg_status);
}
/**
 ****************************************************************************************
 * @brief Handles reception of the @ref SDP_READ_INFO_REQ message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int sdp_read_info_req_handler(ke_msg_id_t const msgid, struct sdp_read_info_req const *param,
                                     ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    int msg_status = KE_MSG_CONSUMED;
    bk_printf("func %s\r\n",__func__);
    uint8_t conidx = KE_IDX_GET(dest_id);
    uint8_t char_idx;
    uint16_t handle = param->handle;
    uint8_t desc_nb;
  
    struct sdp_env_tag *sdp_env = NULL;
    if(SDPC_OPERATE_HANDLE == param->type)
    {
        bk_printf("prf_read_char_send \r\n");
        sdp_env =(struct sdp_env_tag *) prf_env_get_from_handle(conidx,handle);
        if(sdp_env != NULL)
        {      
            prf_read_char_send(&(sdp_env->prf_env), conidx,
                                   handle, handle, handle);
        }   
    }else if(SDPC_OPERATE_UUID == param->type)
    {
    
        sdp_env =(struct sdp_env_tag *) prf_env_get_from_uuid(conidx,param->uuid_len,param->uuid);;
        if(sdp_env != NULL)
        {
            char_idx = sdp_env->prf_db_env->sdp_cont->char_idx;
            desc_nb = sdp_env->prf_db_env->sdp_cont->descs_nb;
            handle = sdp_env->prf_db_env->sdp_cont->chars_descs_inf.chars_inf[char_idx].val_hdl;
            uint16_t val_prop =  sdp_env->prf_db_env->sdp_cont->chars_descs_inf.chars_inf[char_idx].prop;
            bk_printf("desc_nb = 0x%x,char_idx = 0x%x,value_handle = 0x%x\r\n",desc_nb,char_idx,handle);
            uint16_t desc_uuid;
            if(SDPC_CHAR_NTF_CFG == param->info)
            {
                
                for(int i = 0; i < desc_nb; i++)
                {
                    desc_uuid = co_read16(sdp_env->prf_db_env->sdp_cont->chars_descs_inf.descs_inf[i].uuid);
                    if((ATT_DESC_CLIENT_CHAR_CFG == desc_uuid) && (sdp_env->prf_db_env->sdp_cont->chars_descs_inf.descs_inf[i].char_code == char_idx))
                    {
                        handle = sdp_env->prf_db_env->sdp_cont->chars_descs_inf.descs_inf[i].desc_hdl;
                        val_prop |= ATT_CHAR_PROP_RD;
                        break;
                    }
                }
            }
            else if(SDPC_CHAR_USER_DESC_VAL == param->info)
            {
                
                for(int i = 0; i < desc_nb; i++)
                {
                    desc_uuid = co_read16(sdp_env->prf_db_env->sdp_cont->chars_descs_inf.descs_inf[i].uuid);
                    if((ATT_DESC_CHAR_USER_DESCRIPTION == desc_uuid) && (sdp_env->prf_db_env->sdp_cont->chars_descs_inf.descs_inf[i].char_code == char_idx))
                    {
                        handle = sdp_env->prf_db_env->sdp_cont->chars_descs_inf.descs_inf[i].desc_hdl;;
                        val_prop |= ATT_CHAR_PROP_RD;
                        break;
                    }
                }
            }
            bk_printf("prop = 0x%x,char_idx = 0x%x\r\n",val_prop,char_idx);
            bk_printf("handle = 0x%x\r\n",handle);
            if((val_prop & ATT_CHAR_PROP_RD) == ATT_CHAR_PROP_RD ) // ATT_CHAR_PROP_RD
            {
                bk_printf("prf_read_char_send \r\n");
                prf_read_char_send(&(sdp_env->prf_env), conidx,
                                   handle, handle, handle);
            }
            else
            {
                bk_printf("val_prop not read prop!!!! \r\n");
            }
        }
        else
        {
            bk_printf("param unvalid uuid =  0x%02x\r\n",param->uuid);
        }
 
    }
    
    
    return (msg_status);
}
/**
 ****************************************************************************************
 * @brief Handles reception of the @ref BASC_BATT_LEVEL_NTF_CFG_REQ message.
 * It allows configuration of the peer ntf/stop characteristic for Battery Level Characteristic.
 * Will return an error code if that cfg char does not exist.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
 
static int sdp_write_value_info_req_handler(ke_msg_id_t const msgid,
        struct sdp_write_info_req const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)

{
    int msg_status = KE_MSG_CONSUMED;
    
    uint8_t conidx = param->conidx;
    uint8_t char_idx;
    struct sdp_env_tag *sdp_env =(struct sdp_env_tag *) prf_env_get_from_handle(conidx,param->handle);
    if(sdp_env != NULL)
    {
        char_idx = sdp_env->prf_db_env->sdp_cont->char_idx;
        uint16_t handle = sdp_env->prf_db_env->sdp_cont->chars_descs_inf.chars_inf[char_idx].val_hdl;
        uint16_t length = param->length;
        uint16_t val_prop =  sdp_env->prf_db_env->sdp_cont->chars_descs_inf.chars_inf[char_idx].prop;
        //  UART_PRINTF("prop = 0x%x\r\n",val_prop);
        uint8_t *buf = (uint8_t *)(&param->data[0]);
        uint8_t operation = GATTC_WRITE_NO_RESPONSE;
        //  UART_PRINTF("handle = 0x%x\r\n",handle);
        if((val_prop & ATT_CHAR_PROP_WR_NO_RESP) == ATT_CHAR_PROP_WR_NO_RESP)
        {
            operation = GATTC_WRITE_NO_RESPONSE;
        }
        //else if(((val_prop & ATT_CHAR_PROP_WR) == ATT_CHAR_PROP_WR)&& (operation == GATTC_WRITE)) // ATT_CHAR_PROP_WR
        else if((val_prop & ATT_CHAR_PROP_WR) == ATT_CHAR_PROP_WR)
        {
            operation = GATTC_WRITE;
        }
        else
        {
           bk_printf("param operation or val_prop error =  0x%02x,0x%02x\r\n",operation,val_prop);
           return (msg_status);
        }
        bk_printf("operation = %x\r\n",operation);
        prf_gatt_write(&(sdp_env->prf_env),conidx,handle, buf,length,operation,0);
    }
    else
    {
        bk_printf("param unvalid handle =  0x%02x\r\n",param->handle);
    }
    return (msg_status);
}
/**
 ****************************************************************************************
 * @brief Handles reception of the @ref BASC_BATT_LEVEL_NTF_CFG_REQ message.
 * It allows configuration of the peer ntf/stop characteristic for Battery Level Characteristic.
 * Will return an error code if that cfg char does not exist.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int sdp_write_ntf_cfg_req_handler(ke_msg_id_t const msgid,
        struct sdp_write_ntf_cfg_req const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    bk_printf("func %s\r\n",__func__);
    int msg_status = KE_MSG_CONSUMED;
    uint8_t conidx = param->conidx;
    uint8_t char_idx;
    // put value in air format
    bk_printf("param->handle = %x\r\n",param->handle);
    struct sdp_env_tag *sdp_env =(struct sdp_env_tag *) prf_env_get_from_handle(conidx,param->handle + 1);
    if(sdp_env != NULL)
    {
        char_idx = sdp_env->prf_db_env->sdp_cont->char_idx;
        uint16_t uuid = co_read16p(sdp_env->prf_db_env->sdp_cont->chars_descs_inf.descs_inf[char_idx].uuid);
        bk_printf("ntf handle = 0x%x\r\n",param->handle + 1);
        if(uuid == ATT_DESC_CLIENT_CHAR_CFG)
        {
            prf_gatt_write_ntf_ind(&(sdp_env->prf_env), conidx,  param->handle + 1, param->ntf_cfg,param->seq_num);
        }else
        {
            bk_printf("ntf_ind not cfg char :  0x%04x\r\n",uuid);
        }
               
    }
    else
    {
        bk_printf("param unvalid handle =  0x%02x\r\n",param->handle);
    }
    return (msg_status);
}
/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATTC_CMP_EVT message.
 * This generic event is received for different requests, so need to keep track.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

static int gattc_cmp_evt_handler(ke_msg_id_t const msgid,
                                 struct gattc_cmp_evt const *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    bk_printf("sdp %s dest_id = %x\r\n",__func__,dest_id);
    uint8_t conidx = KE_IDX_GET(src_id);
    bk_printf("sdp operation = 0x%x,status = 0x%x,seq_num = 0x%x\r\n",param->operation,param->status,param->seq_num);
    struct gattc_cmp_evt *cmp_evt  = KE_MSG_ALLOC(GATTC_CMP_EVT,KE_BUILD_ID(TASK_APP, conidx), dest_id, gattc_cmp_evt);
    cmp_evt->operation  = param->operation;
    cmp_evt->status = param->status;
    cmp_evt->seq_num = param->seq_num ;
    ke_msg_send(cmp_evt);
	
    return (KE_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATTC_READ_IND message.
 * Generic event received after every simple read command sent to peer server.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gattc_read_ind_handler(ke_msg_id_t const msgid,
                                  struct gattc_read_ind const *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    bk_printf("appm %s \r\n",__func__);
    bk_printf("handle = 0x%x\r\n",param->handle);
    bk_printf("length = 0x%x\r\n",param->length);
    bk_printf("offset = 0x%x\r\n",param->offset);
    bk_printf("value = 0x");
    for(int i =0 ; i < param->length; i++)
    {
        bk_printf("%x ",param->value[i]);
    }
    bk_printf("\r\n");
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATTC_EVENT_IND message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

static int gattc_event_ind_handler(ke_msg_id_t const msgid,
                                   struct gattc_event_ind const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    uint8_t conidx = KE_IDX_GET(dest_id);
    #if 0
    bk_printf("sdp service task gattc_event_ind :%d:%d\r\n", KE_IDX_GET(dest_id), KE_IDX_GET(src_id));
    bk_printf("NOTIF RECIVE length:%d,value = \r\n",param->length);
    for(int i = 0; i< param->length; i++)
    {
        bk_printf("%02x ",param->value[i]);
    }
    bk_printf("\r\n");
    #endif

    struct gattc_event_ind *ind;
        // allocate indication
    ind = KE_MSG_ALLOC_DYN(GATTC_EVENT_IND ,
            KE_BUILD_ID(TASK_APP,conidx),dest_id, gattc_event_ind, param->length);

    // fill in parameters
    ind->type = GATTC_NOTIFY;
    ind->length = param->length;

    ind->handle = param->handle;
    memcpy(ind->value, param->value, param->length);

    /* send the message to app*/
    ke_msg_send(ind);
   
    return (KE_MSG_CONSUMED);
}
#include "user_func.h"
#include "ke_event.h"
static int gattc_event_req_ind_handler(ke_msg_id_t const msgid,
                                       struct gattc_event_ind const *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    
    #if 0
    bk_printf("sdp RECIVE value =  \r\n");
    for(int i = 0; i< param->length; i++)
    {
        bk_printf("%02x ",param->value[i]);
    }
    bk_printf("\r\n");    
    #endif
    set_at_rsp_ch(1);
    #if CLI_CONSOLE
    store_uart_ringbuf_data((uint8_t *)param->value, param->length);
    ke_event_set(KE_EVENT_AOS_CLI);
    #endif
    #if 1
    uint8_t conidx = KE_IDX_GET(dest_id);
    struct gattc_event_ind *ind;
        // allocate indication
    ind = KE_MSG_ALLOC_DYN(GATTC_EVENT_REQ_IND ,
            KE_BUILD_ID(TASK_APP,conidx),dest_id, gattc_event_ind, param->length);

    // fill in parameters
    ind->type = GATTC_INDICATE;
    ind->length = param->length;

    ind->handle = param->handle;
    memcpy(ind->value, param->value, param->length);

    /* send the message to app*/
    ke_msg_send(ind);
    
    struct gattc_event_cfm *cfm  = KE_MSG_ALLOC(GATTC_EVENT_CFM, src_id, dest_id, gattc_event_cfm);
    cfm->handle = param->handle;
    ke_msg_send(cfm);
    #endif
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
static int sdp_default_msg_handler(ke_msg_id_t const msgid,
                            void *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(src_id);
    bk_printf("sdp %s msgid:0x%x,dest_id:0x%x,src_id:0x%x\r\n",__func__,msgid,dest_id,src_id);
    bk_printf("conidx:%x\r\n",conidx);
    return (KE_MSG_CONSUMED);
}
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
/// Default State handlers definition
//const struct ke_msg_handler sdp_default_state[] =
KE_MSG_HANDLER_TAB(sdp)
{
    {KE_MSG_DEFAULT_HANDLER,    	(ke_msg_func_t)sdp_default_msg_handler},
    {SDP_ENABLE_REQ,                (ke_msg_func_t)sdp_enable_req_handler},
    {SDP_READ_INFO_REQ,             (ke_msg_func_t)sdp_read_info_req_handler},
    {SDP_WRITE_VALUE_INFO_REQ,      (ke_msg_func_t)sdp_write_value_info_req_handler},
    {SDP_WRITE_NTF_CFG_REQ,         (ke_msg_func_t)sdp_write_ntf_cfg_req_handler},
    {GATTC_READ_IND,                (ke_msg_func_t)gattc_read_ind_handler},
    {GATTC_EVENT_IND,               (ke_msg_func_t)gattc_event_ind_handler},
    {GATTC_EVENT_REQ_IND,           (ke_msg_func_t)gattc_event_req_ind_handler},
    {GATTC_CMP_EVT,                 (ke_msg_func_t)gattc_cmp_evt_handler},////mark
};
void sdp_task_init(struct ke_task_desc *task_desc)
{
    // Get the address of the environment
    struct sdp_env_tag *srv_env = PRF_ENV_GET(SDP, sdp);

    task_desc->msg_handler_tab = sdp_msg_handler_tab;
    task_desc->msg_cnt         = ARRAY_LEN(sdp_msg_handler_tab);
    task_desc->state           = srv_env->state;
    task_desc->idx_max         = SDP_IDX_MAX;
}

// Specifies the message handlers that are common to all states.

#endif /* (BLE_BATT_CLIENT) */
/// @} SDPTASK

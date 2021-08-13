/**
****************************************************************************************
*
* @file rwip.c
*
* @brief RW IP SW main module
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup RW IP SW main module
 * @ingroup ROOT
 * @brief The RW IP SW main module.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // RW SW configuration

#include <string.h>          // for mem* functions
#include <stdio.h>
#include "arch.h"            // Platform architecture definition
#include "compiler.h"
#include "co_version.h"      // version information
#include "co_utils.h"

#include "rwip.h"            // RW definitions
#include "rwip_int.h"        // RW internal definitions

#if (NVDS_SUPPORT)
#include "nvds.h"         // NVDS definitions
#endif // NVDS_SUPPORT

#if (BT_EMB_PRESENT)
#include "rwbt.h"            // rwbt definitions
#endif //BT_EMB_PRESENT

#if (BLE_EMB_PRESENT)
#include "rwble.h"           // rwble definitions
#endif //BLE_EMB_PRESENT

#if (BLE_HOST_PRESENT)
#include "rwble_hl.h"        // BLE HL definitions
#include "gapc.h"
#include "gapm.h"
#include "gattc.h"
#include "l2cc.h"
#endif //BLE_HOST_PRESENT

#if 0//(BLE_APP_PRESENT)
#include "app.h"             // Application definitions
#endif //BLE_APP_PRESENT


#if (BT_EMB_PRESENT)
#include "ld.h"
#endif //BT_EMB_PRESENT

#if 1//(DISPLAY_SUPPORT)

#include "co_utils.h"        // toolbox
#include "plf.h"             // platform definition
#if (BT_EMB_PRESENT)
#include "reg_btcore.h"
#endif // (BT_EMB_PRESENT)
#if (BLE_EMB_PRESENT)
#include "reg_blecore.h"
#endif // (BLE_EMB_PRESENT)
#if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
#include "reg_ipcore.h"
#endif // (BT_DUAL_MODE)
#endif //DISPLAY_SUPPORT

#if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
#include "sch_arb.h"            // Scheduling Arbiter
#include "sch_prog.h"           // Scheduling Programmer
#include "sch_plan.h"           // Scheduling Planner
#include "sch_slice.h"          // Scheduling Slicer
#include "sch_alarm.h"          // Scheduling Alarm
#endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)

#if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
#include "rf.h"              // RF definitions
#endif //BT_EMB_PRESENT || BLE_EMB_PRESENT

#if (H4TL_SUPPORT)
#include "h4tl.h"
#endif //H4TL_SUPPORT

#if (AHI_TL_SUPPORT)
#include "ahi.h"
#endif //AHI_TL_SUPPORT

#if (HCI_PRESENT)
#include "hci.h"             // HCI definition
#endif //HCI_PRESENT

#include "ke.h"              // kernel definition
#include "ke_event.h"        // kernel event
#include "ke_timer.h"        // definitions for timer
#include "ke_mem.h"          // kernel memory manager

#if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
#include "ecc_p256.h"        // ECC P256 library
#endif // (BT_EMB_PRESENT || BLE_EMB_PRESENT)

#include "aes.h"             // For AES functions

#if (BLE_EMB_PRESENT)
#include "reg_blecore.h"        // ble core registers
#endif //BLE_EMB_PRESENT

#if (BT_EMB_PRESENT)
#include "reg_btcore.h"         // bt core registers
#endif //BT_EMB_PRESENT

#include "dbg.h"             // debug definition

#ifdef CFG_CPU_BK3633
#include "../header/BK3633_Config.h"
#endif


struct rom_env_tag rom_env;



struct rwip_env_tag        rwip_env;

/*
 * DEFINES
 ****************************************************************************************
 */

/// Sleep Duration Value in periodic wake-up mode
#define MAX_SLEEP_DURATION_PERIODIC_WAKEUP      (0x0320 * 3)  // 1.5s
/// Sleep Duration Value in external wake-up mode
#define MAX_SLEEP_DURATION_EXTERNAL_WAKEUP      0x3E80  //10s

#if (DISPLAY_SUPPORT)
///Table of HW image names for display
static const char* plf_type[4] =
{
        "BK",
        "LE",
        "BT",
        "DM"
};
static const char* rf_type[6] =
{
         "NONE",
         "RPLE",
         "EXRC",
         "ICY1",
         "ICY2",
         "BTPT"
 };

/// FW type display line
#if (BT_EMB_PRESENT && BLE_EMB_PRESENT)
#define FW_TYPE_DISPLAY   "FW: BTDM split emb"
#elif (BT_EMB_PRESENT)
#define FW_TYPE_DISPLAY   "FW: BT split emb"
#elif (BLE_EMB_PRESENT && BLE_HOST_PRESENT)
#define FW_TYPE_DISPLAY   "FW: BLE full"
#elif (BLE_EMB_PRESENT)
#define FW_TYPE_DISPLAY   "FW: BLE split emb"
#else
#define FW_TYPE_DISPLAY   "FW: ROM"
#endif // BT_EMB_PRESENT / BLE_EMB_PRESENT / BLE_HOST_PRESENT
#endif //DISPLAY_SUPPORT




/*
 * STRUCT DEFINITIONS
 ****************************************************************************************
 */
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
/// Local supported commands
const struct rwip_prio rwip_priority[RWIP_PRIO_IDX_MAX]={
    #if (BT_EMB_PRESENT)
    {RWIP_PRIO_ACL_DFT,        RWIP_INCR_ACL_DFT},
    {RWIP_PRIO_ACL_ACT,        RWIP_INCR_ACL_ACT},
    {RWIP_PRIO_ACL_RSW,        RWIP_INCR_ACL_RSW},
    {RWIP_PRIO_ACL_SNIFF_DFT,  RWIP_INCR_ACL_SNIFF_DFT},
    {RWIP_PRIO_ACL_SNIFF_TRANS,RWIP_INCR_ACL_SNIFF_TRANS},
    #if MAX_NB_SYNC
    {RWIP_PRIO_SCO_DFT,       RWIP_INCR_SCO_DFT},
    #endif //MAX_NB_SYNC
    {RWIP_PRIO_BCST_DFT,      RWIP_INCR_BCST_DFT},
    {RWIP_PRIO_BCST_ACT,      RWIP_INCR_BCST_ACT},
    {RWIP_PRIO_CSB_RX_DFT,    RWIP_INCR_CSB_RX_DFT},
    {RWIP_PRIO_CSB_TX_DFT,    RWIP_INCR_CSB_TX_DFT},
    {RWIP_PRIO_INQ_DFT,       RWIP_INCR_INQ_DFT},
    {RWIP_PRIO_ISCAN_DFT,     RWIP_INCR_ISCAN_DFT},
    {RWIP_PRIO_PAGE_DFT,      RWIP_INCR_PAGE_DFT},
    {RWIP_PRIO_PAGE_1ST_PKT,  RWIP_INCR_PAGE_1ST_PKT},
    {RWIP_PRIO_PCA_DFT,       RWIP_INCR_PCA_DFT},
    {RWIP_PRIO_PSCAN_DFT,     RWIP_INCR_PSCAN_DFT},
    {RWIP_PRIO_PSCAN_1ST_PKT, RWIP_INCR_PSCAN_1ST_PKT},
    {RWIP_PRIO_SSCAN_DFT,     RWIP_INCR_SSCAN_DFT},
    {RWIP_PRIO_STRAIN_DFT,    RWIP_INCR_STRAIN_DFT},
    #endif // #if (BT_EMB_PRESENT)
    #if (BLE_EMB_PRESENT)
    {RWIP_PRIO_SCAN_DFT,      RWIP_INCR_SCAN_DFT},
    {RWIP_PRIO_AUX_RX_DFT,    RWIP_INCR_AUX_RX_DFT},
    {RWIP_PRIO_INIT_DFT,      RWIP_INCR_INIT_DFT},
    {RWIP_PRIO_CONNECT_DFT,   RWIP_INCR_CONNECT_DFT},
    {RWIP_PRIO_CONNECT_ACT,   RWIP_INCR_CONNECT_ACT},
    {RWIP_PRIO_ADV_DFT,       RWIP_INCR_ADV_DFT},
    {RWIP_PRIO_ADV_HDC_DFT,   RWIP_INCR_ADV_HDC_PRIO_DFT},
    {RWIP_PRIO_ADV_AUX_DFT,   RWIP_INCR_ADV_AUX_DFT},
    {RWIP_PRIO_PER_ADV_DFT,   RWIP_INCR_PER_ADV_DFT},
    {RWIP_PRIO_RPA_RENEW_DFT, RWIP_INCR_RPA_RENEW_DFT},
    #endif // #if (BLE_EMB_PRESENT)
};
#endif//#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
#if (RW_WLAN_COEX || RW_MWS_COEX)
const uint8_t rwip_coex_cfg[RWIP_COEX_CFG_MAX]={
    #if (BT_EMB_PRESENT)
    [RWIP_COEX_MSSWITCH_IDX] = (uint8_t)((RWIP_PTI_TXEN << RWIP_TXBSY_POS) | (RWIP_PTI_RXEN << RWIP_RXBSY_POS) | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS) | (RWIP_SAM_DIS << RWIP_SAMEN_POS)),
    [RWIP_COEX_SNIFFATT_IDX] = (uint8_t)((RWIP_PTI_TXEN << RWIP_TXBSY_POS) | (RWIP_PTI_RXEN << RWIP_RXBSY_POS) | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS) | (RWIP_SAM_DIS << RWIP_SAMEN_POS)),
    [RWIP_COEX_PAGE_IDX]     = (uint8_t)((RWIP_PTI_TXEN << RWIP_TXBSY_POS) | (RWIP_PTI_RXEN << RWIP_RXBSY_POS) | (RWIP_PTI_DNABORTEN << RWIP_DNABORT_POS) | (RWIP_SAM_DIS << RWIP_SAMEN_POS)),
    [RWIP_COEX_PSCAN_IDX]    = (uint8_t)((RWIP_PTI_TXEN << RWIP_TXBSY_POS) | (RWIP_PTI_RXEN << RWIP_RXBSY_POS) | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS) | (RWIP_SAM_DIS << RWIP_SAMEN_POS)),
    [RWIP_COEX_INQ_IDX]      = (uint8_t)((RWIP_PTI_TXEN << RWIP_TXBSY_POS) | (RWIP_PTI_RXEN << RWIP_RXBSY_POS) | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS) | (RWIP_SAM_DIS << RWIP_SAMEN_POS)),
    [RWIP_COEX_INQRES_IDX]   = (uint8_t)((RWIP_PTI_TXEN << RWIP_TXBSY_POS) | (RWIP_PTI_RXEN << RWIP_RXBSY_POS) | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS) | (RWIP_SAM_DIS << RWIP_SAMEN_POS)),
    [RWIP_COEX_SCORSVD_IDX]  = (uint8_t)((RWIP_PTI_TXEN << RWIP_TXBSY_POS) | (RWIP_PTI_RXEN << RWIP_RXBSY_POS) | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS) | (RWIP_SAM_DIS << RWIP_SAMEN_POS)),
    [RWIP_COEX_BCAST_IDX]    = (uint8_t)((RWIP_PTI_TXEN << RWIP_TXBSY_POS) | (RWIP_PTI_RXEN << RWIP_RXBSY_POS) | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS) | (RWIP_SAM_DIS << RWIP_SAMEN_POS)),
    [RWIP_COEX_CONNECT_IDX]  = (uint8_t)((RWIP_PTI_TXEN << RWIP_TXBSY_POS) | (RWIP_PTI_RXEN << RWIP_RXBSY_POS) | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS) | (RWIP_SAM_EN << RWIP_SAMEN_POS)),
    #endif // #if (BT_EMB_PRESENT)
    #if (BLE_EMB_PRESENT)
    [RWIP_COEX_CON_IDX]     = (uint8_t)((RWIP_PTI_TXDIS << RWIP_TXBSY_POS)  | (RWIP_PTI_RXDIS << RWIP_RXBSY_POS)  | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS)),
    [RWIP_COEX_CON_DATA_IDX]= (uint8_t)((RWIP_PTI_TXEN << RWIP_TXBSY_POS)  | (RWIP_PTI_RXEN << RWIP_RXBSY_POS)  | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS)),
    [RWIP_COEX_ADV_IDX]     = (uint8_t)((RWIP_PTI_TXEN << RWIP_TXBSY_POS)  | (RWIP_PTI_RXDIS << RWIP_RXBSY_POS) | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS)),
    [RWIP_COEX_SCAN_IDX]    = (uint8_t)((RWIP_PTI_TXDIS << RWIP_TXBSY_POS) | (RWIP_PTI_RXEN << RWIP_RXBSY_POS)  | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS)),
    [RWIP_COEX_INIT_IDX]    = (uint8_t)((RWIP_PTI_TXEN << RWIP_TXBSY_POS)  | (RWIP_PTI_RXEN << RWIP_RXBSY_POS)  | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS)),
    #endif // #if (BLE_EMB_PRESENT)
};
#endif //(RW_WLAN_COEX || RW_MWS_COEX)
#endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)


/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// RF API
struct rwip_rf_api         rwip_rf;
/// Parameters API
struct rwip_param_api      rwip_param;

#if 1
#if 1
#define KE_HEAP                    __attribute__((section("ke_heap"),zero_init))
#else
#define KE_HEAP 
#endif


#if 0
/// Heap definitions - use uint32 to ensure that memory blocks are 32bits aligned.
/// Memory allocated for environment variables
uint32_t rwip_heap_env[RWIP_CALC_HEAP_LEN(RWIP_HEAP_ENV_SIZE)];

#if (BLE_HOST_PRESENT)
/// Memory allocated for Attribute database
uint32_t rwip_heap_db[RWIP_CALC_HEAP_LEN(RWIP_HEAP_DB_SIZE)];

#endif // (BLE_HOST_PRESENT)
/// Memory allocated for kernel messages
uint32_t rwip_heap_msg[RWIP_CALC_HEAP_LEN(RWIP_HEAP_MSG_SIZE)];

/// Non Retention memory block
uint32_t rwip_heap_non_ret[RWIP_CALC_HEAP_LEN(RWIP_HEAP_NON_RET_SIZE)];

#else
uint32_t* rwip_heap_env;
uint32_t* rwip_heap_db;
uint32_t*rwip_heap_msg;
uint32_t*rwip_heap_non_ret;

//uint32_t rwip_heap_env[100];
//uint32_t rwip_heap_db[100];
//uint32_t rwip_heap_msg[100];
//uint32_t rwip_heap_non_ret[200];
#endif//

#endif//
/// IP reset state variable
uint8_t rwip_rst_state;

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if (!NVDS_SUPPORT)
__STATIC uint8_t rwip_param_dummy_get(uint8_t param_id, uint8_t * lengthPtr, uint8_t *buf)
{
    //stack_printf("rwip_param_dummy_get\r\n");
    return (PARAM_FAIL);
}
__STATIC uint8_t rwip_param_dummy_set(uint8_t param_id, uint8_t length, uint8_t *buf)
{
    return (PARAM_FAIL);
}
__STATIC uint8_t rwip_param_dummy_del(uint8_t param_id)
{
    return (PARAM_FAIL);
}
#endif // (!NVDS_SUPPORT)

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if PLF_UART		////Added
// Creation of uart external interface api
struct rwip_eif_api uart_api =
{
    NULL,NULL,NULL,NULL,
};
#endif




void rwip_eif_api_init(struct rwip_eif_api api)
{
    uart_api.read = api.read;
    uart_api.write = api.write;
    uart_api.flow_off = api.flow_off;
    uart_api.flow_on = api.flow_on;
}

void rwip_param_init(struct rwip_param_api param)
{
    rwip_param.get = param.get;
    rwip_param.set = param.set;
    rwip_param.del = param.del;
    
//        rwip_param.get = nvds_get;
//    rwip_param.set = nvds_put;
//    rwip_param.del = nvds_del;

}

const struct rwip_eif_api* rwip_eif_get(uint8_t idx)
{
    const struct rwip_eif_api* ret = NULL;
    
    if(uart_api.read == NULL)
    {
        return ret;
    }
    switch(idx)
    {
		#if PLF_UART		////
        case 0:
        {
            ret = &uart_api;
        }
        break;
		#endif
        #if PLF_UART2
        case 1:
        {
            ret = &uart_api;
        }
        break;
        #endif // PLF_UART2
        default:
        {
            ASSERT_INFO(0, idx, 0);
        }
        break;
    }
    return ret;
}
 
void rwip_ke_mem_init(uint32_t * heap_env,uint32_t * heap_db,uint32_t * heap_msg,uint32_t * heap_non_ret)
{
        rwip_heap_env = heap_env;
        rwip_heap_db = heap_db;
        rwip_heap_msg = heap_msg;
        rwip_heap_non_ret = heap_non_ret;
}

uint32_t   test_stack = 0x13572468;
void rwip_init(uint32_t error)
{
    // IP initialization
    rwip_rst_state = RWIP_INIT;
    #if (NVDS_SUPPORT)
    // Point to NVDS for parameters get/set
    if(!rwip_param.get || !rwip_param.set || !rwip_param.del)
    {
        ASSERT_WARN(0, 0, 0);
        
        stack_printf("rwip_param null\r\n");
    }

    #else // (NVDS_SUPPORT)
    // !! Need to point to some parameter configuration system
    ASSERT_WARN(0, 0, 0);
    rwip_param.get = rwip_param_dummy_get;
    rwip_param.set = rwip_param_dummy_set;
    rwip_param.del = rwip_param_dummy_del;
    #endif // (NVDS_SUPPORT)

    // Initialize kernel
    ke_init();
    // Initialize memory heap used by kernel.
    // Memory allocated for environment variables
    ke_mem_init(KE_MEM_ENV,           (uint8_t*)rwip_heap_env,     RWIP_CALC_HEAP_LEN_IN_BYTES(RWIP_HEAP_ENV_SIZE));
    //ke_mem_init(KE_MEM_ENV,           (uint8_t*)rwip_heap_env,     400);
    //stack_printf("MEM_ENV size:%d\r\n",RWIP_HEAP_ENV_SIZE);
    //stack_printf("test :%x\r\n",test_stack);
    
    #if (BLE_HOST_PRESENT)
    // Memory allocated for Attribute database
    ke_mem_init(KE_MEM_ATT_DB,        (uint8_t*)rwip_heap_db,      RWIP_CALC_HEAP_LEN_IN_BYTES(RWIP_HEAP_DB_SIZE));
  //  ke_mem_init(KE_MEM_ATT_DB,        (uint8_t*)rwip_heap_db,      400);
    //stack_printf("ATT_DB size:%d\r\n",RWIP_HEAP_DB_SIZE);
    #endif // (BLE_HOST_PRESENT) 
    // Memory allocated for kernel messages
    ke_mem_init(KE_MEM_KE_MSG,        (uint8_t*)rwip_heap_msg,     RWIP_CALC_HEAP_LEN_IN_BYTES(RWIP_HEAP_MSG_SIZE));
   // ke_mem_init(KE_MEM_KE_MSG,        (uint8_t*)rwip_heap_msg,     400);
    //stack_printf("KE_MSG size:%d\r\n",RWIP_HEAP_MSG_SIZE);
    // Non Retention memory block
    ke_mem_init(KE_MEM_NON_RETENTION, (uint8_t*)rwip_heap_non_ret, RWIP_CALC_HEAP_LEN_IN_BYTES(RWIP_HEAP_NON_RET_SIZE));
   // ke_mem_init(KE_MEM_NON_RETENTION, (uint8_t*)rwip_heap_non_ret, 800);
    //stack_printf("RETENTION size:%d\r\n",RWIP_HEAP_NON_RET_SIZE);

    #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    #if (RW_DEBUG)
    // Initialize the debug process
    dbg_init(false);
    #endif //(RW_DEBUG)
    #endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)

    // Initialize RF
    #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    
    #if	(ROM_REGISTER_CALLBACK)
    if(rom_env.rf_init != NULL)
    {
        rom_env.rf_init(&rwip_rf);
    }
    #else
    rf_init(&rwip_rf);
    #endif //(ROM_REGISTER_CALLBACK)
            
	
    #endif //BT_EMB_PRESENT || BLE_EMB_PRESENT

    #if (DISPLAY_SUPPORT)
    // Initialize display module
    //display_init();

    // Add some configuration information to display
    //display_add_config();
    #endif //DISPLAY_SUPPORT

    #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    // Initialize Diffie Hellman Elliptic Curve Algorithm
    ecc_init(false);
    #endif // (BT_EMB_PRESENT || BLE_EMB_PRESENT)

    // Initialize H4TL
    #if (H4TL_SUPPORT)
    #if (H4TL_NB_CHANNEL > 1)
    h4tl_init(1, rwip_eif_get(1));
    //stack_printf("h4tl_init ok\r\n");
    #endif // (H4TL_NB_CHANNEL > 1)
    h4tl_init(0, rwip_eif_get(0));
    //stack_printf("h4tl_init-1 ok\r\n");
    #endif //(H4TL_SUPPORT)

    #if (HCI_PRESENT)
    // Initialize the HCI
    hci_init(false);
    //stack_printf("hci_init ok\r\n");
    #endif //HCI_PRESENT

    #if (AHI_TL_SUPPORT)
    // Initialize the Application Host Interface
    ahi_init();
    //stack_printf("ahi_init ok\r\n");
    #endif //AHI_TL_SUPPORT

    #if (BLE_HOST_PRESENT)
    // Initialize BLE Host stack
    rwble_hl_init(false);
    //stack_printf("rwble_hl_init ok\r\n");
    #endif //BLE_HOST_PRESENT

    #if (BT_EMB_PRESENT)
    // Initialize BT
    rwbt_init();
    //stack_printf("rwbt_init ok\r\n");
    #endif //BT_EMB_PRESENT

    #if (BLE_EMB_PRESENT)
    // Initialize BLE
    rwble_init(false);
    //stack_printf("rwble_init ok\r\n");
    #endif //BLE_EMB_PRESENT

    // Initialize AES
    #if (BT_DUAL_MODE || BLE_STD_MODE)
    aes_init(false);
    //stack_printf("aes_init ok\r\n");
    #endif //(BT_DUAL_MODE || BLE_STD_MODE)

    #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    // Initialize Scheduling blocks
    sch_arb_init(false);
    sch_prog_init(false);
    sch_plan_init(false);
    sch_alarm_init(false);
    sch_slice_init(false);
    #endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)

    // Initialize IP core driver
    
    #if	(ROM_REGISTER_CALLBACK)
    if(rom_env.rwip_driver_init != NULL)
    {
        rom_env.rwip_driver_init(false);
    }
    #else
    rwip_driver_init(false);
    #endif //(ROM_REGISTER_CALLBACK)
    //stack_printf("rwip_driver_init ok\r\n");

    #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    #if (RW_WLAN_COEX)
    rwip_wlcoex_set(1);
    //stack_printf("rwip_wlcoex_set ok\r\n");
    #endif //(RW_WLAN_COEX)
    #endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)

    #if (BT_EMB_PRESENT || (BLE_EMB_PRESENT && !BLE_HOST_PRESENT))
    // If FW initializes due to FW reset, send the message to Host
    if(error != RESET_NO_ERROR)
    {
        //stack_printf("@@error(%x)\r\n",error);
        if(error == RESET_TO_ROM || error == RESET_AND_LOAD_FW)
        {
            // Send platform reset command complete if requested by user
////dbg_platform_reset_complete(error);
        }
        else
        {
            // Allocate a message structure for hardware error event
            struct hci_hw_err_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_HW_ERR_EVT_CODE, hci_hw_err_evt);

            // Fill the HW error code
            switch(error)
            {
                case RESET_MEM_ALLOC_FAIL: evt->hw_code = CO_ERROR_HW_MEM_ALLOC_FAIL; break;
                default: ASSERT_INFO(0, error, 0); break;
            }

            // Send the message
            hci_send_2_host(evt);
        }
    }
    #endif //(BT_EMB_PRESENT || (BLE_EMB_PRESENT && !BLE_HOST_PRESENT))

    /*
     ************************************************************************************
     * Application initialization
     ************************************************************************************
     */
    #if (BLE_APP_PRESENT)
    #if	(ROM_REGISTER_CALLBACK)
    if(rom_env.appm_init != NULL)
    {
        rom_env.appm_init();
        stack_printf("appm_init ok\r\n");
    }else
    {
        stack_printf("appm_init fail\r\n");
    }
    #else
     appm_init();
    #endif //(ROM_REGISTER_CALLBACK)
   
    
    #endif //BLE_APP_PRESENT
}

void rwip_reset(void)
{
    // Disable interrupts until reset procedure is completed
    GLOBAL_INT_DISABLE();

    stack_printf("rwip_reset 0\r\n");
    #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    #if (RW_DEBUG)
    // Reset dbg
    dbg_init(true);
    #endif //(RW_DEBUG)
    #endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)

    //Clear all message and timer pending
    ke_flush();

    #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    // Reset Diffie Hellman Elliptic Curve Algorithm
    ecc_init(true);
    #endif // (BT_EMB_PRESENT || BLE_EMB_PRESENT)

    #if (HCI_PRESENT)
    // Reset the HCI
    hci_init(true);
    #endif //HCI_PRESENT

    #if (BLE_HOST_PRESENT)
    // Initialize BLE Host stack
    rwble_hl_init(true);
    #endif //BLE_HOST_PRESENT

    #if (BT_EMB_PRESENT)
    // Reset BT
    rwbt_reset();
    #endif //BT_EMB_PRESENT

    #if (BLE_EMB_PRESENT)
    // Reset BLE
    rwble_init(true);
    #endif //BLE_EMB_PRESENT
#if (BT_DUAL_MODE || BLE_STD_MODE)
    // Reset AES
    aes_init(true);
#endif //(BT_DUAL_MODE || BLE_STD_MODE) 
    #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    // Reset Scheduling blocks
    sch_arb_init(true);
    sch_prog_init(true);
    sch_plan_init(true);
    sch_alarm_init(true);
    sch_slice_init(true);
    #endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)

    #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    // Initialize IP core driver
    #if	(ROM_REGISTER_CALLBACK)
    if(rom_env.rwip_driver_init != NULL)
    {
        rom_env.rwip_driver_init(false);
    }
    #else
    rwip_driver_init(false);
    #endif //(ROM_REGISTER_CALLBACK)
    
    #endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

    #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    #if (RW_WLAN_COEX)
    rwip_wlcoex_set(1);
    #endif //(RW_WLAN_COEX)

    // Reset the RF
    rwip_rf.reset();
    #endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)


    #if (DISPLAY_SUPPORT)
    // Restart display module
    display_resume();
    #endif //DISPLAY_SUPPORT

    // Restore interrupts once reset procedure is completed
    GLOBAL_INT_RESTORE();
    stack_printf("rwip_reset 1\r\n");
}

void rwip_schedule(void)
{
    #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    // If system is waking up, delay the handling up to the Bluetooth clock is available and corrected
    if ((rwip_env.prevent_sleep & RW_WAKE_UP_ONGOING) == 0)
    #endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    {
        // schedule all pending events
        ke_event_schedule();
    }
}


rwip_time_t rwip_time_get(void)
{
    rwip_time_t res;

    #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    //Sample the base time count
    ip_slotclk_samp_setf(1);
    
    while (ip_slotclk_samp_getf());		////20190905


    // get base time and offset
    res.hs  = ip_slotclk_sclk_getf();
    res.hus = HALF_SLOT_INV(ip_finetimecnt_get());
    #elif (BLE_HOST_PRESENT)
    // get base time (10 ms unit)
    res.hs  = timer_get_time() << 5;
    res.hus = 0;
    #endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

    return res;
}

void rwip_prevent_sleep_set(uint16_t prv_slp_bit)
{
    GLOBAL_INT_DISABLE();
    rwip_env.prevent_sleep |= prv_slp_bit;
    #if(BEKEN_EVENT_2_4G)
    if(prv_slp_bit == RW_BB_FRAME_ONGOING)
    {
        ke_event_set(KE_EVENT_BLE_BB_START); 
    }
    #endif 
    GLOBAL_INT_RESTORE();
}

uint16_t rwip_prevent_sleep_get(void)
{   
    return rwip_env.prevent_sleep; 
}

void rwip_prevent_sleep_clear(uint16_t prv_slp_bit)
{
    GLOBAL_INT_DISABLE();
    rwip_env.prevent_sleep &= ~prv_slp_bit;
    #if(BEKEN_EVENT_2_4G)
    if(prv_slp_bit == RW_BB_FRAME_ONGOING)
    {
        ke_event_set(KE_EVENT_BLE_BB_END);
    }
    #endif 
    GLOBAL_INT_RESTORE();
}
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)

bool rwip_active_check(void)
{
    bool result = true;

    do
    {
        #if BT_EMB_PRESENT
        if(rwip_env.prevent_sleep & (RW_CSB_NOT_LPO_ALLOWED | RW_BT_ACTIVE_MODE))
            break;
        #endif // BT_EMB_PRESENT

        #if BLE_EMB_PRESENT
        if(rwip_env.prevent_sleep & (RW_BLE_ACTIVE_MODE))
            break;
        #endif // BLE_EMB_PRESENT

        result = false;

    } while (0);

    return result;
}
#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)


void rwip_timer_10ms_set(uint32_t target)
{
    GLOBAL_INT_DISABLE();

    if (target != RWIP_INVALID_TARGET_TIME)
    {
        // save target time
        rwip_env.timer_10ms_target = RWIP_10MS_TIME_TO_CLOCK(target);

        #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
         // set the abs timeout in HW
        ip_clkntgt2_set(rwip_env.timer_10ms_target);
        ip_hmicrosectgt2_set(HALF_SLOT_TIME_MAX);

        // if timer is not enabled, it is possible that the irq is raised
        // due to a spurious value, so ack it before
        ip_intack1_timestamptgt2intack_clearf(1);
        ip_intcntl1_timestamptgt2intmsk_setf(1);
        #elif (BLE_HOST_PRESENT)
        // Start timer
        timer_set_timeout(target, rwip_timer_10ms_handler);
        #endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    }
    else
    {
        // save target time - not set
        rwip_env.timer_10ms_target = RWIP_INVALID_TARGET_TIME;

        #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
        // disable timer irq
        ip_intcntl1_timestamptgt2intmsk_setf(0);
        #elif (BLE_HOST_PRESENT)
        // Stop timer
        timer_set_timeout(0, NULL);
        #endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    }
    GLOBAL_INT_RESTORE();
}


#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
void rwip_timer_hs_set(uint32_t target)
{
    // save target time
    rwip_env.timer_hs_target = target;
    
    if (target != RWIP_INVALID_TARGET_TIME)
    {
        // set the abs timeout in HW
        ip_finetimtgt_finetarget_setf(target);

        // if timer is not enabled, it is possible that the irq is raised
        // due to a spurious value, so ack it before
        ip_intack1_finetgtintack_clearf(1);
        ip_intcntl1_finetgtintmsk_setf(1);
    }
    else
    {
        // disable timer irq
        ip_intcntl1_finetgtintmsk_setf(0);
    }
}

void rwip_timer_hus_set(uint32_t target, uint32_t half_us_delay)
{
    // save target time
    rwip_env.timer_hus_target = target;

    if (target != RWIP_INVALID_TARGET_TIME)
    {
        ASSERT_INFO(half_us_delay < HALF_SLOT_SIZE, half_us_delay, 0);

        // set the abs timeout in HW
        ip_clkntgt1_setf(target);
        ip_hmicrosectgt1_setf(HALF_SLOT_TIME_MAX - half_us_delay);

        // if timer is not enabled, it is possible that the irq is raised
        // due to a spurious value, so ack it before
        ip_intack1_timestamptgt1intack_clearf(1);
        ip_intcntl1_timestamptgt1intmsk_setf(1);
    }
    else
    {
        // disable timer irq
        ip_intcntl1_timestamptgt1intmsk_setf(0);
    }
}

void rwip_aes_encrypt(const uint8_t *key, const uint8_t* val)
{
    // Prevent going to deep sleep during encryption
    rwip_prevent_sleep_set(RW_CRYPT_ONGOING);

    // Copy data to EM buffer
    em_wr(val, EM_ENC_IN_OFFSET, KEY_LEN);

    // copy the key in the register dedicated for the encryption
    ip_aeskey31_0_set(  co_read32p(&(key[0])));
    ip_aeskey63_32_set( co_read32p(&(key[4])));
    ip_aeskey95_64_set( co_read32p(&(key[8])));
    ip_aeskey127_96_set(co_read32p(&(key[12])));

    // Set the pointer on the data to encrypt.
    ip_aesptr_setf(EM_ENC_IN_OFFSET >> 2);

    // enable crypt interrupt (and clear a previous interrupt if needed)
    ip_intack1_cryptintack_clearf(1);
    ip_intcntl1_cryptintmsk_setf(1);

    // start the encryption
    ip_aescntl_aes_start_setf(1);
}

void rwip_sw_int_req(void)
{
    // enable SW interrupt (and clear a previous interrupt if needed)
    ip_intack1_swintack_clearf(1);
    ip_intcntl1_swintmsk_setf(1);
    // start the SW interrupt
    ip_rwdmcntl_swint_req_setf(1);
}

#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

#if (BT_EMB_PRESENT)
#if PCA_SUPPORT
bool rwip_pca_clock_dragging_only(void)
{
#if (BLE_EMB_PRESENT)
    return rwble_activity_ongoing_check();
#else
    return false;
#endif // BLE_EMB_PRESENT
}
#endif // PCA_SUPPORT
#endif // BT_EMB_PRESENT

#if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
#if (RW_MWS_COEX)
void rwip_mwscoex_set(bool state)
{
#if (BT_EMB_PRESENT)
    if (state)
    {
        bt_coexifcntl0_mwswci_en_setf(0);
        bt_coexifcntl0_mwscoex_en_setf(1);
    }
    else
    {
        bt_coexifcntl0_mwswci_en_setf(0);
        bt_coexifcntl0_mwscoex_en_setf(0);
    }
#endif // BT_EMB_PRESENT
}
#endif // RW_MWS_COEX
#if (RW_WLAN_COEX)
void rwip_wlcoex_set(bool state)
{
#if (BLE_EMB_PRESENT)
    if (state)
    {
        ble_coexifcntl0_syncgen_en_setf(1);
        ble_coexifcntl0_wlancoex_en_setf(1);
    }
    else
    {
        ble_coexifcntl0_syncgen_en_setf(0);
        ble_coexifcntl0_wlancoex_en_setf(0);
    }
#endif // BLE_EMB_PRESENT
}
#endif // RW_WLAN_COEX
#endif // (BT_EMB_PRESENT || BLE_EMB_PRESENT)

#if RW_DEBUG
void rwip_assert(const char * file, int line, int param0, int param1, uint8_t type)
{
    #if (BT_EMB_PRESENT || (BLE_EMB_PRESENT && !BLE_HOST_PRESENT))
    struct hci_dbg_assert_evt *evt = KE_MSG_ALLOC_DYN(HCI_DBG_EVT, 0, 0, hci_dbg_assert_evt, sizeof(struct hci_dbg_assert_evt) + strlen(file));
    evt->subcode = HCI_DBG_ASSERT_EVT_SUBCODE;
    evt->type = type;
    evt->line = line;
    evt->param0 = param0;
    evt->param1 = param1;
    strcpy((char *) evt->file, file);
    hci_send_2_host(evt);
    #endif //(BT_EMB_PRESENT || (BLE_EMB_PRESENT && !BLE_HOST_PRESENT))
}
#endif //RW_DEBUG

///@} RW

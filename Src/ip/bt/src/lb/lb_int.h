/**
 ****************************************************************************************
 *
 * @file lb.h
 *
 * @brief Link Broadcaster internal definition
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 ****************************************************************************************
 */

#ifndef LB_INT_H_
#define LB_INT_H_

#include "rwbt_config.h"

/**
 ****************************************************************************************
 * @addtogroup LB Link Broadcaster
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>          // standard integer
#include <stdbool.h>         // standard boolean


/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximum number of instances of the LB task
#define LB_IDX_MAX  1

/// Default number broadcast retransmissions
#define LB_DEFAULT_NBC 2

/// As default CSB lt_addr is not reserved - invalid value 0 denotes this
#define LB_CSB_LT_ADDR_NOT_RESERVED 0

/// LMP message instant offset, where a specific clock instant is required (in slots)
#define LB_CLK_ADJ_CLK_TX_INSTANT_OFFSET  10

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Possible states of the LB task
enum LB_STATE
{
    /// FREE state
    LB_IDLE,
    /// WAIT MASTER KEY CFM state message handlers.
    LB_WAIT_MASTER_KEY_CFM,
    /// WAIT RESTART ENC CFM state message handlers.
    LB_WAIT_RESTART_ENC_CFM,

    // Number of states.
    LB_STATE_MAX
};



/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

#if PCA_SUPPORT
/// LC PCA coase clock adjustment Structure
struct lb_cca_tag
{
    /// transaction ID
    uint8_t         tr_id;
    /// aggregate coarse clock adjustment period
    uint8_t         frame_period;
    /// coarse clock adjustment event id
    uint8_t         clk_adj_id;
    /// coarse clock adjustment instant (in 625us BT slots)
    uint32_t         clk_adj_instant;
    /// coarse clock adjustment us offset
    int16_t         clk_adj_us;
    /// coarse clock adjustment slot offset
    uint8_t         clk_adj_slots;
    /// coarse clock adjustment mode
    uint8_t         clk_adj_mode;
    /// coarse clock adjustment PDU instant
    uint32_t         clk_adj_clk;
};
#endif // PCA_SUPPORT

///LB master key environment structure
struct master_key
{
    /// encryption key size mask
    uint16_t        enc_key_size_msk;
    /// encryption mode
    uint8_t         enc_mode;
    /// status
    uint8_t         status;
    /// in use boolean
    bool            in_use;
};

/// LB environment structure
struct lb_env_tag
{
#if CSB_SUPPORT
    /// reserved lt_addr for CSB
    uint8_t             res_lt_addr;
    /// max packet size for CSB data
    uint16_t            csb_max_pkt_size;
    /// CSB mode active flag
    bool                csb_mode_enabled;
    /// CSB interval
    uint16_t            csb_interval;
    /// CSB offset
    uint16_t            csb_offset;
    /// CSB buffer pointer
    uint16_t            buf_ptr;
    /// CSB buffer size
    uint16_t            csb_buf_data_len;
    /// Sync Train service data
    uint8_t             service_data;
#endif // CSB_SUPPORT
#if CSB_SUPPORT || PCA_SUPPORT
    /// Sync Train timeout
    uint32_t            sync_train_to;
    /// Sync Train interval
    uint32_t            sync_train_intv;
    /// Synchronization_scanTO (in slots)
    uint16_t            sync_scan_to;
    /// Sync_Scan_Window (in slots)
    uint16_t           sync_scan_win;
    /// Sync_Scan_Interval (in slots)
    uint16_t           sync_scan_int;
#endif // CSB_SUPPORT || PCA_SUPPORT
     /// Active channel Map for CSB
      struct chnl_map ch_map;

    /// nb of piconet broadcast acl
    uint16_t            nb_pic_acl;
    /// connection handle
    uint16_t            conhdl;
    /// act handle
    uint16_t            act_hdl;
    /// piconet handle
    uint16_t            pic_hdl;
    /// process id count
    uint16_t            pid_lc_cnt;

    /// nb broadcast retransmissions
    uint8_t                num_bcst_ret;

#if PCA_SUPPORT
    /// pca data structure
    struct  lb_cca_tag cca;
#endif // PCA_SUPPORT

    /// master key
    struct master_key   mstkey[MAX_NB_ACTIVE_ACL];
    /// bd address
    uint8_t             lid;
    /// new key flag
    uint8_t             new_key_flag;
    /// master key request flag
    bool                mst_key_req;
    /// encryption broadcast
    bool                enc_brcst;

    /// master key
    struct ltk  LM_MasterKey;
    /// master random key
    struct ltk  LM_MasterKeyRand;
    /// master encryption key
    struct ltk  LM_MasterEncRand;
    /// master encryption key size
    uint8_t     LM_MasterEncKeySize;
};


/*
 * GLOBAL VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LB task descriptor
extern const struct ke_task_desc TASK_DESC_LB;

/// LB task message handlers table
extern const struct ke_state_handler lb_default_handler;

/// LB task state
extern ke_state_t lb_state[LB_IDX_MAX];

/// LB environment variable
extern struct lb_env_tag lb_env;


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handle the master key creation.
 *
 * This function create the master key and send the request over every pid LC
 ****************************************************************************************
 */
void lb_mst_key(void);

/**
 ****************************************************************************************
 * @brief Handle the master key creation completion.
 *
 * This function create the master key and send the request over every pid LC
 ****************************************************************************************
 */
void lb_mst_key_cmp(uint8_t status, uint16_t conhdl, uint8_t new_key_flag);

/**
 ****************************************************************************************
 * @brief Restart encryption
 *
 * @param[in] conhdl       Connection handle
 ****************************************************************************************
 */
void lb_mst_key_restart_enc(uint16_t conhdl);

/**
 ****************************************************************************************
 * @brief This function start Active Broadcast Encryption link.
 ****************************************************************************************
 */
void lb_mst_start_act_bcst_enc(void);

/**
 ****************************************************************************************
 * @brief This function Stop Active Broadcast Encryption.
 ****************************************************************************************
 */
void lb_mst_stop_act_bcst_enc(void);

/// @} LB

#endif // LB_INT_H_

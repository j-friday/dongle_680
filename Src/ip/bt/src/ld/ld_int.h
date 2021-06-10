/**
****************************************************************************************
*
* @file ld_int.h
*
* @brief LD Internal API
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/

#ifndef LD_INT_H_
#define LD_INT_H_

/**
 ****************************************************************************************
 * @addtogroup LD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration

#include <stdint.h>         // integer
#include <stdbool.h>        // boolean

#include "ld.h"

/*
 * CONSTANT DEFINITIONS
 ****************************************************************************************
 */

/// BCH size
#define LD_BCH_SIZE                          8


/// Position of ACL links in the active links bit field
#define LD_ACT_ACL_POS    0
/// Mask of ACL links in the active links bit field
#define LD_ACT_ACL_MSK    (((1<<MAX_NB_ACTIVE_ACL) - 1) << LD_ACT_ACL_POS)
/// Position of SCO links in the active links bit field
#define LD_ACT_SCO_POS    MAX_NB_ACTIVE_ACL
/// Mask of SCO links in the active links bit field
#define LD_ACT_SCO_MSK    (((1<<MAX_NB_SYNC) - 1) << LD_ACT_SCO_POS)

/// Set an ACL link as active
#define LD_ACT_ACL_SET(field, link_id)           (field |= (1<<(link_id + LD_ACT_ACL_POS)))
/// Set a SCO link as active
#define LD_ACT_SCO_SET(field, sco_link_id)       (field |= (1<<(sco_link_id + LD_ACT_SCO_POS)))
/// Clear an ACL link from active links
#define LD_ACT_ACL_CLEAR(field, link_id)         (field &= ~(1<<(link_id + LD_ACT_ACL_POS)))
/// Clear a SCO link from active links
#define LD_ACT_SCO_CLEAR(field, sco_link_id)     (field &= ~(1<<(sco_link_id + LD_ACT_SCO_POS)))

/// Check if there is at least 1 active ACL
#define LD_ACT_ACL_CHECK(field)         ((field & LD_ACT_ACL_MSK) != 0)
/// Check if there is at least 1 active SCO
#define LD_ACT_SCO_CHECK(field)         ((field & LD_ACT_SCO_MSK) != 0)
/// Check if there is at least 1 active link (ACL or SCO)
#define LD_ACT_CHECK(field)             ((field) != 0)

/// Allocate an event with an associated data structure
#define LD_ALLOC_EVT(data_struct)           (struct data_struct *) ke_malloc(sizeof(struct data_struct), KE_MEM_ENV);

/// Initialize memory of an event and an associated data structure
#define LD_INIT_EVT(evt, data_struct)       memset(evt , 0, sizeof(struct data_struct));


/// Index for DM1 packet type (used in both BR and EDR)
#define DM1_IDX         0

/// Basic Rate packet types indexes (classified by packet size)
#define DH1_IDX         1
#define DM3_IDX         2
#define DH3_IDX         3
#define DM5_IDX         4
#define DH5_IDX         5
#define BR_IDX_MAX      6

/// Enhanced Data Rate packet types indexes (classified by packet size)
#define DH1_2_IDX       1
#define DH1_3_IDX       2
#define DH3_2_IDX       3
#define DH3_3_IDX       4
#define DH5_2_IDX       5
#define DH5_3_IDX       6
#define EDR_IDX_MAX     7


/*
 * ENUMERATION DEFINITIONS
 ****************************************************************************************
 */

/// Activity type
enum LD_ACT_TYPE
{
    LD_ACT_TYPE_ACL,
    LD_ACT_TYPE_SCO,
};

/// Active mode
enum LD_ACT_MODE
{
    LD_ACT_MODE_OFF,
    LD_ACT_MODE_ON,
};


/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

/// LD environment structure
struct ld_env_tag
{
    /// Local BD_ADDR
    struct bd_addr local_bd_addr;

    /**
     * Class of Device
     *
     * 0xXXXXXX Class of Device for the device.
     */
    struct devclass class_of_dev;

    /// Channel assessment data
    struct ld_afh_ch_assess_data afh_ch_assess;

    #if MAX_NB_SYNC
    /// Bit field for PCM setting (used when creating an audio link)
    uint8_t pcm_settings[8];
    #endif //MAX_NB_SYNC

    /// Bit field indicating the active links (Bits [M+N-1:N] -> SCO | Bits [N-1:0] -> ACL)
    uint16_t active_links;

    /// Local BCH
    uint8_t local_bch[LD_BCH_SIZE];

    /// Current RX descriptor index [0:N-1]
    uint8_t curr_rxdesc_index;

    /// Round trip delay
    uint8_t round_trip_delay;

    /// Maximum value of the local clock drift
    uint8_t local_drift;

    /// Maximum value of the local clock jitter
    uint8_t local_jitter;

    #if (RW_BT_MWS_COEX)
    /// MWS external frame duration (us)
    uint16_t mws_ext_fr_duration;

    /// MWS external frame sync assert offset (us)
    int16_t mws_ext_fr_offset;

    /// active MWS pattern index [0-2, 0xFF=disabled]
    uint8_t mws_index;
    #endif //RW_BT_MWS_COEX
};

#if RW_BT_MWS_COEX
/// MWS pattern update information
typedef struct
{
    /// MWS pattern index (0-2, 0xFF=disabled)
    uint8_t pattern_index;
    /// slot offset (activation time) (slots)
    uint8_t slot_offset;
} ld_mwsifstat_t;
#endif //RW_BT_MWS_COEX


/*
 * VARIABLE DECLARATION
 *****************************************************************************************
 */

/// Basic Rate packet types table (classified by packet size)
extern const uint8_t ld_acl_br_types[BR_IDX_MAX];

/// Basic Rate packet sizes table (classified by packet size)
extern const uint16_t ld_acl_br_sizes[BR_IDX_MAX];

/// Enhanced Data Rate packet types table (classified by packet size)
extern const uint8_t ld_acl_edr_types[EDR_IDX_MAX];

/// Enhanced Data Rate packet sizes table (classified by packet size)
extern const uint16_t ld_acl_edr_sizes[EDR_IDX_MAX];

/// LD environment variable
extern struct ld_env_tag ld_env;

/*
 * FUNCTIONS DECLARATION
 ****************************************************************************************
 */

#if RW_BT_MWS_COEX
/**
****************************************************************************************
* @brief Get the active MWS index
*
* @return Pattern index (0-2, 0xFF=disabled) and activation offset (@see ld_mwsifstat_t)
*
*****************************************************************************************
*/
ld_mwsifstat_t ld_mws_pattern_index_get(void);

/**
****************************************************************************************
* @brief Get the inactivity MWS duration
*
* @return uint8_t            Inactivity duration (0:none, 1-30: val*5ms, 31:infinite)
*
*****************************************************************************************
*/
uint8_t ld_mws_inactivity_duration_get(void);

/**
 ****************************************************************************************
 * @brief Build Xi scan mask for MWS COEX generically based on the specified uap_lap.
 *
 * @param[in]  mwscoex_xi_scan_mask         Xi mask to build.
 * @param[in]  uap_lap                      UAP/LAP for building Xi mask.
 *  ****************************************************************************************
 */
void ld_mwscoex_xi_scan_mask_build(uint32_t *mwscoex_xi_scan_mask, uint32_t uap_lap);

#endif // RW_BT_MWS_COEX

/**
 ****************************************************************************************
 * @brief Assess a RF channel
 *
 * @param[in]  channel       Channel index
 * @param[in]  rx_ok         True: reception is correct | False: reception is corrupted
 * @param[in]  timestamp     Timestamp of the assessment (in half-slots)
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
void ld_channel_assess(uint8_t channel, bool rx_ok, uint32_t timestamp);

/**
 ****************************************************************************************
 * @brief Free current RX descriptor
 ****************************************************************************************
 */
void ld_rxdesc_free(void);

/**
 ****************************************************************************************
 * @brief Check if the current RX descriptor is consumed by a given activity
 *
 * @param[in]   label   representing the activity (as indicated in the CS)
 *
 * @return True if current RX descriptor is consumed by the activity identified by label
 ****************************************************************************************
 */
bool ld_rxdesc_check(uint8_t label);

/**
 ****************************************************************************************
 * @brief Indicate active mode
 *
 * @param[in]   type        0:ACL | 1:SCO
 * @param[in]   link_id     ACL or SCO link ID
 * @param[in]   mode        0: inactive | 1: active
 ****************************************************************************************
 */
void ld_active_mode_set(uint8_t type, uint8_t link_id, uint8_t mode);


void ld_inq_init(void);
void ld_inq_reset(void);
void ld_iscan_init(void);
void ld_iscan_reset(void);
void ld_page_init(void);
void ld_page_reset(void);
void ld_pscan_init(void);
void ld_pscan_reset(void);
void ld_acl_init(void);
void ld_acl_reset(void);
void ld_bcst_acl_init(void);
void ld_bcst_acl_reset(void);
#if CSB_SUPPORT || PCA_SUPPORT
void ld_sscan_init(void);
void ld_sscan_reset(void);
void ld_strain_init(void);
void ld_strain_reset(void);
#endif // CSB_SUPPORT || PCA_SUPPORT
#if CSB_SUPPORT
void ld_csb_tx_init(void);
void ld_csb_tx_reset(void);
void ld_csb_rx_init(void);
void ld_csb_rx_reset(void);
#endif // CSB_SUPPORT
#if PCA_SUPPORT
void ld_pca_init(void);
void ld_pca_reset(void);
#endif // PCA_SUPPORT

/// @} LD
#endif // LD_INT_H_

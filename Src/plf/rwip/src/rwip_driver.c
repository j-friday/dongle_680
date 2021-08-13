/**
****************************************************************************************
*
* @file rwip_driver.c
*
* @brief RW IP Driver SW module used to manage common IP features.
*
* Copyright (C) RivieraWaves 2009-2015
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
#include "rwip.h"            // RW definitions
#include "rwip_int.h"        // RW internal definitions
#include "arch.h"            // Platform architecture definition

#if (NVDS_SUPPORT)
#include "nvds.h"            // NVDS definitions
#endif // NVDS_SUPPORT

#if (H4TL_SUPPORT)
#include "h4tl.h"            // H4TL definition
#endif //H4TL_SUPPORT

#include "dbg.h"             // debug definition
#include "ke_mem.h"          // for AES client management

#include "ke.h"              // To check if a kernel event is programmed
#include "ke_event.h"

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
#include "sch_alarm.h"       // for the half slot target ISR
#include "sch_arb.h"         // for the half us target ISR
#include "sch_prog.h"        // for the fifo/clock ISRs
#include "led.h"
#include "reg_ipcore.h"
#include "aes.h"             // AES result function

#if (BLE_EMB_PRESENT)
#include "rwble.h"           // for sleep and wake-up specific functions
#include "lld.h"             // for AES encryption handler
#endif // (BLE_EMB_PRESENT)
#if (BT_EMB_PRESENT)
#include "rwbt.h"            // for sleep and wake-up specific functions
#include "ld.h"              // for clock interrupt handler
#endif // (BT_EMB_PRESENT)
#elif  (BLE_HOST_PRESENT)
#include "timer.h"
#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

#include "co_math.h"         // min/max macros
#include "uart.h"


/*
 * DEFINES
 ****************************************************************************************
 */
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
/// Sleep Duration Value in periodic wake-up mode in Half slots
#define MAX_SLEEP_DURATION_PERIODIC_WAKEUP      (0x0640 * 3)  // 1.5s

/// Sleep Duration Value in external wake-up mode
#define MAX_SLEEP_DURATION_EXTERNAL_WAKEUP      0x7D00  //10s



#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */


#if 0//(BLE_EMB_PRESENT || BT_EMB_PRESENT)
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

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
/**
 ****************************************************************************************
 * @brief Converts a duration in lp cycles into a duration in half us.
 *
 * The function converts a duration in lp cycles into a duration is half us, according to the
 * low power clock frequency (32768Hz or 32000Hz).
 *
 * To do this the following formula are applied:
 *
 *   Tus = (x*30.517578125)*2 = (30*x + x/2 + x/64 + x/512)*2 = (61*x + (x*8 + x)/256) for a 32.768kHz clock or
 *   Tus = (x*31.25)*2        = (31*x + x/4) * 2              = (62*x + x/2)           for a 32kHz clock
 *
 * @param[in]     lpcycles    duration in lp cycles
 * @param[in|out] error_corr  Insert and retrieve error created by truncating the LP Cycle Time to a half us (32kHz: 1/2 half us | 32.768kHz: 1/256 half-us)
 *
 * @return duration in half us
 ****************************************************************************************
 */
__STATIC uint32_t rwip_lpcycles_2_hus(uint32_t lpcycles, uint32_t *error_corr)
{
    uint32_t res;

    // Sanity check: The number of lp cycles should not be too high to avoid overflow
    ASSERT_ERR(lpcycles < 2000000);

    #if (HZ32000)
    // Compute the sleep duration in us - case of a 32kHz clock and insert previous computed error
    *error_corr = lpcycles + *error_corr;
    // get the truncated value
    res = *error_corr >> 1;
    // retrieve new inserted error
    *error_corr = *error_corr - (res << 1);
    // finish computation
    res = 62 * lpcycles + res;
    #else //HZ32000
    // Compute the sleep duration in half us - case of a 32.768kHz clock and insert previous computed error
    *error_corr = (lpcycles << 3) + lpcycles + *error_corr;
    // get the truncated value
    res = *error_corr >> 8;
    // retrieve new inserted error
    *error_corr = *error_corr - (res << 8);
    // finish computation
    res = 61 * lpcycles + res;
    #endif //HZ32000

    return(res);
}

/**
 ****************************************************************************************
 * @brief Converts a duration in half slots into a number of low power clock cycles.
 * The function converts a duration in half slots into a number of low power clock cycles.
 * Sleep clock runs at either 32768Hz or 32000Hz, so this function divides the value in
 * slots by 10.24 or 10 depending on the case.
 * To do this the following formulae are applied:
 *
 *   N = x * 10.24 = (1024 * x)/100 for a 32.768kHz clock or
 *   N = x * 10                     for a 32kHz clock
 *
 * @param[in] hs_cnt    The value in half slot count
 *
 * @return The number of low power clock cycles corresponding to the slot count
 *
 ****************************************************************************************
 */
__STATIC int32_t rwip_slot_2_lpcycles(int32_t hs_cnt)
{
    int32_t lpcycles;

    #if HZ32000
    // Sanity check: The number of slots should not be too high to avoid overflow
    ASSERT_ERR(hs_cnt < (0xFFFFFFFF / 10));

    // Compute the low power clock cycles - case of a 32kHz clock
    lpcycles = hs_cnt * 10;
    #else //HZ32000
    // Sanity check: The number of slots should not be too high to avoid overflow
    ASSERT_ERR(hs_cnt < (0xFFFFFFFF >> 10));

    // Compute the low power clock cycles - case of a 32.768kHz clock
    lpcycles = (hs_cnt << 10)/100;
    #endif //HZ32000

    // So reduce little bit sleep duration in order to allow fine time compensation
    // Note compensation will be in range of [1 , 2[ lp cycles if there is no external wake-up
    lpcycles--;

    return(lpcycles);
}



/**
 ****************************************************************************************
 * @brief Converts a duration in us into a duration in lp cycles.
 *
 * The function converts a duration in us into a duration is lp cycles, according to the
 * low power clock frequency (32768Hz or 32000Hz).
 *
 * @param[in] us    duration in us
 *
 * @return duration in lpcycles
 ****************************************************************************************
 */
__STATIC uint32_t rwip_us_2_lpcycles(uint32_t us)
{
    uint32_t lpcycles;

    #if (HZ32000)
    // Compute the low power clock cycles - case of a 32kHz clock
    lpcycles = ((us * 32) + (999)) / 1000;
    #else //HZ32000
    // Compute the low power clock cycles - case of a 32.768kHz clock
    lpcycles = ((us * 32768) + (999999)) / 1000000;
    #endif //HZ32000

    return(lpcycles);
}

/**
 ****************************************************************************************
 * @brief Handles the Half slot timer target
 ****************************************************************************************
 */
__STATIC void rwip_timer_hs_handler(void)
{
    // disable the timer driver
    rwip_env.timer_hs_target = RWIP_INVALID_TARGET_TIME;
    ip_intcntl1_finetgtintmsk_setf(0);

    // call the default half slot call-back
    sch_alarm_timer_isr();
}

/**
 ****************************************************************************************
 * @brief Handles the Half slot timer target
 ****************************************************************************************
 */
__STATIC void rwip_timer_hus_handler(void)
{
    // disable the timer driver
    rwip_env.timer_hus_target = RWIP_INVALID_TARGET_TIME;
    ip_intcntl1_timestamptgt1intmsk_setf(0);

    // call the default half slot call-back
    sch_arb_event_start_isr();
}

#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

/**
 ****************************************************************************************
 * @brief Handles the 10 ms timer target
 ****************************************************************************************
 */
__STATIC void rwip_timer_10ms_handler(void)
{
    // disable the timer driver
    rwip_env.timer_10ms_target = RWIP_INVALID_TARGET_TIME;

    #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    ip_intcntl1_timestamptgt2intmsk_setf(0);
    #elif (BLE_HOST_PRESENT)
    // Stop timer
    timer_set_timeout(0, NULL);
    #endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

    // Mark that 10ms timer is over
    ke_event_set(KE_EVENT_KE_TIMER);
}
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)

/**
 ****************************************************************************************
 * @brief Handles crypto event (to provide results out of interrupt context
 ****************************************************************************************
 */
__STATIC void rwip_crypt_evt_handler(void)
{
    uint8_t aes_result[KEY_LEN];

    // Clear event
    ke_event_clear(KE_EVENT_AES_END);

    // Load AES result
    em_rd(aes_result, EM_ENC_OUT_OFFSET, KEY_LEN);
#if (BT_DUAL_MODE || BLE_STD_MODE)
    // inform AES result handler
    aes_result_handler(CO_ERROR_NO_ERROR, aes_result);
#endif //(BT_DUAL_MODE || BLE_STD_MODE)
}

/**
 ****************************************************************************************
 * @brief Handles crypto interrupt
 ****************************************************************************************
 */
__STATIC void rwip_crypt_isr_handler(void)
{
    // Prevent going to deep sleep during encryption
    rwip_prevent_sleep_clear(RW_CRYPT_ONGOING);

    // Clear interrupt mask
    ip_intcntl1_cryptintmsk_setf(0);

    // mark that AES is done
    ke_event_set(KE_EVENT_AES_END);
}

/**
 ****************************************************************************************
 * @brief Handles Software requested interrupt
 ****************************************************************************************
 */
__STATIC void rwip_sw_int_handler(void)
{
    // Disable interrupt
    ip_intcntl1_swintmsk_setf(0);

    // call the SW interrupt handler
    sch_arb_sw_isr();
}
extern void app_wdt_feed(void);
/**
 ****************************************************************************************
 * @brief Wake-up from Core sleep.
 *
 * Compute and apply the clock correction according to duration of the deep sleep.
 ****************************************************************************************
 */
__STATIC void rwip_wakeup(void)
{
    uint16_t fintetime_correction;
    // duration in half us
    uint32_t dur_hus;
    // duration in half slot
    uint32_t dur_hslot;
    // Get the number of low power sleep period
    uint32_t slp_period = ip_deepslstat_get();
    
   
    DBG_SWDIAG(SLEEP, SLEEP, 0);

    // Prevent going to deep sleep until a slot interrupt is received
    rwip_prevent_sleep_set(RW_WAKE_UP_ONGOING);

    // Re-enable external wake-up by default
    ip_deepslcntl_extwkupdsb_setf(0);

    // Compensate the base time counter and fine time counter by the number of slept periods
    dur_hus = rwip_lpcycles_2_hus(slp_period, &(rwip_env.sleep_acc_error));
    // Compute the sleep duration (based on number of low power clock cycles)
    dur_hslot = dur_hus / HALF_SLOT_SIZE;

    // retrieve halfslot sleep duration
    fintetime_correction = (HALF_SLOT_SIZE-1) - (dur_hus - dur_hslot*HALF_SLOT_SIZE);

    // The correction values are then deduced from the sleep duration in us
    ip_clkncntcorr_pack(/*absdelta*/ 1, /*clkncntcorr*/ dur_hus / HALF_SLOT_SIZE);

    // The correction values are then deduced from the sleep duration in us
    ip_finecntcorr_setf(fintetime_correction);

    // Start the correction
    ip_deepslcntl_deep_sleep_corr_en_setf(1);

    // Enable the RWBT slot interrupt
    ip_intcntl1_clknintsrmsk_setf(0);
    ip_intcntl1_set(IP_CLKNINTMSK_BIT);
    ip_intack1_clear(0xFFFFFFFF);

    #if (H4TL_SUPPORT)
    // Restart the flow on the TL
    h4tl_start();
    #endif //H4TL_SUPPORT

    TRC_REQ_WAKEUP();
    app_wdt_feed();
}



/**
 ****************************************************************************************
 * @brief Restore the core processing after the clock correction
 *
 * Enable the core and check if some timer target has been reached.
 ****************************************************************************************
 */
__STATIC void rwip_wakeup_end(void)
{
    // get current time
    rwip_time_t current_time = rwip_time_get();
 //   GPIO_DEBUG_TRIGER(0x6);
    // Clear slot interrupt, not needed anymore
    ip_intcntl1_clknintmsk_setf(0);

    #if (BT_EMB_PRESENT)
    // Wake-up BT core
    rwbt_sleep_wakeup_end();
    #endif // (BT_EMB_PRESENT)

    #if (BLE_EMB_PRESENT)
    // Wake-up BLE core
    
    #if	(ROM_REGISTER_CALLBACK)
    if(rom_env.rwble_sleep_wakeup_end != NULL)
    {
        rom_env.rwble_sleep_wakeup_end();
    }
    #else
    rwble_sleep_wakeup_end();
    #endif //(ROM_REGISTER_CALLBACK)
    #endif // (BLE_EMB_PRESENT)
  //  uart_printf("wakeup_end:%d:%d:%d\r\n",current_time.hs,rwip_env.timer_hs_target,rwip_env.timer_hus_target);
    // Re-enable default common interrupts
    ip_intcntl1_set(IP_FIFOINTMSK_BIT | IP_CRYPTINTMSK_BIT | IP_ERRORINTMSK_BIT | IP_SWINTMSK_BIT);
  //  bk_printf("wakeup_end timr:%d:%d:%d\r\n",current_time.hs,rwip_env.timer_hs_target,rwip_env.timer_hus_target);
    if(rwip_env.timer_hs_target != RWIP_INVALID_TARGET_TIME)
    {
        // check if half slot timer target is reach
        if(1)//((current_time.hs == rwip_env.timer_hs_target))
        {
            rwip_timer_hs_handler();
        }
        // enable half slot timer interrupt
        else
        {
            ip_intcntl1_finetgtintmsk_setf(1);
        }
    }

    if(rwip_env.timer_hus_target != RWIP_INVALID_TARGET_TIME)
    {
        // check if half us timer target is reach
        if(1)//((current_time.hs == rwip_env.timer_hus_target))
        {
            rwip_timer_hus_handler();
        }
        // enable half us timer interrupt
        else
        {
            ip_intcntl1_timestamptgt1intmsk_setf(1);
        }
    }

    if(rwip_env.timer_10ms_target != RWIP_INVALID_TARGET_TIME)
    {
        // check if 10ms target is reach
        if(current_time.hs == rwip_env.timer_10ms_target)
        {
            rwip_timer_10ms_handler();
        }
        // enable 10ms timer interrupt
        else
        {
            ip_intcntl1_timestamptgt2intmsk_setf(1);
        }
    }  
    // Wake up is complete now, so we allow the deep sleep again
    rwip_prevent_sleep_clear(RW_WAKE_UP_ONGOING);
    
    led_reset(6);
  
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)


#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
#if (BT_EMB_PRESENT)
void rwip_time_set(uint32_t clock)
{
    ip_slotclk_pack(IP_SAMP_RST, 1 /* clk_upd */, clock & 0x0FFFFFFF);
    while(ip_slotclk_clkn_upd_getf());
}
#endif // (BT_EMB_PRESENT)
#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

uint8_t rwip_sleep(void)
{
    uint8_t sleep_res = RWIP_ACTIVE;
    #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    int32_t sleep_duration;
    rwip_time_t current_time;
    #endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

    DBG_SWDIAG(SLEEP, FUNC, 1);

    DBG_SWDIAG(SLEEP, ALGO, 0);

    do
    {
        /************************************************************************
         **************            CHECK KERNEL EVENTS             **************
         ************************************************************************/
        // Check if some kernel processing is ongoing (during wakeup, kernel events are not processed)
        if (((rwip_env.prevent_sleep & RW_WAKE_UP_ONGOING) == 0) && !ke_sleep_check())
            break;
      //  bk_printf("%x:%x\r\n",rwip_env.prevent_sleep,ke_sleep_check());
        // Processor sleep can be enabled
        sleep_res = RWIP_CPU_SLEEP;

        DBG_SWDIAG(SLEEP, ALGO, 1);

        /************************************************************************
         **************              CHECK RW FLAGS                **************
         ************************************************************************/
        // First check if no pending procedure prevent from going to sleep
        if (rwip_env.prevent_sleep != 0)
            break;

        #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
        DBG_SWDIAG(SLEEP, ALGO, 2);

        /************************************************************************
         **************           Retrieve Current time            **************
         ************************************************************************/
        current_time = rwip_time_get();
        // remove the on-going slot for clock correction
        current_time.hs += 1;
        // Remove 1 more slot because next slot will be started at end of function
        if((HALF_SLOT_INV(current_time.hus)) < rwip_env.sleep_algo_dur)
        {
            current_time.hs += 1;
        }
        // Be sure that we don't exceed the clock wrapping time
        current_time.hs &= RWIP_MAX_CLOCK_TIME;

        /************************************************************************
         ******* COMPUTE SLEEP TIME ACCORDING TO 10 MS AND HALF SLOT TIMER ******
         ************************************************************************/

        // put sleep duration to maximum value
        sleep_duration = (rwip_env.ext_wakeup_enable) ? MAX_SLEEP_DURATION_EXTERNAL_WAKEUP : MAX_SLEEP_DURATION_PERIODIC_WAKEUP;

        // check if 10ms timer is active
        if(rwip_env.timer_10ms_target != RWIP_INVALID_TARGET_TIME)
        {
            int32_t duration = CLK_DIFF(current_time.hs, rwip_env.timer_10ms_target);
            // update sleep duration to minimum value
            sleep_duration = co_min_s(sleep_duration, duration);
        }

        // check if Half slot timer is active
        if(rwip_env.timer_hs_target != RWIP_INVALID_TARGET_TIME)
        {
            int32_t duration = CLK_DIFF(current_time.hs, rwip_env.timer_hs_target);
            // update sleep duration to minimum value
            sleep_duration = co_min_s(sleep_duration, duration);
        }

        // check if Half us timer is active
        if(rwip_env.timer_hus_target != RWIP_INVALID_TARGET_TIME)
        {
            int32_t duration = CLK_DIFF(current_time.hs, rwip_env.timer_hus_target);
            // update sleep duration to minimum value
            sleep_duration = co_min_s(sleep_duration, duration);
        }

        // A timer ISR is not yet handled or will be raised soon
        // note the sleep duration could be negative, that's why it's useful to check if a minimum requirement is ok
        // at least one half slot.
        if(sleep_duration <= RWIP_MINIMUM_SLEEP_TIME)
        {
            break;
        }

        DBG_SWDIAG(SLEEP, ALGO, 3);

        /************************************************************************
         **************           CHECK SLEEP TIME                 **************
         ************************************************************************/
        sleep_duration-=RWIP_MINIMUM_SLEEP_TIME;
        sleep_duration = rwip_slot_2_lpcycles(sleep_duration);

        // check if sleep duration is sufficient according to wake-up delay
        // HW issue, if sleep duration = max(twosc,twext) + 1 the HW never wakes up, so we have to ensure that at least
        // sleep duration > max(twosc,twext) + 1 (all in lp clk cycle)
        if(sleep_duration < rwip_env.lp_cycle_wakeup_delay * 2)
        {
            break;
        }

        DBG_SWDIAG(SLEEP, ALGO, 4);

        #if (H4TL_SUPPORT)
        /************************************************************************
         **************                 CHECK TL                   **************
         ************************************************************************/
        // Try to switch off TL
        if (!h4tl_stop())
        {
            sleep_res = RWIP_ACTIVE;
            break;
        }
        #endif //H4TL_SUPPORT

        DBG_SWDIAG(SLEEP, FUNC, 0);
        sleep_res = RWIP_DEEP_SLEEP;

        TRC_REQ_SLEEP();

        /************************************************************************
         **************          PROGRAM CORE DEEP SLEEP           **************
         ************************************************************************/
        #if (BT_EMB_PRESENT)
        // Put BT core into deep sleep
        rwbt_sleep_enter();
        #endif // (BT_EMB_PRESENT)

        #if (BLE_EMB_PRESENT)
        // Put BLE core into deep sleep
        rwble_sleep_enter();
        #endif // (BLE_EMB_PRESENT)
     //   bk_printf("s:%d,%d\r\n",sleep_duration,sleep_duration1);
        // Program wake-up time
        ip_deepslwkup_set(sleep_duration);

        // Mask all interrupts except sleep IRQ
        ip_intcntl1_set(IP_SLPINTMSK_BIT);

        // Clear possible pending IRQs
        ip_intack1_clear(0xFFFFFFFF);
        


        if(!rwip_env.ext_wakeup_enable)
        {
            ip_deepslcntl_extwkupdsb_setf(1);
        }

        led_set(2);

        DBG_SWDIAG(SLEEP, SLEEP, 1);

        /************************************************************************
         **************               SWITCH OFF RF                **************
         ************************************************************************/
        rwip_rf.sleep();
        
        rwip_prevent_sleep_set(RW_BLE_ACTIVE_MODE) ;
        #endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

    } while(0);

    if(sleep_res != RWIP_DEEP_SLEEP)
    {
        DBG_SWDIAG(SLEEP, FUNC, 0);
    }
    return sleep_res;
}

void rwip_driver_init(bool reset)
{
    #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    #if (PLF_NVDS) 
    uint8_t length;
    #endif
    uint8_t sleep_enable = 0;
    uint8_t ext_wakeup_enable;
    #if (BT_DUAL_MODE)
    uint8_t diag_cfg[PARAM_LEN_DIAG_DM_HW];
    #endif // (BT_DUAL_MODE)

    if(!reset)
    {
        // Register AES event
        ke_event_callback_set(KE_EVENT_AES_END, &rwip_crypt_evt_handler);
    }

    // initialize environment
    rwip_env.prevent_sleep     = 0;
    // clear target timer
    rwip_env.timer_10ms_target = RWIP_INVALID_TARGET_TIME;
    rwip_env.timer_hs_target   = RWIP_INVALID_TARGET_TIME;
    rwip_env.timer_hus_target  = RWIP_INVALID_TARGET_TIME;

    if(reset)
    {
        // Reset the IP core
        ip_rwdmcntl_master_soft_rst_setf(1);
        while(ip_rwdmcntl_master_soft_rst_getf());
    }

    // Enable default common interrupts
    ip_intcntl1_set(IP_FIFOINTMSK_BIT | IP_CRYPTINTMSK_BIT | IP_ERRORINTMSK_BIT | IP_SWINTMSK_BIT);

    #if (BT_DUAL_MODE)
    // Read diagport configuration from NVDS
    length = PARAM_LEN_DIAG_DM_HW;
    if(rwip_param.get(PARAM_ID_DIAG_DM_HW, &length, diag_cfg) == PARAM_OK)
    {
        ip_diagcntl_pack(1, diag_cfg[3], 1, diag_cfg[2], 1, diag_cfg[1], 1, diag_cfg[0]);
    }
    else
    {
        ip_diagcntl_set(0);
    }
    #endif // (BT_DUAL_MODE)
#if (PLF_NVDS)
    // Activate deep sleep feature if enabled in NVDS and in reset mode
    length = PARAM_LEN_SLEEP_ENABLE;
    if(!reset || rwip_param.get(PARAM_ID_SLEEP_ENABLE, &length, &sleep_enable) != PARAM_OK)
    {
        sleep_enable = 0;
    }
#endif    
    sleep_enable = 1;
    // check is sleep is enabled
    if(sleep_enable != 0)
    {
        uint16_t twext, twosc, twrm;
#if (PLF_NVDS)
        // Set max sleep duration depending on wake-up mode
        if(rwip_param.get(PARAM_ID_EXT_WAKEUP_ENABLE, &length, &ext_wakeup_enable) != PARAM_OK)
        {
            ext_wakeup_enable = 0;
        }
#endif        
        ext_wakeup_enable = 1;
        rwip_env.ext_wakeup_enable = (ext_wakeup_enable != 0) ? true : false;
#if (PLF_NVDS)
        // Set max sleep duration depending on wake-up mode
        length = sizeof(rwip_env.sleep_algo_dur);
        if(rwip_param.get(PARAM_ID_SLEEP_ALGO_DUR, &length, (uint8_t*) &rwip_env.sleep_algo_dur) != PARAM_OK)
        {
            // set a default duration: 200 us ==> 400 half us
            rwip_env.sleep_algo_dur = 400;
        }
#else
        rwip_env.sleep_algo_dur = 400;        
#endif
        // Initialize sleep parameters
        rwip_env.sleep_acc_error   = 0;
#if (PLF_NVDS)
        // Get TWrm from NVDS
        length = sizeof(uint16_t);
        if (rwip_param.get(PARAM_ID_RM_WAKEUP_TIME, &length, (uint8_t*)&twrm) != PARAM_OK)
        {
            // Set default values : 625 us
            twrm = 1500;//SLEEP_RM_WAKEUP_DELAY;
        }
#else
        twrm = 1500;
#endif
        
#if (PLF_NVDS)
        // Get TWosc from NVDS
        length = sizeof(uint16_t);
        if (rwip_param.get(PARAM_ID_OSC_WAKEUP_TIME, &length, (uint8_t*)&twosc) != PARAM_OK)
        {
            // Set default values : 5 ms
            twosc = 1500;//SLEEP_OSC_NORMAL_WAKEUP_DELAY;
        }
#else
        twosc = 1500;
#endif

        #if (PLF_NVDS)
        // Get TWext from NVDS
        length = sizeof(uint16_t);
        if (rwip_param.get(PARAM_ID_EXT_WAKEUP_TIME, &length, (uint8_t*)&twext) != PARAM_OK)
        {
            // Set default values : 5 ms
            twext = 1500;//SLEEP_OSC_EXT_WAKEUP_DELAY;
        }
        #else
        twext = 1500;
        #endif

         bk_printf("twrm:%d us,twosc:%d us,twext:%d us\r\n",twrm,twosc,twext);
        twrm  = rwip_us_2_lpcycles(twrm);
        twosc = rwip_us_2_lpcycles(twosc);
        twext = rwip_us_2_lpcycles(twext);
        
       
        
        // Program register
        ip_enbpreset_pack(twext, twosc, twext);

        // Configure wake up delay to the highest parameter
        twext = co_max(twext,twrm);
        twext = co_max(twext,twosc);
        bk_printf("lpcycles twrm:%d,twosc:%d,twext:%d\r\n",twrm,twosc,twext);
        // Store wake-up delay in lp cycles
        rwip_env.lp_cycle_wakeup_delay = twext;

        // Set the external wakeup parameter
        ip_deepslcntl_extwkupdsb_setf(rwip_env.ext_wakeup_enable ? false : true);
    }
    else
    {
        // ensure that we will never enter in deep sleep
        rwip_prevent_sleep_set(RW_PLF_DEEP_SLEEP_DISABLED);
    }

    #if BT_DUAL_MODE
    // Set BTDM arbiter
    //TODO: restore //ip_prioscharb_pack(/*blepriomode*/ 1, /*bredrpriomode*/ 1);
    #endif //BT_DUAL_MODE
    #elif (BLE_HOST_PRESENT)
    // initialize environment
    rwip_env.prevent_sleep     = 0;
    rwip_env.timer_10ms_target = RWIP_INVALID_TARGET_TIME;
    // enable timer
    timer_enable(true);
    #endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)
}









void rwip_isr(void)
{
    DBG_SWDIAG(ISR, RWIP, 1);

    // Check interrupt status and call the appropriate handlers
    uint32_t irq_stat      = ip_intstat1_get();

   // bk_printf("rwip:%x\r\n", irq_stat );
    // General purpose timer interrupt - half slot accuracy
    if (irq_stat & IP_FINETGTINTSTAT_BIT) //0x00000010
    {
        DBG_SWDIAG(IP_ISR, FINETGTINT, 1);
        // Clear the interrupt
        ip_intack1_finetgtintack_clearf(1);

        // handles half slot timer target
        rwip_timer_hs_handler();

        DBG_SWDIAG(IP_ISR, FINETGTINT, 0);
    }

    // General purpose timer interrupt - half us accuracy
    if (irq_stat & IP_TIMESTAMPTGT1INTSTAT_BIT)//0x00000020
    {
        DBG_SWDIAG(IP_ISR, TIMESTAMPINT, 1);
        // Clear the interrupt
        ip_intack1_timestamptgt1intack_clearf(1);

        // handles half slot timer target
        rwip_timer_hus_handler();

        DBG_SWDIAG(IP_ISR, TIMESTAMPINT, 0);
    }

    // Clock
    if (irq_stat & IP_CLKNINTSTAT_BIT) // clock interrupt 0x00000001
    {
        DBG_SWDIAG(IP_ISR, CLKNINT, 1);

        // Ack clock interrupt
        ip_intack1_clknintack_clearf(1);


        if(rwip_env.prevent_sleep & RW_WAKE_UP_ONGOING)
        {
            // Handle end of wake-up
            rwip_wakeup_end();
        }
        #if (BT_EMB_PRESENT)
        else // BT uses clock IRQ to program ACL frames
        {
            // Call Scheduling Programmer
            sch_prog_clk_isr();
        }
        #endif //BT_EMB_PRESENT

        DBG_SWDIAG(IP_ISR, CLKNINT, 0);
    }

    // FIFO
    if (irq_stat & IP_FIFOINTSTAT_BIT) // FIFO interrupt 0x00008000
    {
        DBG_SWDIAG(IP_ISR, FIFOINT, 1);

        // Call scheduling programmer
        sch_prog_fifo_isr();

        // Ack FIFO interrupt
        ip_intack1_fifointack_clearf(1);

        DBG_SWDIAG(IP_ISR, FIFOINT, 0);
    }

    if (irq_stat & IP_SLPINTSTAT_BIT) //0x00000002
    {
        DBG_SWDIAG(IP_ISR, SLPINT, 1);

        // ack Sleep wakeup interrupt
        ip_intack1_slpintack_clearf(1);
        // Handle wake-up
        rwip_wakeup();
       // bk_printf("rwip_wakeup\r\n");
  
        rwip_prevent_sleep_clear(RW_BLE_ACTIVE_MODE);
        DBG_SWDIAG(IP_ISR, SLPINT, 0);
    }

    // General purpose timer interrupt
    if (irq_stat & IP_TIMESTAMPTGT2INTSTAT_BIT)//0x00000040
    {
        DBG_SWDIAG(IP_ISR, GROSSTGTINT, 1);

        // Clear the interrupt
        ip_intack1_timestamptgt2intack_clearf(1);

        // handles 10 ms timer target
        rwip_timer_10ms_handler();

        DBG_SWDIAG(IP_ISR, GROSSTGTINT, 0);
    }

    // Encryption interrupt
    if (irq_stat & IP_CRYPTINTSTAT_BIT)//0x00000004
    {
        DBG_SWDIAG(IP_ISR, CRYPTINT, 1);

        ip_intack1_cryptintack_clearf(1);

        // call the crypto ISR handler
        rwip_crypt_isr_handler();

        DBG_SWDIAG(IP_ISR, CRYPTINT, 0);
    }

    // SW interrupt
    if (irq_stat & IP_SWINTSTAT_BIT)//0x00000008
    {
        DBG_SWDIAG(IP_ISR, SWINT, 1);
        // Clear the interrupt
        ip_intack1_swintack_clearf(1);

        // call SW interrupt handler
        rwip_sw_int_handler();

        DBG_SWDIAG(IP_ISR, SWINT, 0);
    }

    DBG_SWDIAG(ISR, RWIP, 0);
}



///@} RW

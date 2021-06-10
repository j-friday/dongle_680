/**
 ****************************************************************************************
 *
 * @file boot_vectors.s
 *
 * @brief ARM Exception Vectors table.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 * $Rev:  $
 *
 ****************************************************************************************
 */

.text
    .align  4
    .global vectors, boot_breakpoint
    .type   vectors, function

vectors:
    # reset handler
    B       boot_reset
    # undefined handler
    B       boot_undefined
    # SWI handler
    B       boot_swi
    # Prefetch error handler
    B       boot_pabort
    # abort handler
    B       boot_dabort
    # reserved vector
    B       boot_reserved
    # irq
    B       intc_irq
    # fiq
    B       intc_fiq

boot_breakpoint:
    # If set to 0 by host before getting out of reset, the cpu
    # will loop and light the leds indefinitely
    .word   1

;/**
; ****************************************************************************************
; *
; * @file boot_handlers.s
; *
; * @brief ARM Exception Vector handler functions.
; *
; * Copyright (C) RivieraWaves 2009-2015
; *
; * $Rev: 7583 $
; *
; ****************************************************************************************
; */

    IF {CPU} /= "Cortex-M3"
    IF {CPU} /= "Cortex-M1"
    IF {CPU} /= "Cortex-M0"

    ;Export pointers to the vector handlers.
    EXPORT   boot_reset
    EXPORT   boot_undefined
    EXPORT   boot_swi
    EXPORT   boot_pabort
    EXPORT   boot_dabort
    EXPORT   boot_reserved

    IMPORT   rw_main

;/* ========================================================================
; *                                Constants
; * ======================================================================== */

BOOT_MODE_MASK EQU 0x1F

BOOT_MODE_USR  EQU 0x10
BOOT_MODE_FIQ  EQU 0x11
BOOT_MODE_IRQ  EQU 0x12
BOOT_MODE_SVC  EQU 0x13
BOOT_MODE_ABT  EQU 0x17
BOOT_MODE_UND  EQU 0x1B
BOOT_MODE_SYS  EQU 0x1F

I_BIT          EQU 0x80
F_BIT          EQU 0x40

BOOT_COLOR_UNUSED  EQU 0xAAAAAAAA      ; Pattern to fill UNUSED stack
BOOT_COLOR_SVC     EQU 0xBBBBBBBB      ; Pattern to fill SVC stack
BOOT_COLOR_IRQ     EQU 0xCCCCCCCC      ; Pattern to fill IRQ stack
BOOT_COLOR_FIQ     EQU 0xDDDDDDDD      ; Pattern to fill FIQ stack

;/* ========================================================================
; *                                Macros
; * ======================================================================== */

;/* ========================================================================
;/**
; * Macro for switching ARM mode
; */
    MACRO
$x  BOOT_CHANGE_MODE $newMode
        MRS   R0, CPSR
        BIC   R0, R0, #BOOT_MODE_MASK
        ORR   R0, R0, #BOOT_MODE_$newMode
        MSR   CPSR_c, R0
    MEND


;/* ========================================================================
;/**
; * Macro for setting the stack
; */
    MACRO
$x  BOOT_SET_STACK $stackName
        LDR   R0, boot_stack_base_$stackName
        LDR   R2, boot_stack_len_$stackName
        ADD   R1, R0, R2
        MOV   SP, R1        ; Set stack pointer

        LDR   R2, =BOOT_COLOR_$stackName

90      CMP   R0, R1        ; End of stack?
        STRLT R2, [r0]      ; Colorize stack word
        ADDLT R0, R0, #4
        BLT   %B90          ; branch to previous local label

    MEND


   PRESERVE8
   AREA  ||.text||, CODE, READONLY
;/* ========================================================================
; *                                Globals
; * ======================================================================== */

;/* ========================================================================
;/**
; * CP15 DTCM Control Reg settings
; */
boot_Cp15DtcmReg
    DCD 0x01010001


;/* ========================================================================
;/**
; * CP15 ITCM Control Reg settings
; */
boot_Cp15ItcmReg
    DCD 0x01000001

;/* ========================================================================
;/**
; * ROM
; */
   IMPORT    |Load$$RAM_DATA$$Base|           ; Base of ROM data
rom_base
    DCD |Load$$RAM_DATA$$Base|
   ;/* ========================================================================
;/**
; * RAM to initialize with rom data
; */
    IMPORT    |Image$$RAM_DATA$$Base|
ram_base
    DCD |Image$$RAM_DATA$$Base|
    IMPORT    |Image$$RAM_DATA$$Length|
ram_length
    DCD |Image$$RAM_DATA$$Length|

;/* ========================================================================
;/**
; * RAM_BSS
; */
    IMPORT   |Image$$RAM_BSS$$ZI$$Base|
ram_bss_base
    DCD |Image$$RAM_BSS$$ZI$$Base|

    IMPORT   |Image$$RAM_BSS$$ZI$$Length|
ram_bss_length
    DCD |Image$$RAM_BSS$$ZI$$Length|

;/* ========================================================================
;/**
; * Unused (ABT, UNDEFINED, SYSUSR) Mode
; */
    IMPORT   |Image$$RAM_STACK_UNUSED$$Base|
boot_stack_base_UNUSED
    DCD |Image$$RAM_STACK_UNUSED$$Base|

    IMPORT   |Image$$RAM_STACK_UNUSED$$ZI$$Length|
boot_stack_len_UNUSED
    DCD |Image$$RAM_STACK_UNUSED$$ZI$$Length|


;/* ========================================================================
;/**
; * IRQ Mode
; */
    IMPORT   |Image$$RAM_STACK_IRQ$$Base|
boot_stack_base_IRQ
    DCD |Image$$RAM_STACK_IRQ$$Base|

    IMPORT   |Image$$RAM_STACK_IRQ$$ZI$$Length|
boot_stack_len_IRQ
    DCD |Image$$RAM_STACK_IRQ$$ZI$$Length|


;/* ========================================================================
;/**
; * Supervisor Mode
; */
    IMPORT   |Image$$RAM_STACK_SVC$$Base|
boot_stack_base_SVC
    DCD |Image$$RAM_STACK_SVC$$Base|

    IMPORT   |Image$$RAM_STACK_SVC$$ZI$$Length|
boot_stack_len_SVC
    DCD |Image$$RAM_STACK_SVC$$ZI$$Length|


;/* ========================================================================
;/**
; * FIQ Mode
; */
    IMPORT   |Image$$RAM_STACK_FIQ$$Base|
boot_stack_base_FIQ
    DCD |Image$$RAM_STACK_FIQ$$Base|

    IMPORT   |Image$$RAM_STACK_FIQ$$ZI$$Length|
boot_stack_len_FIQ
    DCD |Image$$RAM_STACK_FIQ$$ZI$$Length|


;/* ========================================================================
; *                                Functions
; * ======================================================================== */

;/* ========================================================================
;/**
; * Function to handle reset vector
; */
boot_reset
    ; * ==================
    ; Setup all stacks
    ; Note: Sys and Usr mode are not used
    BOOT_CHANGE_MODE SYS
    BOOT_SET_STACK   UNUSED
    BOOT_CHANGE_MODE ABT
    BOOT_SET_STACK   UNUSED
    BOOT_CHANGE_MODE UND
    BOOT_SET_STACK   UNUSED
    BOOT_CHANGE_MODE IRQ
    BOOT_SET_STACK   IRQ
    BOOT_CHANGE_MODE FIQ
    BOOT_SET_STACK   FIQ
    BOOT_CHANGE_MODE SVC
    BOOT_SET_STACK   SVC


    ; Stay in Supervisor Mode



    ; Recopy rom to ram
    LDR     R0, rom_base                 ; Get base of ROM data
    LDR     R1, ram_base                 ; Get base of RAM to initialise
    LDR     R3, ram_length               ; Get length of RAM to initialise

copy_rom_ram
    CMP     R3, #0                     ; Copy init data, init of RW area
    LDRGT   R2, [R0], #4               ; Load data
    STRGT   R2, [R1], #4               ; Store data
    SUBGT   R3, R3, #4                 ; Decrement RW length
    BGT     copy_rom_ram               ; For each word

    ; Init the BSS section
    LDR     R0, ram_bss_base
    LDR     R1, ram_bss_length
    MOV     R2, #0
    MOV     R3, #0
    MOV     R4, #0
    MOV     R5, #0
init_bss_loop
    SUBS    R1, R1, #16
    STMCSIA R0!, {R2, R3, R4, R5}
    BHI     init_bss_loop
    LSLS    R1, R1, #29
    STMCSIA R0!, {R4, R5}
    STRMI   R3, [R0]

    ; * ==================
    ; Clear Registers
    MOV R0, #0
    MOV R1, #0
    MOV R2, #0
    MOV R3, #0
    MOV R4, #0
    MOV R5, #0
    MOV R6, #0
    MOV R7, #0
    MOV R8, #0
    MOV R9, #0
    MOV R10, #0
    MOV R11, #0
    MOV R12, #0

; Now safe to enable interrupts, so do this and remain in SVC mode
    MOV        r0, #BOOT_MODE_SVC:OR:I_BIT:OR:F_BIT ; IRQ and FIQ still disabled
    MSR        CPSR_c, r0


    BL rw_main


    ; If For some reason main returns (which it shouldn't)
    ; Just loop here and wait for the watchdog to reset
_boot_reset_loop
    B _boot_reset_loop


;/* ========================================================================
;/**
; * Function to handle undefined vector
; */
boot_undefined

    B boot_undefined

;/* ========================================================================
;/**
; * Function to handle software interrupt vector
; */
boot_swi

    B boot_swi


;/* ========================================================================
;/**
; * Function to handle Prefetch Abort vector
; */
boot_pabort

    B boot_pabort

;/* ========================================================================
;/**
; * Function to handle Data Abort vector
; */
boot_dabort

    B boot_dabort

;/* ========================================================================
;/**
; * Function to handle Reserved vector
; */
boot_reserved

    B boot_reserved
    SUBS PC, LR, #4

    ENDIF
    ENDIF
    ENDIF

    END

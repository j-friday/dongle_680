

MODE_USR        EQU     0x10
MODE_FIQ        EQU     0x11
MODE_IRQ        EQU     0x12
MODE_SVC        EQU     0x13
MODE_ABT        EQU     0x17
MODE_UND        EQU     0x1B
MODE_SYS        EQU     0x1F ; available on ARM Arch 4 and later

I_BIT           EQU     0x80 ; when I bit is set, IRQ is disabled
F_BIT           EQU     0x40 ; when F bit is set, FIQ is disabled

; Amount of memory (in bytes) allocated for stacks for each ARM mode
; Tailor thoese values to your application needs
USR_STACK_LEGTH     EQU         1024     ;64
SVC_STACK_LEGTH     EQU         64
FIQ_STACK_LEGTH     EQU         1024      ;16
IRQ_STACK_LEGTH     EQU         1024     ;64
ABT_STACK_LEGTH     EQU         64
UND_STACK_LEGTH     EQU         64

OFFSET_FIQ_STACK         EQU     0                               
OFFSET_IRQ_STACK         EQU     OFFSET_FIQ_STACK + FIQ_STACK_LEGTH
OFFSET_SVC_STACK         EQU     OFFSET_IRQ_STACK + IRQ_STACK_LEGTH
OFFSET_ABT_STACK         EQU     OFFSET_SVC_STACK + SVC_STACK_LEGTH
OFFSET_UND_STACK         EQU     OFFSET_ABT_STACK + ABT_STACK_LEGTH
OFFSET_USR_STACK         EQU     OFFSET_UND_STACK + UND_STACK_LEGTH    


    IMPORT  Reset
    IMPORT  intc_fiq
    IMPORT  intc_irq

    PRESERVE8


    EXPORT  InitStack
    EXPORT  Vectors


    CODE32

    AREA    Startup,CODE,READONLY

;interrupt vectors table
Vectors
        LDR     PC, ResetAddr
        LDR     PC, UndefinedAddr
        LDR     PC, SWI_Addr
        LDR     PC, PrefetchAddr
        LDR     PC, DataAbortAddr
        DCD     0xb9205f80
        LDR     PC, IRQ_Addr
        LDR     PC, FIQ_Addr

ResetAddr           DCD     Reset
UndefinedAddr       DCD     Undefined
SWI_Addr            DCD     SoftwareInterrupt
PrefetchAddr        DCD     PrefetchAbort
DataAbortAddr       DCD     DataAbort
Nouse               DCD     0
IRQ_Addr            DCD     IRQ_Handler
FIQ_Addr            DCD     FIQ_Handler

;*******************************************************************************
;* Function Name  : UndefinedHandler
;* Description    : This function called when undefined instruction exception
;*                  is entered.
;* Input          : none
;* Output         : none
;*******************************************************************************
Undefined
        B       Undefined

;*******************************************************************************
;* Function Name  : SWIHandler
;* Description    : This function called when SWI instruction executed.
;* Input          : none
;* Output         : none
;*******************************************************************************
SoftwareInterrupt
        B       SoftwareInterrupt

PrefetchAbort
        B       PrefetchAbort

;*******************************************************************************
;* Function Name  : DataAbortHandler
;* Description    : This function is called when Data Abort exception is entered.
;* Input          : none
;* Output         : none
;*******************************************************************************
DataAbort
        B       DataAbort

;*******************************************************************************
;* Function Name  : IRQHandler
;* Description    : This function called when IRQ exception is entered.
;* Input          : none
;* Output         : none
;*******************************************************************************
IRQ_Handler
        STMFD   SP!, {R0-R12, LR}
        BL      intc_irq
        LDMFD   SP!, {R0-R12, LR}
        SUBS    PC,  LR,  #4


;*******************************************************************************
;* Function Name  : FIQHandler
;* Description    : This function is called when FIQ exception is entered.
;* Input          : none
;* Output         : none
;*******************************************************************************
FIQ_Handler
        STMFD   SP!, {R0-R7, LR}
        BL      intc_fiq
        LDMFD   SP!, {R0-R7, LR}
        SUBS    PC,  LR,  #4


InitStack
;        IMPORT  FirstUnusedRAM
        MOV     R0, LR

; enter each mode in turn and set up the stack pointer
        LDR R1, STACK_BASE

;init SV stack
        MSR     CPSR_c, #0x13:OR:I_BIT:OR:F_BIT
        SUB     sp, r1, #OFFSET_SVC_STACK

;init IRQ stack
        MSR     CPSR_c, #0x12:OR:I_BIT:OR:F_BIT 
        SUB     sp, r1, #OFFSET_IRQ_STACK
        
;init FIQ stack
        MSR     CPSR_c, #0x11:OR:I_BIT:OR:F_BIT 
        SUB     sp, r1, #OFFSET_FIQ_STACK
        
;init abort stack
        MSR     CPSR_c, #0x17:OR:I_BIT:OR:F_BIT 
        SUB     sp, r1, #OFFSET_ABT_STACK
        
;init undef stack
        MSR     CPSR_c, #0x1b:OR:I_BIT:OR:F_BIT 
        SUB     sp, r1, #OFFSET_UND_STACK
        
;init user stack
        MSR     CPSR_c, #0x1f:OR:I_BIT:OR:F_BIT 
        SUB     sp, r1, #OFFSET_USR_STACK
        BX      R0 ; MOV     PC, R0


STACK_BASE      DCD      0x420000

;BC StackUsr           DCD     UsrStackSpace + (USR_STACK_LEGTH - 1) * 4
;BC StackSvc           DCD     SvcStackSpace + (SVC_STACK_LEGTH - 1)* 4
;BC StackIrq           DCD     IrqStackSpace + (IRQ_STACK_LEGTH - 1)* 4
;BC StackFiq           DCD     FiqStackSpace + (FIQ_STACK_LEGTH - 1)* 4
;BC StackAbt           DCD     AbtStackSpace + (ABT_STACK_LEGTH - 1)* 4
;BC StackUnd           DCD     UndtStackSpace + (UND_STACK_LEGTH - 1)* 4


;BC         AREA    MyStacks, DATA, NOINIT, ALIGN=2
;BC UsrStackSpace      SPACE   USR_STACK_LEGTH * 4  ;
;BC SvcStackSpace      SPACE   SVC_STACK_LEGTH * 4  ;
;BC IrqStackSpace      SPACE   IRQ_STACK_LEGTH * 4  ;
;BC FiqStackSpace      SPACE   FIQ_STACK_LEGTH * 4  ;
;BC AbtStackSpace      SPACE   ABT_STACK_LEGTH * 4  ;
;BC UndtStackSpace     SPACE   UND_STACK_LEGTH * 4  ;

    END
;/*********************************************************************************************************
;**                            End Of File
;********************************************************************************************************/

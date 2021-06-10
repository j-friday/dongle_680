
;Define address for ram base and Depth 32K Bytes
RAM_BASE   EQU 0x00400000
RAM_LIMIT  EQU 0x0041FFFF

    IMPORT  __main                          ;C entry point�J�f 
    IMPORT  InitStack                       ;init stack
    ;IMPORT  arm_sys_init
    
    EXPORT  Reset
    EXPORT __rt_div0
    EXPORT __user_initial_stackheap
    
;
    CODE32
    AREA    Init,CODE,READONLY
    

Reset
; Stack Init              
        BL      InitStack 
        
;Flash Init
    MOV     r0, #0;
    LDR     R0, =0x0080005c;
	MOV     R1, #0 ;
	STR     R1, [R0,#0];		//open PLL
		
	LDR		R0, =0x0080201c;
	LDR		R1, =0x04000001;
	STR		R1,[R0,#0];		//flash_clk select DPLL 48M
		
		
        ;BL      arm_sys_init 

;Enable IRQ and FIQ Interrupt
	    MRS		r0,CPSR		
	    bic		r0,r0,#0xC0		
	    MSR     CPSR_c,r0

; --- Branch to C Library entry point                                        
        B       __main          ; use B not BL, because an application will never return this way

; �⺯����ʼ���Ѻ�ջ������ɾ��
__user_initial_stackheap    
        LDR     r0,=bottom_of_heap
        BX      lr ;MOV   pc,lr

; ������������Ϊ0�������������ԭʼ��__rt_div0����Ŀ������С
__rt_div0
        B       __rt_div0


        AREA    Myheap, DATA, NOINIT, ALIGN=2
bottom_of_heap     SPACE   256  ;

    END
;/*********************************************************************************************************
;**                            End Of File
;********************************************************************************************************/

;*
;******************************************************************************
;* @file       sp_delay_gcc.c
;* @author     YTom
;* @version    v0.0
;* @date       2018.Dec.12
;* @brief      Assembly delay module GNU version.
;******************************************************************************
;* @license
;*
;******************************************************************************
;*/

    PRESERVE8   ; Preserve 8 byte stack alignment
    THUMB       ; indicate THUMB code is used
    AREA    sp_delay, CODE, READONLY


    ;brief  delay 1us for 168MHz
    ;1us = 168clock*1.25[https://blog.csdn.net/lintax/article/details/83047326] = 210cmd
    EXPORT  delay_us
delay_us
    ; Input : R0
    MOV     R1, #0x2A       ; R1 = 42:0x2A
    MUL     R0, R0, R1      ; R0 = R0*R1
delay_us_loop
    SUBS    R0, R0, #0x01   ; R0--                  ##1clock
    CMP     R0, #0x00       ; if(R0==0)             ##1clock
    BNE     delay_us_loop   ; goto delay_us_loop;   ##1clock+2clock
    NOP
    BX      LR              ; return
    
    
    ;brief  delay 1ms for 168MHz
    ;1ms = 168kclock*1.25 = 210kcmd (-5)
    EXPORT  delay_ms
delay_ms
    ; Input : R0
    MOV     R1, #0xa410     ; R1 = 42000:0xa410
    MUL     R0, R0, R1      ; R0 = R0*R1
delay_ms_loop
    SUBS    R0, R0, #0x01   ; R0--
    CMP     R0, #0x00       ; if(R0==0)
    BNE     delay_ms_loop   ; goto delay_us_loop;
    NOP
    BX      LR              ; return
    
    
    ;brief  delay for DMA transfer
    ;1byte ~= 1cpu_clock (5cpu_clock here)
    EXPORT  dma_delay
dma_delay
    ; Input : R0
    CMP     R0, #0x00       ; if(R0==0)
    BNE     dma_delay_loop  ; goto delay_us_loop;
dma_delay_loop
    SUBS    R0, R0, #0x01   ; R0--
    CMP     R0, #0x00       ; if(R0==0)
    BNE     dma_delay_loop  ; goto delay_us_loop;
    BX      LR              ; return
    
    
    
    END

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/

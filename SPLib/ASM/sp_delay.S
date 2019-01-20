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
    EXPORT  delay_us
delay_us
    ; Input : R0
    MOV     R1, #0x37       ; R1 = 55
    MUL     R0, R0, R1      ; R0 = R0*R1
delay_us_loop
    SUBS    R0, R0, #0x01   ; R0--
    CMP     R0, #0x00       ; if(R0==0)
    BNE     delay_us_loop   ; goto delay_us_loop;
    NOP
    BX      LR              ; return
    
    
    ;brief  delay 1ms for 168MHz
    EXPORT  delay_ms
delay_ms
    ; Input : R0
    MOV     R1, #0xdabf     ; R1 = 55998
    MUL     R0, R0, R1      ; R0 = R0*R1
delay_ms_loop
    SUBS    R0, R0, #0x01   ; R0--
    CMP     R0, #0x00       ; if(R0==0)
    BNE     delay_ms_loop   ; goto delay_us_loop;
    NOP
    BX      LR              ; return
    
    
    END

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/

/**
  ******************************************************************************
  * @file       sp_delay_gcc.c
  * @author     YTom
  * @version    v0.0
  * @date       2018.Dec.12
  * @brief      Assembly delay module GNU version.
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb

.global  delay_us
.global  delay_ms

/**
 * @brief delay 1us for 168MHz
 * @note  1us = 168clock*1.25[https://blog.csdn.net/lintax/article/details/83047326] = 210cmd
*/
.section  .text
.type  delay_us, %function
delay_us:                       /* Entry to the function "main" */
    /* Input : r0 */
    mov     r1, #0x2A           /* R1 = 42:0x2A */
    mul     r0, r0, r1          /* r0 = r0*R1 */
delay_us_loop:
    subs    r0, r0, #0x01       /* r0-- */
    cmp     r0, #0x00           /* if(R0==0) */
    bne     delay_us_loop       /* goto delay_us_loop */
    nop
    bx      lr                  /* return */

/**
 * @brief  delay 1ms for 168MHz
 * @param  None
 * @retval : None
*/
.section  .text
.type  delay_ms, %function
delay_ms:
    /* Input : r0 */
    mov     r1, #0xa410         /* R1 = 42000:0xa410 */
    mul     r0, r0, r1          /* r0 = r0*R1 */
delay_ms_loop:
    subs    r0, r0, #0x01       /* r0-- */
    cmp     r0, #0x00           /* if(R0==0) */
    bne     delay_ms_loop       /* goto delay_us_loop */
    nop
    bx      lr                  /* return */

/**
 * @brief  delay for DMA transfer
 * @param  None
 * @retval : None
 * @notw   1byte ~= 1cpu_clock (5cpu_clock here)
 */
.section  .text
.type  dma_delay, %function
dma_delay:
    /* Input : r0 */
    cmp     r0, #0x00           /* if(R0!=0) */
    bne     delay_ms_loop       /* goto delay_us_loop */
dma_delay_loop:
    subs    r0, r0, #0x01       /* r0-- */
    cmp     r0, #0x00           /* if(R0==0) */
    bne     delay_ms_loop       /* goto delay_us_loop */
    nop
    bx      lr                  /* return */

.end

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/

/**
  ******************************************************************************
  * @file       template.h
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Oct.21
  * @brief      project header template
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SP_PID_H
#define __SP_PID_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <math.h>
#include "sp_type.h"
#include "sp_conf.h"


#define PID_CHANGERATE_THRESHOLD        0.5f        /* Value whos change rate over this will be replaced 
                                                       by averaged recent value. */
#define PID_SOFTPID_THRESHOLD           0.3f        /* Error whos change rate lower than this will 
                                                       using soft control mode. */
#define PID_BANGBANG_THRESHOLD          0.8f        /* Error whos change rate over this will 
                                                       using bang-bang(on-off) control. */
//#define PID_BUFFERSIZE                  3           /* How many data will be save for calculating.(SHOULD >=3) */
#define PID_SYSTEMSCALE                 16000       /* System scale means what's the order of magnitude 
                                                       of the input/output value. */

#define PID_SetGainP(pid, x)       (pid)->Kp = (x)
#define PID_SetGainI(pid, x)       (pid)->Ki = (x)
#define PID_SetGainD(pid, x)       (pid)->Kd = (x)

/**
  * @brief  PID type structure
  * @note   
  */ 
typedef struct {
    float Kp, Ki, Kd;
    
    float time_stamp, delta_time;
    
    double sum_error;
    float target;                           /* Current target. */
    float inputs[3];                        /* Current detected sensor value. */
    float errors[3];                        /* Current difference value between target and input. */
    float ouputs[3];                        /* Optimilized output value. */
    
    float intergration_separation;
    
    float differential_limit;            /* Limit of differential term. */
    float intergration_limit;            /* Limit of intergration term. */
    float output_limit;                  /* Limit of output value. */
    float delta_limit;                   /* Limit of output increment. */
    
    struct {
        uint8_t     target_changed:1;
        uint8_t     initialized:1;
        uint8_t     using_output_limit:1;
        uint8_t     using_delta_limit:1;
    } ctrl_reg;
    
    struct {
        float (*input_filter)(float* data, uint8_t size, float newval, float rela_limit);
        float (*output_filter)(float* data, uint8_t size, float newval, float rela_limit);
    } functions;
} PID_Type;



/** @defgroup PID User API
  * @brief    PID initialization and implement
  * @{
  */
  
/**
  * @brief  Init PID controller with limitations
  * @param  pid: PID controller @ref PID_Type
  * @param  lim_i: integration limit
  * @param  lim_d: differential limit
  * @param  lim_out: output limit
  * @param  dt: intergration and differential delta time
  * @note   Limitations MUST be given
  */ 
void PID_ControllerInit(PID_Type* pid, float lim_i, float lim_d, float lim_out);
void PID_ControllerInit_withDt(PID_Type* pid, float lim_i, float lim_d, float lim_out, float dt);

/**
  * @brief  PID controller driver for calculating controller output
  * @param  pid: PID controller @ref PID_Type
  * @param  input: current value for comparision
  * @retval PID controller output
  */ 
float PID_ControllerDriver(PID_Type* pid, float target, float input);
float PID_ControllerDriver_Incremental(PID_Type* pid, float target, float input, float lim_delta);

float PID_ControllerDriver_test(PID_Type* pid, float target, float input);

/**
  * @brief  Set PID controller's current target value
  * @param  pid: PID controller @ref PID_Type
  * @param  target: the target value
  */ 
void PID_SetTarget(PID_Type* pid, float target);

/**
  * @brief  Set PID controller's indexes
  * @param  pid: PID controller @ref PID_Type
  * @param  kp,ki,kd: indexed of P,I,D
  */ 
void PID_SetGains(PID_Type* pid, float kp, float ki, float kd);

/**
  * @brief  Update PID controller's limitations
  * @param  pid: PID controller @ref PID_Type
  * @param  lim_i: integration limit
  * @param  lim_d: differential limit
  * @param  lim_out: output limit
  */ 
void PID_UpdateLimits(PID_Type* pid, float lim_i, float lim_d, float lim_out);


/**
  * @}
  */
  

#ifdef __cplusplus
}
#endif

#endif /* __SP_PID_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/

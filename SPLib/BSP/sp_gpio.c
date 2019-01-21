/**
  ******************************************************************************
  * @file       sp_gpio.c
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Nov.25
  * @brief      General GPIO config module
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sp_gpio.h"


void GPIO_Config(GPIO_TypeDef* GPIOx, uint16_t Pinx, GPIOMode_TypeDef Modex, 
    GPIOOType_TypeDef OTyperx, GPIOPuPd_TypeDef PuPdx, GPIOSpeed_TypeDef Speedx) {
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin     =       Pinx;
        GPIO_InitStructure.GPIO_Mode    =       Modex;
        GPIO_InitStructure.GPIO_OType   =       OTyperx;
        GPIO_InitStructure.GPIO_Speed   =       Speedx;
        GPIO_InitStructure.GPIO_PuPd    =       PuPdx;
        GPIO_Init(GPIOx, &GPIO_InitStructure);
}

void GPIO_OUT_Config(GPIO_TypeDef* GPIOx, uint16_t Pinx, GPIOOType_TypeDef OTyperx, 
    GPIOPuPd_TypeDef PuPdx, GPIOSpeed_TypeDef Speedx) {
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin     =       Pinx;
        GPIO_InitStructure.GPIO_Mode    =       GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType   =       OTyperx;
        GPIO_InitStructure.GPIO_Speed   =       Speedx;
        GPIO_InitStructure.GPIO_PuPd    =       PuPdx;
        GPIO_Init(GPIOx, &GPIO_InitStructure);
}

void GPIO_IN_Config(GPIO_TypeDef* GPIOx, uint16_t Pinx, GPIOPuPd_TypeDef PuPdx, GPIOSpeed_TypeDef Speedx) {
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin     =       Pinx;
        GPIO_InitStructure.GPIO_Mode    =       GPIO_Mode_IN;
        GPIO_InitStructure.GPIO_OType   =       GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed   =       Speedx;
        GPIO_InitStructure.GPIO_PuPd    =       PuPdx;
        GPIO_Init(GPIOx, &GPIO_InitStructure);
}

void GPIO_AF_Config(GPIO_TypeDef* GPIOx, uint16_t Pinx, 
        GPIOOType_TypeDef OTyperx, GPIOPuPd_TypeDef PuPdx, GPIOSpeed_TypeDef Speedx) {
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin     =       Pinx;
        GPIO_InitStructure.GPIO_Mode    =       GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType   =       OTyperx;
        GPIO_InitStructure.GPIO_Speed   =       Speedx;
        GPIO_InitStructure.GPIO_PuPd    =       PuPdx;
        GPIO_Init(GPIOx, &GPIO_InitStructure);
}

void GPIO_AN_Config(GPIO_TypeDef* GPIOx, uint16_t Pinx) {
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin     =       Pinx;
        GPIO_InitStructure.GPIO_Mode    =       GPIO_Mode_AN;
        GPIO_InitStructure.GPIO_PuPd    =       GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOx, &GPIO_InitStructure);
}



/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/

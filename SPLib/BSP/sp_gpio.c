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

void GPIO_RCC(GPIO_TypeDef* GPIOx) {
#ifdef spRCC_Set_GPIOA
    if(GPIOx==GPIOA) {
        spRCC_Set_GPIOA();
    }
#endif
#ifdef spRCC_Set_GPIOB
    if(GPIOx==GPIOB) {
        spRCC_Set_GPIOB();
    }
#endif
#ifdef spRCC_Set_GPIOC
    if(GPIOx==GPIOC) {
        spRCC_Set_GPIOC();
    }
#endif
#ifdef spRCC_Set_GPIOD
    if(GPIOx==GPIOD) {
        spRCC_Set_GPIOD();
    }
#endif
#ifdef spRCC_Set_GPIOE
    if(GPIOx==GPIOE) {
        spRCC_Set_GPIOE();
    }
#endif
#ifdef spRCC_Set_GPIOF
    if(GPIOx==GPIOF) {
        spRCC_Set_GPIOF();
    }
#endif
#ifdef spRCC_Set_GPIOG
    if(GPIOx==GPIOG) {
        spRCC_Set_GPIOG();
    }
#endif
#ifdef spRCC_Set_GPIOH
    if(GPIOx==GPIOH) {
        spRCC_Set_GPIOH();
    }
#endif
#ifdef spRCC_Set_GPIOI
    if(GPIOx==GPIOI) {
        spRCC_Set_GPIOI();
    }
#endif
#ifdef spRCC_Set_GPIOJ
    if(GPIOx==GPIOJ) {
        spRCC_Set_GPIOJ();
    }
#endif
#ifdef spRCC_Set_GPIOK
    if(GPIOx==GPIOK) {
        spRCC_Set_GPIOK();
    }
#endif
}



void GPIO_Config(GPIO_TypeDef* GPIOx, uint16_t Pinx, GPIOMode_TypeDef Modex, 
    GPIOOType_TypeDef OTyperx, GPIOPuPd_TypeDef PuPdx, GPIOSpeed_TypeDef Speedx) {
        GPIO_RCC(GPIOx);
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
        GPIO_RCC(GPIOx);
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin     =       Pinx;
        GPIO_InitStructure.GPIO_Mode    =       GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType   =       OTyperx;
        GPIO_InitStructure.GPIO_Speed   =       Speedx;
        GPIO_InitStructure.GPIO_PuPd    =       PuPdx;
        GPIO_Init(GPIOx, &GPIO_InitStructure);
}

void GPIO_IN_Config(GPIO_TypeDef* GPIOx, uint16_t Pinx, GPIOPuPd_TypeDef PuPdx, GPIOSpeed_TypeDef Speedx) {
        GPIO_RCC(GPIOx);
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
        GPIO_RCC(GPIOx);
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin     =       Pinx;
        GPIO_InitStructure.GPIO_Mode    =       GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType   =       OTyperx;
        GPIO_InitStructure.GPIO_Speed   =       Speedx;
        GPIO_InitStructure.GPIO_PuPd    =       PuPdx;
        GPIO_Init(GPIOx, &GPIO_InitStructure);
}

void GPIO_AN_Config(GPIO_TypeDef* GPIOx, uint16_t Pinx) {
        GPIO_RCC(GPIOx);
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin     =       Pinx;
        GPIO_InitStructure.GPIO_Mode    =       GPIO_Mode_AN;
        GPIO_InitStructure.GPIO_PuPd    =       GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOx, &GPIO_InitStructure);
}

void GPIO_ChangeOutput(GPIO_TypeDef* GPIOx, uint16_t Pinx, bool high) {
    if(high) {
        GPIO_SetBits(GPIOx, Pinx);
    } else {
        GPIO_ResetBits(GPIOx, Pinx);
    }
}

void GPIO_SetMode(GPIO_TypeDef* GPIOx, uint16_t Pinx, GPIOMode_TypeDef mode) {
    GPIOx->MODER  &= ~(GPIO_MODER_MODER0 << (Pinx * 2));
    GPIOx->MODER |= (((uint32_t)mode) << (Pinx * 2));
}




const struct GPIO_Controllers_Type spGPIO = {
    .general_config = GPIO_Config,
    .output_config = GPIO_OUT_Config,
    .input_config = GPIO_IN_Config,
    .alternal_config = GPIO_AF_Config,
    .analog_config = GPIO_AN_Config,
    
    .set = GPIO_ChangeOutput,
    .mode = GPIO_SetMode,
    
    .toggle = GPIO_ToggleBits,
    .get_output = GPIO_ReadOutputDataBit,
    .get_input = GPIO_ReadInputDataBit
};


/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/

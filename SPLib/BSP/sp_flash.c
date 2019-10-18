/**
  ******************************************************************************
  * @file       sp_flash.c
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2019.Jul.25
  * @brief      Flash storage module.
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sp_flash.h"

#define __SECTOR_SIZE       (0x00020000)

#define __FLASH_FORMAT_KEY  (0x677CA759)

//    uint8_t     bytes[__FLASH_SIZE];
//    uint16_t    hwords[__FLASH_SIZE/2];
//    uint32_t    words[__FLASH_SIZE/4];
//    uint64_t    dwords[__FLASH_SIZE/8];
//    packed_union {
//        uint8_t     bytes[__SECTOR_SIZE];
//        uint16_t    hwords[__SECTOR_SIZE/2];
//        uint32_t    words[__SECTOR_SIZE/4];
//        uint64_t    dwords[__SECTOR_SIZE/8];
//    } sector[4];
//    // Flash status info in header.

typedef packed_struct {
    uint32_t format_key;
    uint64_t total;
    uint64_t used;
} FLASH_INFO_REG;

FLASH_INFO_REG* __flash_info = ((FLASH_INFO_REG*)(__FLASH_START_PTR));


int FLASH_Init(void) {
    if(!spFLASH.unlock()) {
        return -1;
    }
    if((*__flash_info).format_key != __FLASH_FORMAT_KEY) {
        return spFLASH.format();
    }
    if(!spFLASH.lock()) {
        return -2;
    }
    return 0;
}

bool __FLASH_Unlock(void) {
    FLASH_Unlock();
    FLASH->CR &= CR_PSIZE_MASK;
    FLASH->CR |= FLASH_PSIZE_BYTE;
    FLASH->CR |= FLASH_CR_PG;
    return (!(FLASH->CR & FLASH_CR_LOCK)) && (FLASH->CR & FLASH_CR_PG);
}

bool __FLASH_Lock(void) {
    FLASH->CR &= ~FLASH_CR_PG;
    FLASH_Lock();
    return (FLASH->CR & FLASH_CR_LOCK) && (!(FLASH->CR & FLASH_CR_PG));
}

bool __FLASH_IsLock(void) {
    return (FLASH->CR & FLASH_CR_LOCK) && (!(FLASH->CR & FLASH_CR_PG));
}

int FLASH_ClearAll(void) {
    if(!spFLASH.unlock()) {
        return -1;
    }
    FLASH_EraseSector(FLASH_Sector_20, VoltageRange_3);
    FLASH_EraseSector(FLASH_Sector_21, VoltageRange_3);
    FLASH_EraseSector(FLASH_Sector_22, VoltageRange_3);
    FLASH_EraseSector(FLASH_Sector_23, VoltageRange_3);
    (*__flash_info).format_key = __FLASH_FORMAT_KEY;
    (*__flash_info).total = __FLASH_SIZE - sizeof((*__flash_info));
    (*__flash_info).used = 0;
    FLASH_WaitForLastOperation();
    if(!spFLASH.lock()) {
        return -2;
    }
    return 0;
}


const struct FLASH_Controllers_Type spFLASH = {
    .init = FLASH_Init,
    
    .format = FLASH_ClearAll,
    
    .unlock = __FLASH_Unlock,
    .lock = __FLASH_Lock,
    .islock = __FLASH_IsLock,
};

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/

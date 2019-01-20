#ifndef SPI_H
#define SPI_H

#include "sp_conf.h"

#ifdef __cplusplus
 extern "C" {
#endif

void SPI5_Init(void);
uint8_t SPI5_ReadWriteByte(uint8_t TxData);
void SPI5_DMA(void);

#define SPI5_NSS_Select     GPIO_ResetBits(GPIOF, GPIO_Pin_6)
#define SPI5_NSS_Release    GPIO_SetBits(GPIOF, GPIO_Pin_6)

uint8_t MPU_ReadByte(uint8_t reg);
uint8_t MPU_Read(uint8_t reg,uint8_t *buf,uint8_t len);
uint8_t MPU_WriteByte(uint8_t reg,uint8_t data);
uint8_t MPU_Write(uint8_t reg,uint8_t *buf,uint8_t len);

#ifdef __cplusplus
}
#endif

#endif

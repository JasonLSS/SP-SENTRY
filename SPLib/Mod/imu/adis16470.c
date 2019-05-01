/**
  ******************************************************************************
  * @file       sp_imu.h
  * @author     YTom
  * @version    v0.1
  * @date       2019.Jan.23
  * @brief      IMU(MPU6500 & IST8310) module
  * @usage      
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */
  
#include "adis16470.h"
#include "adis16470_reg.h"


#define ADIS_SPIPins        SPI4_Pins
#define ADIS16470_ID        0x4056

packed_union __raw_busrt_data {
    uint16_t bytes[10];
    packed_struct {
        uint16_t diag_stat;
        int16_t gyro[3];
        int16_t accel[3];
        int16_t temp;
        uint16_t data_cnt;
        uint16_t checksum;
    } data;
} raw_busrt_data;

void ADIS16470_BurstRead(void) {

    /* Send out address */
    spSPI.select(&ADIS_SPIPins);
    spSPI.read_write_b(ADIS_SPIPins.this, ADIS_BURST);
    delay_us(2);
    
    /* Read returned value */
    for(uint8_t i=0; i<10; ++i) {
        spSPI.select(&ADIS_SPIPins);
        raw_busrt_data.bytes[i] = spSPI.read_write_b(ADIS_SPIPins.this, 0x00);
        delay_us(2);
    }
    
    spSPI.release(&ADIS_SPIPins);
    delay_us(6);
}

bool ADIS16470_ReadByte(uint8_t addr, uint16_t* output) {
    /* Send out address */
    spSPI.select(&ADIS_SPIPins);
    spSPI.read_write_b(ADIS_SPIPins.this, (uint16_t)(addr<<8)&0xFF00);
    spSPI.release(&ADIS_SPIPins);
    delay_us(6);
    /* Read returned value */
    spSPI.select(&ADIS_SPIPins);
    uint16_t ret = spSPI.read_write_b(ADIS_SPIPins.this, 0x00);
    if(output) output[0] = ret;
    spSPI.release(&ADIS_SPIPins);
    delay_us(6);

    return true;
}

uint16_t cmdA=0x00, cmdB=0x00;
bool ADIS16470_WriteByte(uint8_t addr, uint16_t data) {
    /* If write 0x0004 to addr:0xDC 0xDD, should send 0xDC04 then 0xDD00 */
    cmdA = (((uint16_t)(addr|0x80)<<8)&0xFF00)|(data&0x00FF);
    cmdB = ((((uint16_t)(addr|0x80)+1)<<8)&0xFF00)|((data>>8)&0x00FF);
    
    spSPI.select(&ADIS_SPIPins);
    spSPI.read_write_b(ADIS_SPIPins.this, cmdA);
    spSPI.release(&ADIS_SPIPins);
    delay_us(10);
    
    spSPI.select(&ADIS_SPIPins);
    spSPI.read_write_b(ADIS_SPIPins.this, cmdB);
    spSPI.release(&ADIS_SPIPins);
    delay_us(10);
    
    uint16_t regval;
    bool ret = ADIS16470_ReadByte(addr, &regval);
    
    return true;
}

bool ADIS16470_DeviceIDCheck(void) {
    
    uint16_t prod_id[2];
    ADIS16470_ReadByte(ADIS_PROD_ID, &prod_id[0]);
    ADIS16470_ReadByte(ADIS_PROD_ID, &prod_id[1]);
    if((prod_id[0] == ADIS16470_ID) && (prod_id[1] == ADIS16470_ID)) {
        return true;
    }
    return false;
}

void ADIS16470_Reset(bool hardware) {
    if(hardware) {
        /* Hardware reset */
        GPIOF->BSRRH |= GPIO_Pin_1;
        delay_ms(50);
        GPIOF->BSRRL |= GPIO_Pin_1;
        delay_ms(200);
    } else {
        /* Software reset */
        ADIS16470_WriteByte(ADIS_GLOB_CMD, 0b10000000);
        delay_ms(200);
    }
}

uint8_t ADIS16470_GetState(void) {
    uint16_t state = -1;
    ADIS16470_ReadByte(ADIS_DIAG_STAT, &state);
    return state;
}


//#define ADIS_DEBUG_BUFFER

#ifdef ADIS_DEBUG_BUFFER
    uint16_t buffer[6]={0x00};
#endif
    
bool ADIS16470_Init(void) {
    
    bool ret = false;
    
    /* Config SPI bus */
    extern void SPI4_Init(void);
    SPI4_Init();
    delay_ms(1);
    
    /* Hardware reset IMU */
    spGPIO.output_config(GPIOF, GPIO_Pin_1, GPIO_OType_PP, GPIO_PuPd_UP, GPIO_Speed_100MHz);
    ADIS16470_Reset(true);
    
    /* Check communication */
    if(!ADIS_Driver.user.check_device())
        return false;
    
#ifdef ADIS_DEBUG_BUFFER
    ADIS16470_ReadByte(ADIS_MSC_CTRL, &buffer[0]);
    ADIS16470_ReadByte(ADIS_FILT_CTRL, &buffer[1]);
    ADIS16470_ReadByte(ADIS_DEC_RATE, &buffer[2]);
#endif
    
    /* Config IMU */
    /* [DR=high when data ready] */
//    // [internal clock mode: Fg = 2000Hz, Fa = 4000Hz][MSC_CTRL def=0x00C1]
//    ret = ADIS16470_WriteByte(ADIS_MSC_CTRL, 0x0001);   delay_us(10);
    /* Set internal FIR filter size variable 3 (means number of taps in each stage) */
    ret = ADIS16470_WriteByte(ADIS_FILT_CTRL, 0x0003);      delay_us(10);
    if(!ret) return false;
    /* When working in internal clock mode, nominal output data rate = 2000/(DEC_RATE+1) */
    ret = ADIS16470_WriteByte(ADIS_DEC_RATE, 0x0003);       delay_us(10);       // [500Hz]
    if(!ret) return false;
    
#ifdef ADIS_DEBUG_BUFFER
    ADIS16470_ReadByte(ADIS_MSC_CTRL, &buffer[3]);
    ADIS16470_ReadByte(ADIS_FILT_CTRL, &buffer[4]);
    ADIS16470_ReadByte(ADIS_DEC_RATE, &buffer[5]);
#endif
    
    /* Update IMU parameters */
    
    

    /* Condif EXIT for MPU interrupt */
    spRCC_Set_SYSCFG();
    spGPIO.input_config(GPIOF, GPIO_Pin_0, GPIO_PuPd_UP, GPIO_Speed_100MHz);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, GPIO_PinSource0); 
    EXTI_InitTypeDef            exit_initer;
    exit_initer.EXTI_Line       = EXTI_Line0;
    exit_initer.EXTI_LineCmd    = ENABLE;
    exit_initer.EXTI_Mode       = EXTI_Mode_Interrupt;
    exit_initer.EXTI_Trigger    = EXTI_Trigger_Rising;
    EXTI_Init(&exit_initer);
    /* Registe IMU update callback */
    spIRQ.registe(EXTI0_IRQn, NULL, ADIS_Driver.user.update);
    
    return true;
}

void ADIS16470_Update(void) {
    ADIS16470_BurstRead();
}


struct __ADIS16470_Driver ADIS_Driver = {
    .__param_const = {
        
    }, 
    .register_ope = {
        .read_byte = ADIS16470_ReadByte,
        .burst_read = ADIS16470_BurstRead,
        .write_byte = ADIS16470_WriteByte,
    },
    .user = {
        .init = ADIS16470_Init,
        .check_device = ADIS16470_DeviceIDCheck,
        .update = ADIS16470_Update,
        .reset = ADIS16470_Reset,
        .get_state = ADIS16470_GetState,
    }
};

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/

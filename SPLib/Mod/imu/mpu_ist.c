/**
  ******************************************************************************
  * @file       mpu_ist.h
  * @author     YTom
  * @version    v0.1
  * @date       2019.Jan.23
  * @brief      IMU(MPU6500 & IST8310) module
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

#include <math.h>
#include "mpu_reg.h"
#include "sp_imu.h"
#include "sp_conf.h"


/*!< MPU data reading buffer */
uint8_t mpu_data_buffer[64] = {0x00};

/* Internal Parameters */
#define INTERNAL_SAMPRATE           1000

const float ACCEL_SEN = 16384.0f;   //
const float GYRO_SEN = spMATH_RAD2DEG(65.0f);     //
const float MAG_SEN = 0.3f;         //


/* Internal Declarations */
uint8_t MPU_ReadByte(uint8_t reg);
void    MPU_Read(uint8_t reg,uint8_t *buf,uint8_t len);
void    MPU_WriteByte(uint8_t reg,uint8_t data);
void    MPU_Write(uint8_t reg,uint8_t *buf,uint8_t len);


/*
DECLARATIONS:
    Following IST driver code are copied from DJI's project
    <RM Standard Robot Open Source Code> with some of my modification
    and comments. (by YTom)
    DJI All Right Reserved.
*/
/* IST Register map */
typedef enum {
    IST8310_WHO_AM_I           = 0x00,
    IST8310_ODR_MODE           = 0x01,
    IST8310_R_MODE             = 0x02,
    IST8310_R_XL               = 0x03,
    IST8310_R_XM               = 0x04,
    IST8310_R_YL               = 0x05,
    IST8310_R_YM               = 0x06,
    IST8310_R_ZL               = 0x07,
    IST8310_R_ZM               = 0x08,
    IST8310_R_CONFA            = 0x0A,
    IST8310_R_CONFB            = 0x0B,
    IST8310_ADDRESS            = 0x0E,
    IST8310_DEVICE_ID_A        = 0x10,  // Previously 0x10
    IST8310_AVGCNTL            = 0x41,
    IST8310_PDCNTL             = 0x42,
} IST8310_REG;
/* Read register data from IST83110 */
static uint8_t ist8310_read_reg(uint8_t addr) {
    MPU_WriteByte(MPU_I2C_SLV4_REG, addr);
    delay_ms(10);
    MPU_WriteByte(MPU_I2C_SLV4_CTRL, 0x80);
    delay_ms(10);
    uint8_t data = MPU_ReadByte(MPU_I2C_SLV4_DI);
    MPU_WriteByte(MPU_I2C_SLV4_CTRL, 0x00);     // Turn off slave4 after read
    delay_ms(10);
    return data;
}
/* Write data to IST83110 register*/
static void ist8310_write_reg(uint8_t addr, uint8_t data) {
    MPU_WriteByte(MPU_I2C_SLV1_CTRL, 0x00);     // Turn off slave 1 at first
    delay_ms(2);
    MPU_WriteByte(MPU_I2C_SLV1_REG, addr);
    delay_ms(2);
    MPU_WriteByte(MPU_I2C_SLV1_DO, data);
    delay_ms(2);
    // Turn on slave 1 with one byte transmitting
    MPU_WriteByte(MPU_I2C_SLV1_CTRL, 0x80 | 0x01);
    delay_ms(10);
}
/* Config IST83110 via MPU */
static void mpu_read_ist_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num) {
    // Use slave 1 to automatically transmit single measure mode
    MPU_WriteByte(MPU_I2C_SLV1_ADDR, device_address);
    delay_ms(2);
    MPU_WriteByte(MPU_I2C_SLV1_REG, IST8310_R_CONFA);
    delay_ms(2);
    MPU_WriteByte(MPU_I2C_SLV1_DO, IST8310_ODR_MODE);
    delay_ms(2);
    // Use slave 0 to read data automatically
    MPU_WriteByte(MPU_I2C_SLV0_ADDR, 0x80 | device_address);
    delay_ms(2);
    MPU_WriteByte(MPU_I2C_SLV0_REG, reg_base_addr);
    delay_ms(2);
    // Every 8 mpu6500 internal samples, one i2c master read
    MPU_WriteByte(MPU_I2C_SLV4_CTRL, 0x03);
    delay_ms(2);
    // Enable slave 0 and 1 access delay
    MPU_WriteByte(MPU_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
    delay_ms(2);
    // Enable slave 1 auto transmit
    MPU_WriteByte(MPU_I2C_SLV1_CTRL, 0x80 | 0x01);
    delay_ms(6); // Wait 6ms (minimum waiting time for 16 times internal average setup)
    // Enable slave 0 with data_num bytes reading
    MPU_WriteByte(MPU_I2C_SLV0_CTRL, 0x80 | data_num);
    delay_ms(2);
}
/* Initialize IST83110 */
bool ist8310_INIT(void) {
    // Enable the I2C Master I/F module, Reset I2C Slave module
    MPU_WriteByte(MPU_USER_CTRL, 0x30);
    delay_ms(10);
    // I2C master clock 400kHz
    MPU_WriteByte(MPU_I2C_MST_CTRL, 0x0D);
    delay_ms(10);
    // Turn on slave 1 for ist write and slave 4 for ist read
    MPU_WriteByte(MPU_I2C_SLV1_ADDR, IST8310_ADDRESS); // Write from slave 1
    delay_ms(10);
    MPU_WriteByte(MPU_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS); // Read from slave 4
    delay_ms(10);
    // reset ist8310
    ist8310_write_reg(IST8310_R_CONFB, 0x01);
    delay_ms(10);
    // Check IST id
    if(IST8310_DEVICE_ID_A != ist8310_read_reg(IST8310_WHO_AM_I)) {
        // bsp_error_handler(__FUNCTION__, __LINE__, "IST8310 ID does not match.");
        return false;
    }
    // Reset ist8310
    ist8310_write_reg(IST8310_R_CONFB, 0x01);
    delay_ms(10);
    // Config as ready mode to access reg
    ist8310_write_reg(IST8310_R_CONFA, 0x00);
    if(ist8310_read_reg(IST8310_R_CONFA) != 0x00) {
        // bsp_error_handler(__FUNCTION__, __LINE__, "IST ready mode failed.");
        return false;
    }
    delay_ms(10);
    // Normal state, no int
    ist8310_write_reg(IST8310_R_CONFB, 0x00);
    if(ist8310_read_reg(IST8310_R_CONFB) != 0x00) {
        // bsp_error_handler(__FUNCTION__, __LINE__, "IST normal state init failed.");
        return false;
    }
    delay_ms(10);
    // Config low noise mode, x,y,z axis 16 time 1 avg,
    ist8310_write_reg(IST8310_AVGCNTL, 0x24); // 100100
    if(ist8310_read_reg(IST8310_AVGCNTL) != 0x24) {
        // bsp_error_handler(__FUNCTION__, __LINE__, "IST low noise mode failed.");
        return false;
    }
    delay_ms(10);
    // Set/Reset pulse duration setup, normal mode
    ist8310_write_reg(IST8310_PDCNTL, 0xC0);
    if(ist8310_read_reg(IST8310_PDCNTL) != 0xC0) {
        // bsp_error_handler(__FUNCTION__, __LINE__, "IST pulse duration set failed.");
        return false;
    }
    delay_ms(10);
    // Turn off slave1 & slave 4
    MPU_WriteByte(MPU_I2C_SLV1_CTRL, 0x00);
    delay_ms(10);
    MPU_WriteByte(MPU_I2C_SLV4_CTRL, 0x00);
    delay_ms(10);
    // Configure and turn on slave 0
    mpu_read_ist_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
    delay_ms(100);
    return true;
}
/* Get data from IST8310 */
void ist8310_get_data(float* mag ) {
    if(mag) {
        uint8_t ist_buff[6];
        MPU_Read(MPU_EXT_SENS_DATA_00, ist_buff, 6);
        mag[0] = (int16_t)(ist_buff[1]<<8 | ist_buff[0]) * MAG_SEN;
        mag[1] = (int16_t)(ist_buff[3]<<8 | ist_buff[2]) * MAG_SEN;
        mag[2] = (int16_t)(ist_buff[5]<<8 | ist_buff[4]) * MAG_SEN;
    }
}



/* MPU device operations vis SPI5 */
uint8_t MPU_ReadByte(uint8_t reg) {
    uint8_t res;
    spSPI.select(&SPI5_Pins);
    spSPI.read_write_b(SPI5_Pins.this, reg|0x80);
    res = spSPI.read_write_b(SPI5_Pins.this, 0xff);
    spSPI.release(&SPI5_Pins);
    return res;
}
void MPU_Read(uint8_t reg,uint8_t *buf,uint8_t len) {
    uint8_t i;
    spSPI.select(&SPI5_Pins);
    spSPI.read_write_b(SPI5_Pins.this, reg|0x80);
    for(i=0; i<len; i++) {
        *buf = spSPI.read_write_b(SPI5_Pins.this, 0xff);
        buf++;
    }
    spSPI.release(&SPI5_Pins);
}
void MPU_WriteByte(uint8_t reg,uint8_t data) {
    spSPI.select(&SPI5_Pins);
    spSPI.read_write_b(SPI5_Pins.this, reg);
    spSPI.read_write_b(SPI5_Pins.this, data);
    spSPI.release(&SPI5_Pins);
}
void MPU_Write(uint8_t reg,uint8_t *buf,uint8_t len) {
    spSPI.select(&SPI5_Pins);
    spSPI.read_write_b(SPI5_Pins.this, reg);
    while(len--) {
        spSPI.read_write_b(SPI5_Pins.this, *buf++);
    }
    spSPI.release(&SPI5_Pins);
    delay_ms(1);
}




/* Check MPU device ID */
bool mpuCheckDeviceID(void) {
    return MPU_ReadByte(MPU_WHO_AM_I) == MPU_ID;
}
/* Set MPU sampling rate */
void mpuSetSampleRate(uint32_t rate) {
    uint8_t samp_div = INTERNAL_SAMPRATE/rate-1;
    MPU_WriteByte(MPU_SMPLRT_DIV, MPU_SMPLRT_DIV_(samp_div));
}




void MPU6500_Stream_All(float* gyro, float* accel, float* temp, float* mag) {
    if(gyro || accel || temp) {
        MPU_Read(MPU_ACCEL_XOUT_H, mpu_data_buffer, 14);
        if(accel) {
            accel[0] = (short)(((mpu_data_buffer[0])<<8)|mpu_data_buffer[1]) /ACCEL_SEN;
            accel[1] = (short)(((mpu_data_buffer[2])<<8)|mpu_data_buffer[3]) /ACCEL_SEN;
            accel[2] = (short)(((mpu_data_buffer[4])<<8)|mpu_data_buffer[5]) /ACCEL_SEN;
        }
        if(temp) *temp= (short)(((mpu_data_buffer[6])<<8)|mpu_data_buffer[7])/340.f+36.53f;
        if(gyro) {
            gyro[0] = (short)(((mpu_data_buffer[8])<<8)|mpu_data_buffer[9]) /GYRO_SEN;
            gyro[1] = (short)(((mpu_data_buffer[10])<<8)|mpu_data_buffer[11]) /GYRO_SEN;
            gyro[2] = (short)(((mpu_data_buffer[12])<<8)|mpu_data_buffer[13]) /GYRO_SEN;
        }
    }
    if(mag) ist8310_get_data(mag);
}

void MPU6500_Stream_GyroAccel(float* gyro, float* accel) {
    if(gyro || accel) {
        MPU_Read(MPU_ACCEL_XOUT_H, mpu_data_buffer, 14);
        if(accel) {
            accel[0] = (short)(((mpu_data_buffer[0])<<8)|mpu_data_buffer[1]) /ACCEL_SEN;
            accel[1] = (short)(((mpu_data_buffer[2])<<8)|mpu_data_buffer[3]) /ACCEL_SEN;
            accel[2] = (short)(((mpu_data_buffer[4])<<8)|mpu_data_buffer[5]) /ACCEL_SEN;
        }
        if(gyro) {
            gyro[0] = (short)(((mpu_data_buffer[8])<<8)|mpu_data_buffer[9]) /GYRO_SEN;
            gyro[1] = (short)(((mpu_data_buffer[10])<<8)|mpu_data_buffer[11]) /GYRO_SEN;
            gyro[2] = (short)(((mpu_data_buffer[12])<<8)|mpu_data_buffer[13]) /GYRO_SEN;
        }
    }
}

void MPU6500_Read_FIFO_Stream(float* gyro, float* accel) {
    if(gyro || accel) {
        MPU_Read(MPU_FIFO_R_W, mpu_data_buffer, 12);
        if(accel) {
            accel[0] = (short)(((mpu_data_buffer[0])<<8)|mpu_data_buffer[1]) /ACCEL_SEN;
            accel[1] = (short)(((mpu_data_buffer[2])<<8)|mpu_data_buffer[3]) /ACCEL_SEN;
            accel[2] = (short)(((mpu_data_buffer[4])<<8)|mpu_data_buffer[5]) /ACCEL_SEN;
        }
        if(gyro) {
            gyro[0] = (short)(((mpu_data_buffer[8])<<8)|mpu_data_buffer[9]) /GYRO_SEN;
            gyro[1] = (short)(((mpu_data_buffer[10])<<8)|mpu_data_buffer[11]) /GYRO_SEN;
            gyro[2] = (short)(((mpu_data_buffer[12])<<8)|mpu_data_buffer[13]) /GYRO_SEN;
        }
    }
}

void MPU6500_Reset_FIFO(void) {
    uint8_t data = MPU_ReadByte(MPU_USER_CTRL);
    delay_us(20);
    data |= MPU_USER_CTRL_FIFO_RST;
    MPU_WriteByte(MPU_USER_CTRL, data);
    delay_us(100);
}

/* Condif EXIT for MPU interrupt */
#ifdef SP_USING_BOARD_TYPEA
IRQn_Type MPU6500_IRQ_Config(void) {
    
    spRCC_Set_SYSCFG();
    spGPIO.input_config(GPIOB, GPIO_Pin_8, GPIO_PuPd_UP, GPIO_Speed_100MHz);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, GPIO_PinSource8); 

    EXTI_InitTypeDef            exit_initer;
    exit_initer.EXTI_Line       = EXTI_Line8;
    exit_initer.EXTI_LineCmd    = ENABLE;
    exit_initer.EXTI_Mode       = EXTI_Mode_Interrupt;
    exit_initer.EXTI_Trigger    = EXTI_Trigger_Rising;
    EXTI_Init(&exit_initer);
    
    return EXTI9_5_IRQn;
}
#else
IRQn_Type MPU6500_IRQ_Config(void) {
    
    spRCC_Set_SYSCFG();
    spGPIO.input_config(GPIOE, GPIO_Pin_1);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, GPIO_PinSource1);
    
    EXTI_InitTypeDef              exit_initer;
    exit_initer.EXTI_Line       = EXTI_Line1;
    exit_initer.EXTI_LineCmd    = ENABLE;
    exit_initer.EXTI_Mode       = EXTI_Mode_Interrupt;
    exit_initer.EXTI_Trigger    = EXTI_Trigger_Rising;
    EXTI_Init(&exit_initer);
    
    return EXTI1_IRQn;
}
#endif

int MPU6500_Init(void) {

    int result = 0;

    extern void SPI5_Init(void);
    SPI5_Init();
//    SPI5_DMA();

//    uint8_t i = 16;
//    mpu_data_buffer[d++] = MPU_ReadByte(MPU_PWR_MGMT_1);delay_ms(10);
//    mpu_data_buffer[d++] = MPU_ReadByte(MPU_PWR_MGMT_2);delay_ms(10);
//    mpu_data_buffer[d++] = MPU_ReadByte(MPU_CONFIG);delay_ms(10);
//    mpu_data_buffer[d++] = MPU_ReadByte(MPU_GYRO_CONFIG);delay_ms(10);
//    mpu_data_buffer[d++] = MPU_ReadByte(MPU_ACCEL_CONFIG_2);delay_ms(10);
//    mpu_data_buffer[d++] = MPU_ReadByte(MPU_ACCEL_CONFIG);delay_ms(10);
//    mpu_data_buffer[d++] = MPU_ReadByte(MPU_INT_PIN_CFG);delay_ms(10);
//    mpu_data_buffer[d++] = MPU_ReadByte(MPU_I2C_MST_CTRL);delay_ms(10);
//    mpu_data_buffer[d++] = MPU_ReadByte(MPU_I2C_MST_DELAY_CTRL);delay_ms(10);
//    mpu_data_buffer[d++] = MPU_ReadByte(MPU_INT_ENABLE);delay_ms(10);
//    mpu_data_buffer[d++] = MPU_ReadByte(MPU_FIFO_EN);delay_ms(10);
//    mpu_data_buffer[d++] = MPU_ReadByte(MPU_USER_CTRL);delay_ms(10);

    {
        uint8_t data;
        /* MPU working mode configurations */
        if(mpuCheckDeviceID()) {
            delay_ms(10);
            /*----------------------------------------*/
            /* BASIS CONFIGURATION */
            /*----------------------------------------*/
            /* Reset MPU6500 */
            data = MPU_PWR_MGMT_1_DEVICE_RESET;
            MPU_WriteByte(MPU_PWR_MGMT_1, data);
            delay_ms(100);

            /* Select MPU6500 clock source */
            data = MPU_PWR_MGMT_1_CLKSEL(1);
            MPU_WriteByte(MPU_PWR_MGMT_1, data);
            delay_ms(10);

            /*----------------------------------------*/
            /* SENSOR CONFIGURATION */
            /*----------------------------------------*/
            /* Enable all accelemters and gyroscopes */
            data = 0x00;
            MPU_WriteByte(MPU_PWR_MGMT_2, data);
            delay_ms(10);

            /* Config sample rate */
            mpuSetSampleRate(DEFAULT_MPU_HZ);
            /* Config gyrpscope and temperature snesor */
            /* GYRO DLPF:none[bandwidth=8800Hz][delay=0.064ms][Fs=32kHz] */
            /* TEMP DLPF:none[bandwidth=4000Hz][delay=0.04ms] */
            data = MPU_CONFIG_DLPG_CFG(2) | MPU_GYRO_CONFIG_FCHOICE_B(0);
            MPU_WriteByte(MPU_CONFIG, data);
            delay_ms(10);

            /* GYRO [sensi=500deg/s] */
            data = MPU_GYRO_CONFIG_GYRO_FS_SEL(1);
            MPU_WriteByte(MPU_GYRO_CONFIG, data);
            delay_ms(10);

            /* Config accelemeter snesor */
            /* ACCEL DLPF:none[bandwidth=1130Hz][delay=0.75ms][noise=220ug/rtHz][Fs=4kHz] */
            data = MPU_ACCEL_CONFIG_2_A_DLPF_CFG(2);
            MPU_WriteByte(MPU_ACCEL_CONFIG_2, data);
            delay_ms(10);

            /* ACCEL [sensi=+-2g] */
            data = MPU_ACCEL_CONFIG_ACCEL_FS_SEL(0);
            MPU_WriteByte(MPU_ACCEL_CONFIG, data);
            delay_ms(10);


            /*----------------------------------------*/
            /* I2C CONFIGURATION FOR IST8310 */
            /*----------------------------------------*/
            data = MPU_INT_PIN_CFG_BYPASS_EN;
            MPU_WriteByte(MPU_INT_PIN_CFG, data);
            delay_ms(10);

            data = (~MPU_I2C_MST_CTRL_MULI_MST_EN) |
                   (MPU_I2C_MST_CTRL_WAIT_FOR_ES) |
                   (~MPU_I2C_MST_CTRL_SLV_3_FIFO_EN) |
                   (~MPU_I2C_MST_CTRL_I2C_MST_P_NSR) |
                   MPU_I2C_MST_CLK_400_KHZ;    // 400kHz, 20MHzdelay_ms(10);
            MPU_WriteByte(MPU_I2C_MST_CTRL, data);
            delay_ms(10);

            data = MPU_I2C_MST_DELAY_CTRL_I2C_SLV0_DLY_EN;
            MPU_WriteByte(MPU_I2C_MST_DELAY_CTRL, data);
            delay_ms(10);


            /*----------------------------------------*/
            /* START CONFIGURATION */
            /*----------------------------------------*/
//        /* Enable FIFO for accel and gyro. */
//        data = MPU_FIFO_EN_GYRO_XOUT |
//           MPU_FIFO_EN_GYRO_YOUT |
//           MPU_FIFO_EN_GYRO_ZOUT |
//           MPU_FIFO_EN_ACCEL_OUT;
//        MPU_WriteByte(MPU_FIFO_EN, data);delay_ms(10);

            /* Disable DMP, reset all sensors and clear all registers, enable I2C interface */
            data = MPU_USER_CTRL_I2C_MST_EN | \
                   MPU_USER_CTRL_I2C_IF_DIS| \
                   MPU_USER_CTRL_SIG_COND_RST;
            MPU_WriteByte(MPU_USER_CTRL, data);
            delay_ms(10);


            /*----------------------------------------*/
            /* IST8310 CONFIGURATION */
            /*----------------------------------------*/
            /* Init IST8310 */
            if(ist8310_INIT()) {
                result += 2;
            }
            delay_ms(50);

            /*----------------------------------------*/
            /* MPU INTERRUPT CONFIGURATION */
            /*----------------------------------------*/
            /* Enable interrupts and auto-clear flags */
            data = MPU_INT_ENABLE_RAW_RDY_EN;
            MPU_WriteByte(MPU_INT_ENABLE, data);
            delay_ms(10);
            
            result += 1;
        }
    }
    
//    uint8_t i = 0;
//    mpu_data_buffer[d++] = MPU_ReadByte(MPU_PWR_MGMT_1);delay_ms(10);
//    mpu_data_buffer[d++] = MPU_ReadByte(MPU_PWR_MGMT_2);delay_ms(10);
//    mpu_data_buffer[d++] = MPU_ReadByte(MPU_CONFIG);delay_ms(10);
//    mpu_data_buffer[d++] = MPU_ReadByte(MPU_GYRO_CONFIG);delay_ms(10);
//    mpu_data_buffer[d++] = MPU_ReadByte(MPU_ACCEL_CONFIG_2);delay_ms(10);
//    mpu_data_buffer[d++] = MPU_ReadByte(MPU_ACCEL_CONFIG);delay_ms(10);
//    mpu_data_buffer[d++] = MPU_ReadByte(MPU_INT_PIN_CFG);delay_ms(10);
//    mpu_data_buffer[d++] = MPU_ReadByte(MPU_I2C_MST_CTRL);delay_ms(10);
//    mpu_data_buffer[d++] = MPU_ReadByte(MPU_I2C_MST_DELAY_CTRL);delay_ms(10);
//    mpu_data_buffer[d++] = MPU_ReadByte(MPU_INT_ENABLE);delay_ms(10);
//    mpu_data_buffer[d++] = MPU_ReadByte(MPU_FIFO_EN);delay_ms(10);
//    mpu_data_buffer[d++] = MPU_ReadByte(MPU_USER_CTRL);delay_ms(10);
    
    return result;
}

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/


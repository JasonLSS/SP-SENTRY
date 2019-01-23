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

#ifndef __SP_IMU_H
#define __SP_IMU_H

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup SP
  * @brief      SuperPower
  * @{
  */

/** @defgroup IMU
  * @brief    IMU Module
  * @{
  */

#include "sp_spi.h"
#include "sp_conf.h"
#include "AHRS.h"
#include "kalman.h"

typedef struct {
    float q[4];
    float gyro[3];
    float accel[3];
    float mag[3];
    float temp;
    float r,p,y;
} AHRS_t;



/** @defgroup Declarations
  * @brief    Exported Function Declaration
  * @ingroup  IMU
  * @{
  */
/** 
  * @brief  IMU Controller
  */
extern struct IMU_Controllers_Type {
    /** 
      * @brief  IMU control operations
      */
    struct {
        /** 
          * @brief  Initialize MPU6500
          * @retval NULL
          */
        int (*init)(void);
        /** 
          * @brief  Reset MPU6500 data FIFOs
          * @retval NULL
          */
        uint16_t (*fifo_reset)(void);
        /** 
          * @brief  Read data from MPU6500 FIFOs
          * @retval NULL
          */
        uint16_t (*fifo_read)(void);
        /** 
          * @brief  Read data stream from MPU6500 from FIFO
          * @param  gyro: 3*1 gyroscope output array
          * @param  accel: 3*1 accelerometer output array
          * @retval NULL
          */
        uint16_t (*fifo_read_stream)(short* gyro, short* accel);
        /** 
          * @brief  Read data stream from MPU6500
          * @param  gyro: 3*1 gyroscope output array
          * @param  accel: 3*1 accelerometer output array
          * @param  temp: temprature output pointer
          * @param  mag: 3*1 magnetnometer output array
          */
        void (*read_stream)(float* gyro, float* accel, float* temp, float* mag);
    } operations;

    /** 
      * @brief  IMU informations
      */
    struct {
        AHRS_t          ahrs;                       /*!< AHRS converted data */
        Kalman_t        kalman;
        bool            inited;                     /*!< If AHRS data initialized */
        
        float           temp;                       /*!< Temprature value from MPU6500 */
        float           timestamp;                  /*!< IMU data timestamp */
        float           freq;                       /*!< IMU realtime reading frequency */
        uint32_t        count;                      /*!< IMU data package counters */
    } imu_state;
    
} IMU_Controllers;

/**
  * @brief  MPU sampling frequency
  */ 
#define DEFAULT_MPU_HZ              (200)

/**
  * @brief  Gyroscope, accelerometer and magnetometer's precision.s
  */ 
extern const float ACCEL_SEN;
extern const float GYRO_SEN;
extern const float MAG_SEN;
/** @} */



/** @defgroup RegisterMap
  * @brief    Register map of MPU6500
  * @ingroup  IMU
  * @{
  */
#define MPU_SELF_TEST_XG                            (0x00)
#define MPU_SELF_TEST_YG                            (0x01)
#define MPU_SELF_TEST_ZG                            (0x02)
#define MPU_SELF_TEST_XA                            (0x0D)
#define MPU_SELF_TEST_YA                            (0x0E)
#define MPU_SELF_TEST_ZA                            (0x0F)
#define MPU_XG_OFFSET_H                             (0x13)
#define MPU_XG_OFFSET_L                             (0x14)
#define MPU_YG_OFFSET_H                             (0x15)
#define MPU_YG_OFFSET_L                             (0x16)
#define MPU_ZG_OFFSET_H                             (0x17)
#define MPU_ZG_OFFSET_L                             (0x18)
#define MPU_SMPLRT_DIV                              (0x19)
    /* on FCHOICE = 0b11 (FCHOICE_B register bits are 0b00), and (0 < DLPF_CFG < 7)  */
    #define MPU_SMPLRT_DIV_REG                      (0xFF)
    #define MPU_SMPLRT_DIV_(x)                      (0xFF&(x))
#define MPU_CONFIG                                  (0x1A)
    #define MPU_CONFIG_FIFO_MODE                    (0x40)
    #define MPU_CONFIG_EXT_SYNC_SET_REG             (0x38)
    #define MPU_CONFIG_EXT_SYNC_SET(x)              (0x07&(x)<<3)
    #define MPU_CONFIG_DLPG_CFG_REG                 (0x07)
    #define MPU_CONFIG_DLPG_CFG(x)                  (0x07&(x))    
#define MPU_GYRO_CONFIG                             (0x1B)
    #define MPU_GYRO_CONFIG_XG_ST                   (0x80)
    #define MPU_GYRO_CONFIG_YG_ST                   (0x40)
    #define MPU_GYRO_CONFIG_ZG_ST                   (0x20)
    #define MPU_GYRO_CONFIG_GYRO_FS_SEL_REG         (0x18)
    #define MPU_GYRO_CONFIG_GYRO_FS_SEL(x)          (0x03&(x)<<3)
    #define MPU_GYRO_CONFIG_FCHOICE_B_REG           (0x03)
    #define MPU_GYRO_CONFIG_FCHOICE_B(x)            (0x03&(x))
#define MPU_ACCEL_CONFIG                            (0x1C)
    #define MPU_ACCEL_CONFIG_XA_ST                  (0x80)
    #define MPU_ACCEL_CONFIG_YA_ST                  (0x40)
    #define MPU_ACCEL_CONFIG_ZA_ST                  (0x20)
    #define MPU_ACCEL_CONFIG_ACCEL_FS_SEL_REG       (0x18)
    #define MPU_ACCEL_CONFIG_ACCEL_FS_SEL(x)        (0x03&(x)<<3)
#define MPU_ACCEL_CONFIG_2                          (0x1D)
    #define MPU_ACCEL_CONFIG_2_ACCEL_FCHOICE_B      (0x04)
    #define MPU_ACCEL_CONFIG_2_A_DLPF_CFG_REG       (0x03)
    #define MPU_ACCEL_CONFIG_2_A_DLPF_CFG(x)        (0x03&(x))
#define MPU_LP_ACCEL_ODR                            (0x1E)
    #define MPU_LP_ACCEL_ODR_LPOSC_CLKSEL_REG       (0x0F)
    #define MPU_LP_ACCEL_ODR_LPOSC_CLKSEL(x)        (0x0F&(x))
#define MPU_MOT_THR                                 (0x1F)
    #define MPU_MOT_THR_WOM_THRESHOLD_REG           (0xFF)
    #define MPU_MOT_THR_WOM_THRESHOLD(x)            (0xFF&(x))
#define MPU_FIFO_EN                                 (0x23)
    #define MPU_FIFO_EN_TEMP_OUT                    (0x80)
    #define MPU_FIFO_EN_GYRO_XOUT                   (0x40)
    #define MPU_FIFO_EN_GYRO_YOUT                   (0x20)
    #define MPU_FIFO_EN_GYRO_ZOUT                   (0x10)
    #define MPU_FIFO_EN_ACCEL_OUT                   (0x08)
    #define MPU_FIFO_EN_SLV_2                       (0x04)
    #define MPU_FIFO_EN_SLV_1                       (0x02)
    #define MPU_FIFO_EN_SLV_0                       (0x01)
#define MPU_I2C_MST_CTRL                            (0x24)
    #define MPU_I2C_MST_CTRL_MULI_MST_EN            (0x80)
    #define MPU_I2C_MST_CTRL_WAIT_FOR_ES            (0x40)
    #define MPU_I2C_MST_CTRL_SLV_3_FIFO_EN          (0x20)
    #define MPU_I2C_MST_CTRL_I2C_MST_P_NSR          (0x10)
    #define MPU_I2C_MST_CTRL_I2C_MST_CLK_REG        (0x0F)
    #define MPU_I2C_MST_CTRL_I2C_MST_CLK(x)         (0x0F&(x))
        # define MPU_I2C_MST_CLK_348_KHZ            (MPU_I2C_MST_CTRL_I2C_MST_CLK(0x0))
        # define MPU_I2C_MST_CLK_333_KHZ            (MPU_I2C_MST_CTRL_I2C_MST_CLK(0x1))
        # define MPU_I2C_MST_CLK_320_KHZ            (MPU_I2C_MST_CTRL_I2C_MST_CLK(0x2))
        # define MPU_I2C_MST_CLK_308_KHZ            (MPU_I2C_MST_CTRL_I2C_MST_CLK(0x3))
        # define MPU_I2C_MST_CLK_296_KHZ            (MPU_I2C_MST_CTRL_I2C_MST_CLK(0x4))
        # define MPU_I2C_MST_CLK_286_KHZ            (MPU_I2C_MST_CTRL_I2C_MST_CLK(0x5))
        # define MPU_I2C_MST_CLK_276_KHZ            (MPU_I2C_MST_CTRL_I2C_MST_CLK(0x6))
        # define MPU_I2C_MST_CLK_267_KHZ            (MPU_I2C_MST_CTRL_I2C_MST_CLK(0x7))
        # define MPU_I2C_MST_CLK_258_KHZ            (MPU_I2C_MST_CTRL_I2C_MST_CLK(0x8))
        # define MPU_I2C_MST_CLK_500_KHZ            (MPU_I2C_MST_CTRL_I2C_MST_CLK(0x9))
        # define MPU_I2C_MST_CLK_471_KHZ            (MPU_I2C_MST_CTRL_I2C_MST_CLK(0xA))
        # define MPU_I2C_MST_CLK_444_KHZ            (MPU_I2C_MST_CTRL_I2C_MST_CLK(0xB))
        # define MPU_I2C_MST_CLK_421_KHZ            (MPU_I2C_MST_CTRL_I2C_MST_CLK(0xC))
        # define MPU_I2C_MST_CLK_400_KHZ            (MPU_I2C_MST_CTRL_I2C_MST_CLK(0xD))
        # define MPU_I2C_MST_CLK_381_KHZ            (MPU_I2C_MST_CTRL_I2C_MST_CLK(0xE))
        # define MPU_I2C_MST_CLK_364_KHZ            (MPU_I2C_MST_CTRL_I2C_MST_CLK(0xF))
#define MPU_I2C_SLV0_ADDR                           (0x25)
#define MPU_I2C_SLV0_REG                            (0x26)
#define MPU_I2C_SLV0_CTRL                           (0x27)
#define MPU_I2C_SLV1_ADDR                           (0x28)
#define MPU_I2C_SLV1_REG                            (0x29)
#define MPU_I2C_SLV1_CTRL                           (0x2A)
#define MPU_I2C_SLV2_ADDR                           (0x2B)
#define MPU_I2C_SLV2_REG                            (0x2C)
#define MPU_I2C_SLV2_CTRL                           (0x2D)
#define MPU_I2C_SLV3_ADDR                           (0x2E)
#define MPU_I2C_SLV3_REG                            (0x2F)
#define MPU_I2C_SLV3_CTRL                           (0x30)
#define MPU_I2C_SLV4_ADDR                           (0x31)
#define MPU_I2C_SLV4_REG                            (0x32)
#define MPU_I2C_SLV4_DO                             (0x33)
#define MPU_I2C_SLV4_CTRL                           (0x34)
#define MPU_I2C_SLV4_DI                             (0x35)
#define MPU_I2C_MST_STATUS                          (0x36)
#define MPU_INT_PIN_CFG                             (0x37)
    #define MPU_INT_PIN_CFG_ACTL                    (0x80)
    #define MPU_INT_PIN_CFG_OPEN                    (0x40)
    #define MPU_INT_PIN_CFG_LATCH_INT_EN            (0x20)
    #define MPU_INT_PIN_CFG_INT_ANYRD_2CLEAR        (0x10)
    #define MPU_INT_PIN_CFG_ACTL_FSYNC              (0x08)
    #define MPU_INT_PIN_CFG_FSYNC_INT_MODE_EN       (0x04)
    #define MPU_INT_PIN_CFG_BYPASS_EN               (0x02)
#define MPU_INT_ENABLE                              (0x38)
    #define MPU_INT_ENABLE_WOM_EN                   (0x40)
    #define MPU_INT_ENABLE_FIFO_OVERFLOW_EN         (0x10)
    #define MPU_INT_ENABLE_FSYNC_INT_EN             (0x08)
    #define MPU_INT_ENABLE_RAW_RDY_EN               (0x01)
#define MPU_INT_STATUS                              (0x3A)
    #define MPU_INT_STATUS_WOM                      (0x40)
    #define MPU_INT_STATUS_FIFO_OVERFLOW            (0x0010)
    #define MPU_INT_STATUS_FSYNC_INT                (0x08)
    #define MPU_INT_STATUS_DMP_INT                  (0x02)
    #define MPU_INT_STATUS_RAW_RDY                  (0x01)
#define MPU_ACCEL_XOUT_H                            (0x3B)
#define MPU_ACCEL_XOUT_L                            (0x3C)
#define MPU_ACCEL_YOUT_H                            (0x3D)
#define MPU_ACCEL_YOUT_L                            (0x3E)
#define MPU_ACCEL_ZOUT_H                            (0x3F)
#define MPU_ACCEL_ZOUT_L                            (0x40)
/* T_deg = (T_out - Room)/snese + 21 deg */
#define MPU_TEMP_OUT_H                              (0x41)
#define MPU_TEMP_OUT_L                              (0x42)
#define MPU_GYRO_XOUT_H                             (0x43)
#define MPU_GYRO_XOUT_L                             (0x44)
#define MPU_GYRO_YOUT_H                             (0x45)
#define MPU_GYRO_YOUT_L                             (0x46)
#define MPU_GYRO_ZOUT_H                             (0x47)
#define MPU_GYRO_ZOUT_L                             (0x48)
#define MPU_EXT_SENS_DATA_00                        (0x49)
#define MPU_EXT_SENS_DATA_01                        (0x4A)
#define MPU_EXT_SENS_DATA_02                        (0x4B)
#define MPU_EXT_SENS_DATA_03                        (0x4C)
#define MPU_EXT_SENS_DATA_04                        (0x4D)
#define MPU_EXT_SENS_DATA_05                        (0x4E)
#define MPU_EXT_SENS_DATA_06                        (0x4F)
#define MPU_EXT_SENS_DATA_07                        (0x50)
#define MPU_EXT_SENS_DATA_08                        (0x51)
#define MPU_EXT_SENS_DATA_09                        (0x52)
#define MPU_EXT_SENS_DATA_10                        (0x53)
#define MPU_EXT_SENS_DATA_11                        (0x54)
#define MPU_EXT_SENS_DATA_12                        (0x55)
#define MPU_EXT_SENS_DATA_13                        (0x56)
#define MPU_EXT_SENS_DATA_14                        (0x57)
#define MPU_EXT_SENS_DATA_15                        (0x58)
#define MPU_EXT_SENS_DATA_16                        (0x59)
#define MPU_EXT_SENS_DATA_17                        (0x5A)
#define MPU_EXT_SENS_DATA_18                        (0x5B)
#define MPU_EXT_SENS_DATA_19                        (0x5C)
#define MPU_EXT_SENS_DATA_20                        (0x5D)
#define MPU_EXT_SENS_DATA_21                        (0x5E)
#define MPU_EXT_SENS_DATA_22                        (0x5F)
#define MPU_EXT_SENS_DATA_23                        (0x60)
#define MPU_I2C_SLV0_DO                             (0x63)
#define MPU_I2C_SLV1_DO                             (0x64)
#define MPU_I2C_SLV2_DO                             (0x65)
#define MPU_I2C_SLV3_DO                             (0x66)
#define MPU_I2C_MST_DELAY_CTRL                      (0x67)
    #define MPU_I2C_MST_DELAY_CTRL_DELAY_ES_SHADOW  (0x80)
    #define MPU_I2C_MST_DELAY_CTRL_I2C_SLV4_DLY_EN  (0x10)
    #define MPU_I2C_MST_DELAY_CTRL_I2C_SLV3_DLY_EN  (0x08)
    #define MPU_I2C_MST_DELAY_CTRL_I2C_SLV2_DLY_EN  (0x04)
    #define MPU_I2C_MST_DELAY_CTRL_I2C_SLV1_DLY_EN  (0x02)
    #define MPU_I2C_MST_DELAY_CTRL_I2C_SLV0_DLY_EN  (0x01)
#define MPU_SIGNAL_PATH_RESET                       (0x68)
    #define MPU_SIGNAL_PATH_RESET_GYRO_RST          (0x04)
    #define MPU_SIGNAL_PATH_RESET_ACCEL_RST         (0x02)
    #define MPU_SIGNAL_PATH_RESET_TEMP_RST          (0x01)
#define MPU_ACCEL_INTEL_CTRL                        (0x69)
    #define MPU_ACCEL_INTEL_CTRL_EN                 (0x80)
    #define MPU_ACCEL_INTEL_CTRL_MODE               (0x40)
#define MPU_USER_CTRL                               (0x6A)
    #define MPU_USER_CTRL_DMP_EN                    (0x80)
    #define MPU_USER_CTRL_FIFO_EN                   (0x40)
    #define MPU_USER_CTRL_I2C_MST_EN                (0x20)
    #define MPU_USER_CTRL_I2C_IF_DIS                (0x10)
    #define MPU_USER_CTRL_DMP_RST                   (0x08)
    #define MPU_USER_CTRL_FIFO_RST                  (0x04)
    #define MPU_USER_CTRL_I2C_MST_RST               (0x02)
    #define MPU_USER_CTRL_SIG_COND_RST              (0x01)
#define MPU_PWR_MGMT_1                              (0x6B)
    #define MPU_PWR_MGMT_1_DEVICE_RESET             (0x80)
    #define MPU_PWR_MGMT_1_SLEEP                    (0x40)
    #define MPU_PWR_MGMT_1_CYCLE                    (0x20)
    #define MPU_PWR_MGMT_1_GYRO_STANDBY             (0x10)
    #define MPU_PWR_MGMT_1_TEMP_DIS                 (0x08)
    #define MPU_PWR_MGMT_1_CLKSEL_REG               (0x07)
    #define MPU_PWR_MGMT_1_CLKSEL(x)                (0x07&(x))
#define MPU_PWR_MGMT_2                              (0x6C)
    #define MPU_PWR_MGMT_2_LP_WAKE_CTRL_REG         (0xC0)
    #define MPU_PWR_MGMT_2_LP_WAKE_CTRL(x)          (0x30&(x)<<6)
    #define MPU_PWR_MGMT_2_DISABLE_XA               (0x20)
    #define MPU_PWR_MGMT_2_DISABLE_YA               (0x10)
    #define MPU_PWR_MGMT_2_DISABLE_ZA               (0x08)
    #define MPU_PWR_MGMT_2_DISABLE_XG               (0x04)
    #define MPU_PWR_MGMT_2_DISABLE_YG               (0x02)
    #define MPU_PWR_MGMT_2_DISABLE_ZG               (0x01)
#define MPU_FIFO_COUNTH                             (0x72)
#define MPU_FIFO_COUNTL                             (0x73)
#define MPU_FIFO_R_W                                (0x74)
#define MPU_WHO_AM_I                                (0x75)        /* typical 0x70 */
#define MPU_XA_OFFSET_H                             (0x77)
#define MPU_XA_OFFSET_L                             (0x78)
#define MPU_YA_OFFSET_H                             (0x7A)
#define MPU_YA_OFFSET_L                             (0x7B)
#define MPU_ZA_OFFSET_H                             (0x7D)
#define MPU_ZA_OFFSET_L                             (0x7E)

#define MPU_ID                                      (0x70)
/** @} */


/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /*__MPU6500_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/

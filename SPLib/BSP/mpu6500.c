#include "mpu6500.h"

//    struct int_param_s int_param;
//    /* Set up gyro.
//     * Every function preceded by mpu_ is a driver function and can be found
//     * in inv_mpu.h.
//     */
//    int_param.cb = gyro_data_ready_cb;
//    int_param.pin = INT_PIN_P20;
//    int_param.lp_exit = INT_EXIT_LPM0;
//    int_param.active_low = 1;
//    result = mpu_init(&int_param);
//    if (result)
//        msp430_reset();

//    /* If you're not using an MPU9150 AND you're not using DMP features, this
//     * function will place all slaves on the primary bus.
//     * mpu_set_bypass(1);
//     */

//    /* Get/set hardware configuration. Start gyro. */
//    /* Wake up all sensors. */
//    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
//    /* Push both gyro and accel data into the FIFO. */
//    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
//    mpu_set_sample_rate(DEFAULT_MPU_HZ);
//    /* Read back configuration in case it was set improperly. */
//    mpu_get_sample_rate(&gyro_rate);
//    mpu_get_gyro_fsr(&gyro_fsr);
//    mpu_get_accel_fsr(&accel_fsr);

//    /* Initialize HAL state variables. */
//    memset(&hal, 0, sizeof(hal));
//    hal.sensors = ACCEL_ON | GYRO_ON;
//    hal.report = PRINT_QUAT;

//    /* To initialize the DMP:
//     * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
//     *    inv_mpu_dmp_motion_driver.h into the MPU memory.
//     * 2. Push the gyro and accel orientation matrix to the DMP.
//     * 3. Register gesture callbacks. Don't worry, these callbacks won't be
//     *    executed unless the corresponding feature is enabled.
//     * 4. Call dmp_enable_feature(mask) to enable different features.
//     * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
//     * 6. Call any feature-specific control functions.
//     *
//     * To enable the DMP, just call mpu_set_dmp_state(1). This function can
//     * be called repeatedly to enable and disable the DMP at runtime.
//     *
//     * The following is a short summary of the features supported in the DMP
//     * image provided in inv_mpu_dmp_motion_driver.c:
//     * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
//     * 200Hz. Integrating the gyro data at higher rates reduces numerical
//     * errors (compared to integration on the MCU at a lower sampling rate).
//     * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
//     * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
//     * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
//     * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
//     * an event at the four orientations where the screen should rotate.
//     * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
//     * no motion.
//     * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
//     * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
//     * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
//     * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
//     */
//    dmp_load_motion_driver_firmware();
//    dmp_set_orientation(
//        inv_orientation_matrix_to_scalar(gyro_orientation));
//    dmp_register_tap_cb(tap_cb);
//    dmp_register_android_orient_cb(android_orient_cb);
//    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
//        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
//        DMP_FEATURE_GYRO_CAL;
//    dmp_enable_feature(hal.dmp_features);
//    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
//    mpu_set_dmp_state(1);
//    hal.dmp_on = 1;

/* Data requested by client. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (100)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)
struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
    struct rx_s rx;
};




const float ACCEL_SEN = 16384.0f;
const float GYRO_SEN = 1880.0f;
const float MAG_SEN = 0.3f;
MPU6500_RAW_DATA mpu6500_raw_data;
MPU6500_REAL_DATA mpu6500_real_data;

// All 45.47 Hz peak
// DLPF_CFG(0) -> (5*n)Hz peak
float offset_x=0,offset_y=0,offset_z=0;
uint8_t mpu_data_buffer[64] = {0x00};
bool getDeviceID(void){
    return (bool)(MPU_ReadByte(MPU6500_WHO_AM_I) == MPU6500_ID);
}
/* Starting sampling rate. */
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};
static inline void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
    }
}
#define DEFAULT_MPU_HZ  (100)
void inv_mpu_init(void) {

#ifdef USING_MPU_DMP
    if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))    //设置需要的传感器
        printf("mpu_set_sensor complete ......\r\n");
    if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)) //设置fifo
        printf("mpu_configure_fifo complete ......\r\n");
    if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))              //设置采集样率
        printf("mpu_set_sample_rate complete ......\r\n");
    if(!dmp_load_motion_driver_firmware())                //加载dmp固件
        printf("dmp_load_motion_driver_firmware complete ......\r\n");
    if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
        printf("dmp_set_orientation complete ......\r\n"); //设置陀螺仪方向
    if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL))
        printf("dmp_enable_feature complete ......\r\n");
    if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))    //设置速率
        printf("dmp_set_fifo_rate complete ......\r\n");
    run_self_test();                          //自检
#else
    int result;
    /* Set up gyro.
     * Every function preceded by mpu_ is a driver function and can be found
     * in inv_mpu.h.
     */
    result = mpu_init(NULL);
    /* If you're not using an MPU9150 AND you're not using DMP features, this
     * function will place all slaves on the primary bus.
     * mpu_set_bypass(1);
     */
    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    /* Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
//    /* Read back configuration in case it was set improperly. */
//    mpu_get_sample_rate(&gyro_rate);
//    mpu_get_gyro_fsr(&gyro_fsr);
//    mpu_get_accel_fsr(&accel_fsr);

//    /* Initialize HAL state variables. */
//    memset(&hal, 0, sizeof(hal));
//    hal.sensors = ACCEL_ON | GYRO_ON;
//    hal.report = PRINT_QUAT;
    
#endif
    if(!mpu_set_dmp_state(1))                 //使能
        printf("mpu_set_dmp_state complete ......\r\n");
//    hal.dmp_on = 1;
}


void MPU_SetSampleRate(uint32_t rate);
void MPU6500_Init(void) {
    SPI5_Init();
    inv_mpu_init();
    {
//    SPI5_DMA();
//    uint8_t i = 0;
//    /* MPU working mode configurations */
//    if(getDeviceID()) {
//        delay_ms(10);
//        /* Reset MPU6500 */
//        mpu_data_buffer[i++] = MPU_ReadByte(MPU6500_PWR_MGMT_1);delay_ms(10);
//        MPU_WriteByte(MPU6500_PWR_MGMT_1, MPU_PWR_MGMT_1_DEVICE_RESET);
//        delay_ms(100);
//        /* Select MPU6500 clock source */
//        mpu_data_buffer[i++] = MPU_ReadByte(MPU6500_PWR_MGMT_1);delay_ms(10);
//        MPU_WriteByte(MPU6500_PWR_MGMT_1, MPU_PWR_MGMT_1_CLKSEL(3));
//        delay_ms(10);
//        /* Enable all accelemters and gyroscopes */
//        mpu_data_buffer[i++] = MPU_ReadByte(MPU6500_PWR_MGMT_2);delay_ms(10);
//        MPU_WriteByte(MPU6500_PWR_MGMT_2, 0x00);
//        delay_ms(10);
//        
//        /* Enable DMP, resst all sensors , DMP and clear all registers, disable I2C interface */
//        mpu_data_buffer[i++] = MPU_ReadByte(MPU6500_USER_CTRL);delay_ms(10);
//        MPU_WriteByte(MPU6500_USER_CTRL, 
//            MPU_USER_CTRL_DMP_EN|MPU_USER_CTRL_FIFO_EN|
//            MPU_USER_CTRL_I2C_IF_EN|MPU_USER_CTRL_SIG_COND_RST);
//        delay_ms(100);
//        
//        /* TEST CONFIGURATION */
//        /* Config sample rate 256Hz */
//        MPU_SetSampleRate(256);
//        /* Config gyrpscope and temperature snesor */
//        /* GYRO DLPF:none[bandwidth=8800Hz][delay=0.064ms][Fs=32kHz] */
//        /* TEMP DLPF:none[bandwidth=4000Hz][delay=0.04ms] */
//        mpu_data_buffer[i++] = MPU_ReadByte(MPU6500_CONFIG);delay_ms(10);
//        MPU_WriteByte(MPU6500_CONFIG, MPU_CONFIG_DLPG_CFG(2)|
//            MPU_GYRO_CONFIG_FCHOICE_B(0));
//        delay_ms(10);
//        /* GYTO [sensi=500deg/s] */
//        mpu_data_buffer[i++] = MPU_ReadByte(MPU6500_GYRO_CONFIG);delay_ms(10);
//        MPU_WriteByte(MPU6500_GYRO_CONFIG, MPU_GYRO_CONFIG_GYRO_FS_SEL(1));
//        delay_ms(10);
//        
//        /* Config accelemeter snesor */
//        /* ACCEL DLPF:none[bandwidth=1130Hz][delay=0.75ms][noise=220ug/rtHz][Fs=4kHz] */
//        mpu_data_buffer[i++] = MPU_ReadByte(MPU6500_ACCEL_CONFIG_2);delay_ms(10);
//        MPU_WriteByte(MPU6500_ACCEL_CONFIG_2, MPU_ACCEL_CONFIG_2_A_DLPF_CFG(2));
//        delay_ms(10);
//        /* ACCEL [sensi=+-2g] */
//        mpu_data_buffer[i++] = MPU_ReadByte(MPU6500_ACCEL_CONFIG);delay_ms(10);
//        MPU_WriteByte(MPU6500_ACCEL_CONFIG, MPU_ACCEL_CONFIG_ACCEL_FS_SEL(0));
//        delay_ms(10);
//        
//        /* Disable all I2C devices */
//        mpu_data_buffer[i++] = MPU_ReadByte(MPU6500_INT_PIN_CFG);delay_ms(10);
//        MPU_WriteByte(MPU6500_INT_PIN_CFG, MPU_INT_PIN_CFG_BYPASS_EN);
//        delay_ms(10);
//        
//        /* Enable interrupts and auto-clear flags */
//        mpu_data_buffer[i++] = MPU_ReadByte(MPU6500_INT_ENABLE);delay_ms(10);
//        MPU_WriteByte(MPU6500_INT_ENABLE, MPU_INT_ENABLE_RAW_RDY_EN);
//        delay_ms(10);
//    }
    }
    /* Condif EXIT for MPU interrupt */
    spRCC_Set_SYSCFG();
    GPIO_IN_Config(GPIOB, GPIO_Pin_8, GPIO_PuPd_UP, GPIO_Speed_100MHz);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, GPIO_PinSource8); 
    EXTI_InitTypeDef            exit_initer;
    exit_initer.EXTI_Line       = EXTI_Line8;
    exit_initer.EXTI_LineCmd    = ENABLE;
    exit_initer.EXTI_Mode       = EXTI_Mode_Interrupt;
    exit_initer.EXTI_Trigger    = EXTI_Trigger_Rising;
    EXTI_Init(&exit_initer);
//    NVIC_IRQEnable(EXTI9_5_IRQn, 1, 3);
    delay_ms(5);
}

void MPU6500_Read(void){
    uint8_t buf[14];

    MPU_Read(MPU6500_ACCEL_XOUT_H,buf,14);

    mpu6500_raw_data.Accel_X=(short)((buf[0])<<8) | buf[1];
    mpu6500_real_data.Accel_X=mpu6500_raw_data.Accel_X/ACCEL_SEN;

    mpu6500_raw_data.Accel_Y=(short)((buf[2])<<8 )| buf[3];
    mpu6500_real_data.Accel_Y=mpu6500_raw_data.Accel_Y/ACCEL_SEN;

    mpu6500_raw_data.Accel_Z=(short)((buf[4])<<8 )| buf[5];
    mpu6500_real_data.Accel_Z=mpu6500_raw_data.Accel_Z/ACCEL_SEN;

    mpu6500_raw_data.Temp=(short)((buf[6])<<8 )| buf[7];
    mpu6500_real_data.Temp=mpu6500_raw_data.Temp;

    mpu6500_raw_data.Gyro_X=(short)((buf[8])<<8 )| buf[9];
    mpu6500_real_data.Gyro_X=mpu6500_raw_data.Gyro_X/GYRO_SEN -offset_x;

    mpu6500_raw_data.Gyro_Y=(short)((buf[10])<<8 )| buf[11];
    mpu6500_real_data.Gyro_Y=mpu6500_raw_data.Gyro_Y/GYRO_SEN -offset_y;

    mpu6500_raw_data.Gyro_Z=(short)((buf[12])<<8 )| buf[13];
    mpu6500_real_data.Gyro_Z=mpu6500_raw_data.Gyro_Z/GYRO_SEN -offset_z;

}
void MPU6500_ReadACCEL(void){
    uint8_t buf[6];
    MPU_Read(MPU6500_ACCEL_XOUT_H,buf,6);
    mpu6500_raw_data.Accel_X=(short)((buf[0])<<8) | buf[1];
    mpu6500_real_data.Accel_X=mpu6500_raw_data.Accel_X/ACCEL_SEN;
    mpu6500_raw_data.Accel_Y=(short)((buf[2])<<8 )| buf[3];
    mpu6500_real_data.Accel_Y=mpu6500_raw_data.Accel_Y/ACCEL_SEN;
    mpu6500_raw_data.Accel_Z=(short)((buf[4])<<8 )| buf[5];
    mpu6500_real_data.Accel_Z=mpu6500_raw_data.Accel_Z/ACCEL_SEN;
}

void MPU6500ReadGYRO(void){
    uint8_t buf[6];
    MPU_Read(MPU6500_GYRO_XOUT_H,buf,6);
    mpu6500_raw_data.Gyro_X=(short)((buf[0])<<8 )| buf[1];
    mpu6500_real_data.Gyro_X=mpu6500_raw_data.Gyro_X/GYRO_SEN - offset_x;
    mpu6500_raw_data.Gyro_Y=(short)((buf[2])<<8 )| buf[3];
    mpu6500_real_data.Gyro_Y=mpu6500_raw_data.Gyro_Y/GYRO_SEN - offset_y;
    mpu6500_raw_data.Gyro_Z=(short)((buf[4])<<8 )| buf[5];
    mpu6500_real_data.Gyro_Z=mpu6500_raw_data.Gyro_Z/GYRO_SEN - offset_z;
}

uint16_t MPU6500_Read_FIFO(void){
    uint16_t size = MPU_ReadByte(MPU6500_FIFO_COUNTH)<<8;
    size |= MPU_ReadByte(MPU6500_FIFO_COUNTL);
    uint8_t i=0;
    while(i<size) {
        mpu_data_buffer[i++] = MPU_ReadByte(MPU6500_FIFO_R_W);
    }
    return size;
}



#define INTERNAL_SAMPRATE           1000
void MPU_SetSampleRate(uint32_t rate) {
    uint8_t samp_div = INTERNAL_SAMPRATE/rate-1;
    MPU_WriteByte(MPU6500_SMPLRT_DIV, MPU_SMPLRT_DIV(samp_div));
}


void gyro_offset(){
    uint8_t buf[6];
    short last_x=0,last_y=0,last_z=0;
    short gyro_x,gyro_y,gyro_z;
    long x_add=0,y_add=0,z_add=0;
    unsigned short i;

    for(i=0;i<1000;i++){
        MPU_Read(MPU6500_GYRO_XOUT_H,buf,6);
        gyro_x=(short)((buf[0])<<8 )| buf[1];
        gyro_y=(short)((buf[2])<<8 )| buf[3];
        gyro_z=(short)((buf[4])<<8 )| buf[5];
        if(fabs(gyro_x-last_x)>100||fabs(gyro_y-last_y)>100||fabs(gyro_z-last_z)>100){
            static unsigned int num=0;
            num+=i;
            i=0;
            last_x=last_y=last_z=0;
            x_add=y_add=z_add=0;
            if(num>500){
                return ;
            }
            continue;
        }
        x_add+=gyro_x;
        y_add+=gyro_y;
        z_add+=gyro_z;
        last_x=gyro_x;
        last_y=gyro_y;
        last_z=gyro_z;
        delay_ms(1);
    }
    offset_x=x_add/1000.f/GYRO_SEN;
    offset_y=y_add/1000.f/GYRO_SEN;
    offset_z=z_add/1000.f/GYRO_SEN;

}



uint8_t MPU_ReadByte(uint8_t reg){
    uint8_t res;
    SPI5_NSS_Select;
    SPI5_ReadWriteByte(reg|0x80);
    res=SPI5_ReadWriteByte(0xff);
    SPI5_NSS_Release;
    return res;
}
uint8_t MPU_Read(uint8_t reg,uint8_t *buf,uint8_t len){
    uint8_t i;
    SPI5_NSS_Select;
    SPI5_ReadWriteByte(reg|0x80);
    for(i=0;i<len;i++){
        *buf=SPI5_ReadWriteByte(0xff);
        buf++;
    }
    SPI5_NSS_Release;
    return 0;
}
uint8_t MPU_WriteByte(uint8_t reg,uint8_t data){
    SPI5_NSS_Select;
    SPI5_ReadWriteByte(reg);
    SPI5_ReadWriteByte(data);
    SPI5_NSS_Release;
    return 0;
}
uint8_t MPU_Write(uint8_t reg,uint8_t *buf,uint8_t len){
    SPI5_NSS_Select;
    SPI5_ReadWriteByte(reg);
    while(len--){
        SPI5_ReadWriteByte(*buf++);
    }
    SPI5_NSS_Release;
    delay_us(1000);
    return 0;
}



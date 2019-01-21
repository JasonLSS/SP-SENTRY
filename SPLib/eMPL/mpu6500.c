#include <math.h>
#include "mpu6500.h"
#include "ist8310driver.h"
#include "ist8310driver_middleware.h"

/* Data requested by client. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (500)

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


// SCK  PF7
// MOSI PF9
// MISO PF8
// NSS  PF6
// INT  PB8

const float ACCEL_SEN = 16384.0f;   //
const float GYRO_SEN = 1880.0f;     //  
const float MAG_SEN = 0.3f;         // uT
MPU_RAW_DATA mpu6500_raw_data;
MPU_REAL_DATA mpu6500_real_data;

// All 45.47 Hz peak
// DLPF_CFG(0) -> (5*n)Hz peak
float offset_x=0,offset_y=0,offset_z=0;
uint8_t mpu_data_buffer[64] = {0x00};
bool getDeviceID(void){
    return (bool)(MPU_ReadByte(MPU_WHO_AM_I) == MPU_ID);
}


#ifdef USING_MPU_DMP

#include "inv_mpu.h"

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

void inv_mpu_init(void) {
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
    
////    int result;
//    /* Set up gyro.
//     * Every function preceded by mpu_ is a driver function and can be found
//     * in inv_mpu.h.
//     */
//    mpu_init(NULL);
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
////    /* Read back configuration in case it was set improperly. */
////    mpu_get_sample_rate(&gyro_rate);
////    mpu_get_gyro_fsr(&gyro_fsr);
////    mpu_get_accel_fsr(&accel_fsr);

////    /* Initialize HAL state variables. */
////    memset(&hal, 0, sizeof(hal));
////    hal.sensors = ACCEL_ON | GYRO_ON;
////    hal.report = PRINT_QUAT;


//    if(!mpu_set_dmp_state(1))                 //使能
//        printf("mpu_set_dmp_state complete ......\r\n");
////    hal.dmp_on = 1;
}
#endif


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

static uint8_t ist8310_write_reg(uint8_t addr, uint8_t data) {
    MPU_WriteByte(MPU_I2C_SLV1_CTRL, 0x00);     // Turn off slave 1 at first
    delay_ms(2);
    MPU_WriteByte(MPU_I2C_SLV1_REG, addr);
    delay_ms(2);
    MPU_WriteByte(MPU_I2C_SLV1_DO, data);
    delay_ms(2);
    // Turn on slave 1 with one byte transmitting
    MPU_WriteByte(MPU_I2C_SLV1_CTRL, 0x80 | 0x01);
    delay_ms(10);
    return 1;
}

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

/* Static function for IST8310 */
uint8_t ist8310_INIT(void) {
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
        return 0;
    }
    // Reset ist8310
    ist8310_write_reg(IST8310_R_CONFB, 0x01);
    delay_ms(10);
    // Config as ready mode to access reg
    ist8310_write_reg(IST8310_R_CONFA, 0x00);
    if(ist8310_read_reg(IST8310_R_CONFA) != 0x00) {
        // bsp_error_handler(__FUNCTION__, __LINE__, "IST ready mode failed.");
        return 0;
    }
    delay_ms(10);
    // Normal state, no int
    ist8310_write_reg(IST8310_R_CONFB, 0x00);
    if(ist8310_read_reg(IST8310_R_CONFB) != 0x00) {
        // bsp_error_handler(__FUNCTION__, __LINE__, "IST normal state init failed.");
        return 0;
    }
    delay_ms(10);
    // Config low noise mode, x,y,z axis 16 time 1 avg,
    ist8310_write_reg(IST8310_AVGCNTL, 0x24); // 100100
    if(ist8310_read_reg(IST8310_AVGCNTL) != 0x24) {
        // bsp_error_handler(__FUNCTION__, __LINE__, "IST low noise mode failed.");
        return 0;
    }
    delay_ms(10);
    // Set/Reset pulse duration setup, normal mode
    ist8310_write_reg(IST8310_PDCNTL, 0xC0);
    if(ist8310_read_reg(IST8310_PDCNTL) != 0xC0) {
        // bsp_error_handler(__FUNCTION__, __LINE__, "IST pulse duration set failed.");
        return 0;
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
    return 1;
}


void ist8310_get_data(float* mag ) {
    if(mag) {
        uint8_t ist_buff[6];
        MPU_Read(MPU_EXT_SENS_DATA_00, ist_buff, 6);
        mag[0] = (int16_t)(ist_buff[1]<<8 | ist_buff[0]) * MAG_SEN;
        mag[1] = (int16_t)(ist_buff[3]<<8 | ist_buff[2]) * MAG_SEN;
        mag[2] = (int16_t)(ist_buff[5]<<8 | ist_buff[4]) * MAG_SEN;
    }
}





int MPU6500_Read(void){
    uint8_t buf[14];

    MPU_Read(MPU_ACCEL_XOUT_H,buf,14);

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
    
    return 0;
}

void MPU6500_Stream(float* gyro, float* accel, float* temp, float* mag){
    MPU_Read(MPU_ACCEL_XOUT_H, mpu_data_buffer, 14);
    if(accel) {
        accel[0] = (short)(((mpu_data_buffer[0])<<8)|mpu_data_buffer[1]) /ACCEL_SEN;
        accel[1] = (short)(((mpu_data_buffer[2])<<8)|mpu_data_buffer[3]) /ACCEL_SEN;
        accel[2] = (short)(((mpu_data_buffer[4])<<8)|mpu_data_buffer[5]) /ACCEL_SEN;
    }
    if(temp) *temp= (short)(((mpu_data_buffer[6])<<8)|mpu_data_buffer[7])/340.f+36.53f;
    if(gyro){
        gyro[0] = (short)(((mpu_data_buffer[8])<<8)|mpu_data_buffer[9]) /GYRO_SEN;
        gyro[1] = (short)(((mpu_data_buffer[10])<<8)|mpu_data_buffer[11]) /GYRO_SEN;
        gyro[2] = (short)(((mpu_data_buffer[12])<<8)|mpu_data_buffer[13]) /GYRO_SEN;
    }
    ist8310_get_data(mag);
}

void MPU_ReadACCEL(void){
    uint8_t buf[6];
    MPU_Read(MPU_ACCEL_XOUT_H,buf,6);
    mpu6500_raw_data.Accel_X=(short)((buf[0])<<8) | buf[1];
    mpu6500_real_data.Accel_X=mpu6500_raw_data.Accel_X/ACCEL_SEN;
    mpu6500_raw_data.Accel_Y=(short)((buf[2])<<8 )| buf[3];
    mpu6500_real_data.Accel_Y=mpu6500_raw_data.Accel_Y/ACCEL_SEN;
    mpu6500_raw_data.Accel_Z=(short)((buf[4])<<8 )| buf[5];
    mpu6500_real_data.Accel_Z=mpu6500_raw_data.Accel_Z/ACCEL_SEN;
}

void MPU_ReadGYRO(void){
    uint8_t buf[6];
    MPU_Read(MPU_GYRO_XOUT_H,buf,6);
    mpu6500_raw_data.Gyro_X=(short)((buf[0])<<8 )| buf[1];
    mpu6500_real_data.Gyro_X=mpu6500_raw_data.Gyro_X/GYRO_SEN - offset_x;
    mpu6500_raw_data.Gyro_Y=(short)((buf[2])<<8 )| buf[3];
    mpu6500_real_data.Gyro_Y=mpu6500_raw_data.Gyro_Y/GYRO_SEN - offset_y;
    mpu6500_raw_data.Gyro_Z=(short)((buf[4])<<8 )| buf[5];
    mpu6500_real_data.Gyro_Z=mpu6500_raw_data.Gyro_Z/GYRO_SEN - offset_z;
}

static uint8 len = 12;

uint16_t MPU6500_Read_FIFO(void){
    MPU_Read(MPU_FIFO_R_W, mpu_data_buffer, len);
    return 0;
}

uint16_t MPU6500_Read_FIFO_Stream(short* gyro, short* accel){
    MPU6500_Read_FIFO();
    accel[0] = (short)(((mpu_data_buffer[0])<<8)|mpu_data_buffer[1]);
    accel[1] = (short)(((mpu_data_buffer[2])<<8)|mpu_data_buffer[3]);
    accel[2] = (short)(((mpu_data_buffer[4])<<8)|mpu_data_buffer[5]);
    gyro[0] = (short)(((mpu_data_buffer[6])<<8)|mpu_data_buffer[7]);
    gyro[1] = (short)(((mpu_data_buffer[8])<<8)|mpu_data_buffer[9]);
    gyro[2] = (short)(((mpu_data_buffer[10])<<8)|mpu_data_buffer[11]);
    return 0;
}

uint16_t MPU6500_Reset_FIFO(void){
    uint8_t data = MPU_ReadByte(MPU_USER_CTRL);
    delay_us(20);
    data |= MPU_USER_CTRL_FIFO_RST;
    MPU_WriteByte(MPU_USER_CTRL, data);
    delay_us(100);
    return 0;
}


#define INTERNAL_SAMPRATE           1000
void MPU_SetSampleRate(uint32_t rate) {
    uint8_t samp_div = INTERNAL_SAMPRATE/rate-1;
    MPU_WriteByte(MPU_SMPLRT_DIV, MPU_SMPLRT_DIV_(samp_div));
}


void gyro_offset(){
    uint8_t buf[6];
    short last_x=0,last_y=0,last_z=0;
    short gyro_x,gyro_y,gyro_z;
    long x_add=0,y_add=0,z_add=0;
    unsigned short i;

    for(i=0;i<1000;i++){
        MPU_Read(MPU_GYRO_XOUT_H,buf,6);
        gyro_x=(short)((buf[0])<<8 )| buf[1];
        gyro_y=(short)((buf[2])<<8 )| buf[3];
        gyro_z=(short)((buf[4])<<8 )| buf[5];
        if(abs(gyro_x-last_x)>100||abs(gyro_y-last_y)>100||abs(gyro_z-last_z)>100){
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



int MPU6500_Init(void) {
    
    int result = 0;
    
    SPI5_Init();
    
    /* Condif EXIT for MPU interrupt */
    GPIO_IN_Config(GPIOB, GPIO_Pin_8, GPIO_PuPd_UP, GPIO_Speed_100MHz);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, GPIO_PinSource8); 
    EXTI_InitTypeDef            exit_initer;
    exit_initer.EXTI_Line       = EXTI_Line8;
    exit_initer.EXTI_LineCmd    = ENABLE;
    exit_initer.EXTI_Mode       = EXTI_Mode_Interrupt;
    exit_initer.EXTI_Trigger    = EXTI_Trigger_Rising;
    EXTI_Init(&exit_initer);
//    NVIC_IRQEnable(EXTI9_5_IRQn, 1, 3);

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
    
#ifdef USING_MPU_DMP
    inv_mpu_init();
#else
    {
//    SPI5_DMA();
//    uint8_t i = 0;
    uint8_t data;
    /* MPU working mode configurations */
    if(getDeviceID()) {
        delay_ms(10);
        /*----------------------------------------*/
        /* BASIS CONFIGURATION */
        /*----------------------------------------*/
        /* Reset MPU6500 */
        data = MPU_PWR_MGMT_1_DEVICE_RESET;
        MPU_WriteByte(MPU_PWR_MGMT_1, data);delay_ms(100);

        /* Select MPU6500 clock source */
        data = MPU_PWR_MGMT_1_CLKSEL(1);
        MPU_WriteByte(MPU_PWR_MGMT_1, data);delay_ms(10);
        
        /*----------------------------------------*/
        /* SENSOR CONFIGURATION */
        /*----------------------------------------*/
        /* Enable all accelemters and gyroscopes */
        data = 0x00;
        MPU_WriteByte(MPU_PWR_MGMT_2, data);delay_ms(10);

        /* Config sample rate */
        MPU_SetSampleRate(200);
        /* Config gyrpscope and temperature snesor */
        /* GYRO DLPF:none[bandwidth=8800Hz][delay=0.064ms][Fs=32kHz] */
        /* TEMP DLPF:none[bandwidth=4000Hz][delay=0.04ms] */
        data = MPU_CONFIG_DLPG_CFG(2) | MPU_GYRO_CONFIG_FCHOICE_B(0);
        MPU_WriteByte(MPU_CONFIG, data);delay_ms(10);
        
        /* GYRO [sensi=500deg/s] */
        data = MPU_GYRO_CONFIG_GYRO_FS_SEL(1);
        MPU_WriteByte(MPU_GYRO_CONFIG, data);delay_ms(10);

        /* Config accelemeter snesor */
        /* ACCEL DLPF:none[bandwidth=1130Hz][delay=0.75ms][noise=220ug/rtHz][Fs=4kHz] */
        data = MPU_ACCEL_CONFIG_2_A_DLPF_CFG(2);
        MPU_WriteByte(MPU_ACCEL_CONFIG_2, data);delay_ms(10);

        /* ACCEL [sensi=+-2g] */
        data = MPU_ACCEL_CONFIG_ACCEL_FS_SEL(0);
        MPU_WriteByte(MPU_ACCEL_CONFIG, data);delay_ms(10);
        
        
        /*----------------------------------------*/
        /* I2C CONFIGURATION FOR IST8310 */
        /*----------------------------------------*/
        data = MPU_INT_PIN_CFG_BYPASS_EN;
        MPU_WriteByte(MPU_INT_PIN_CFG, data);delay_ms(10);
        
        data = (~MPU_I2C_MST_CTRL_MULI_MST_EN) | 
           (MPU_I2C_MST_CTRL_WAIT_FOR_ES) |
           (~MPU_I2C_MST_CTRL_SLV_3_FIFO_EN) |
           (~MPU_I2C_MST_CTRL_I2C_MST_P_NSR) |
           MPU_I2C_MST_CLK_400_KHZ;    // 400kHz, 20MHzdelay_ms(10);
        MPU_WriteByte(MPU_I2C_MST_CTRL, data);delay_ms(10);
        
        data = MPU_I2C_MST_DELAY_CTRL_I2C_SLV0_DLY_EN;
        MPU_WriteByte(MPU_I2C_MST_DELAY_CTRL, data);delay_ms(10);
        
        
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
        MPU_WriteByte(MPU_USER_CTRL, data);delay_ms(10);
        
        
        /*----------------------------------------*/
        /* IST8310 CONFIGURATION */
        /*----------------------------------------*/
        /* Init IST8310 */
        result = ist8310_INIT();
        delay_ms(50);
        
        /*----------------------------------------*/
        /* MPU INTERRUPT CONFIGURATION */
        /*----------------------------------------*/
        /* Enable interrupts and auto-clear flags */
        data = MPU_INT_ENABLE_RAW_RDY_EN;
        MPU_WriteByte(MPU_INT_ENABLE, data);delay_ms(10);
    }
    }
#endif

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

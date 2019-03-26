/************************************************************
 *File        :    Auto_aim.c
 *Author    : @TangJiaxin ,tjx1024@126.com
 *Version    : 
 *Update    : 
 *Description:     
 ************************************************************/
#include "Auto_aim.h"
#include "gimbal.h"
#include "sp_utility.h"
 
frame frame_ex;//存储上一个视觉发来的结构体
frame fram;//存储视觉数据的结构体
/*
typedef struct _frame            //视觉发来的数据框架    
{
    u8 head[2];        //帧头，为0xffff
    u32 timestamp;//包序号
    float yaw;      //yaw误差值
    float pitch;  //pitch误差值
    u32 extra[2]; //additional imformation    
    u8 crc8check;    //crc8校验位                
}frame;
*/

double  parameter_yaw=0.008;
double  parameter_pitch=0.005;
double  view_ch2;
double  view_ch3;
double  virual_ch2;
double  virual_ch3;

double  last_view_ch2;
double  last_view_ch3;
double  view_ch2_interval;
double  view_ch3_interval;

double  step_ch2=0.0;
double  step_ch3=0.0;
double  last_step_ch2=0.0;
double  last_step_ch3=0.0;

double  visual_yaw_kp=1.0;  //1.0
double  visual_yaw_ki=0.3;
double  visual_yaw_kd=1.5; //0.45
double    integral_yaw=0.0;

double  visual_pitch_kp=1.0;
double  visual_pitch_ki=0.02;
double  visual_pitch_kd=0.8;
double    integral_pitch=0.0;
double  now_yaw_angle=0.0;
double  last_yaw_angle=0.0;

double  visual_yaw_pid_out;
double  visual_pitch_pid_out;

int     FirstFlag=0;
int     time_count=0;
int     time_interval=40;
int     if_newframe = 0;//CAN2收到视觉信号标志位
int     if_rotate_ok = 0;
int         if_if_newframe=0;

int miss = 0;
float last_pitch=0;
float last_yaw=0;
char uart6_buff[256];

uint8_t auto_aim_flag = 0;
uint8_t small_power_flag = 0;

uint32_t last_time_tick_1ms,last_last_time_tick_1ms;//上一个time_tick_1ms的数，上上个time_tick_1ms的数
/***************************************************************************************
 *Name     : Auto_aim
 *Function ：自动
 *Input    ：rx_buf(视觉从串口2传来的数组),len
 *Output   ：无 
 *Description :以视觉发送的数据（pitch、yaw）作为误差值，以误差为0为目标，
               做PID运算，来控制云台电机目标值的变化
****************************************************************************************/
void Auto_aim(u8 *rx_buf,int len)
{
	 if(unpackFrame(rx_buf,len,&fram) == 0)  //解包成功
    {    
        if(fram.timestamp != frame_ex.timestamp)//如果前一帧数据和当前帧时间戳一样,目标丢失,不作为
        {
            if_newframe = 1;
        }
				else
				{
					if_newframe = 0;
				}
        frame_ex.timestamp = fram.timestamp;
    }
		else
		{
			if_newframe = 0;
		}
    if(auto_aim_flag == 0x00 && small_power_flag == 0xFF)
    { 
        if(if_newframe == 1) {
            spGIMBAL_Controller.user.update_target_limit(
                spGIMBAL_Controller._target.gimbal_pitch_motor->state.angle + fram.pitch/0.04394f,
                spGIMBAL_Controller._target.gimbal_yaw_motor->state.angle - fram.yaw/0.04394f);
            //yaw 10 5 0.5    pitch  2 10 0.2
            if((-last_yaw+fram.yaw)!=1)
                miss++;
            u8 size = sprintf(uart6_buff, "%f,%d\r\n", fram.yaw, miss);
            spDMA_Controllers.controller.start(spDMA_UART7_tx_stream, (uint32_t)uart6_buff, (uint32_t)&UART7->DR, size);
            last_yaw=fram.yaw;
        }
    }
}


static uint8_t __view_buffer[128];
struct {
    UsartBuffer_t buffer;
    float stamp, stamp_ex;
    float freq;
} view_buffer = {
    .buffer = {
        .buffer = __view_buffer,
        .size = 128,
        .curr_ptr = 0,
        .last_ptr = 0
    },
    .stamp = 0.f,
    .stamp_ex = 0.f,
    .freq = 0.f
};

void Autoaim_Init(void) {
    USART_RX_Config(USART2, 115200);
    DMA_USART_RX_Config(USART2, (uint32_t)view_buffer.buffer.buffer, view_buffer.buffer.size, false);
    USART_TX_Config(USART2, 115200);
    DMA_USART_TX_Config(USART2);
//        DMA_ITConfig(spDMA_USART2_rx_stream, DMA_IT_TC, ENABLE);
    DMA_Cmd(spDMA_USART2_rx_stream, ENABLE);
    spIRQ_Manager.registe(USART2_IRQn, USART_IT_IDLE, Autoaim_USART_Interface);
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
    USART_Cmd(USART2, ENABLE);
    sendtoComputerInit();
}

void Autoaim_USART_Interface(void) {
    view_buffer.stamp_ex = view_buffer.stamp;
    view_buffer.stamp = TASK_GetSecond();
    view_buffer.freq = view_buffer.stamp - view_buffer.stamp_ex;
    
    uint16_t size = view_buffer.buffer.size - spDMA_USART2_rx_stream->NDTR;
    spDMA_Controllers.controller.reset_counter(spDMA_USART2_rx_stream, view_buffer.buffer.size);
    Auto_aim(view_buffer.buffer.buffer, size);
    
    LED8_BIT_ON(LED8_BIT4);
    delay_us(100);
    LED8_BIT_OFF(LED8_BIT4);
}



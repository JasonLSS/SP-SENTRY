/************************************************************
 *File        :    Auto_aim.c
 *Author    : @TangJiaxin ,tjx1024@126.com
 *Version    : 
 *Update    : 
 *Description:     
 ************************************************************/
#include "Auto_aim.h"
#include "gimbal.h"
 
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
            #ifdef little_board//如果安装了周陈旺的板子

                if(auto_aim_flag == 0xFF && small_power_flag == 0x00)//如果开启了自瞄而没开启打符模式
                {
                    if(if_newframe == 1)//如果从CAN2收到了视觉信号
                 {
                        last_last_time_tick_1ms=last_time_tick_1ms;;//更新时钟
                        last_time_tick_1ms=time_tick_1ms;//controltask.c里面的计数变量，1ms加一次
                        if(FirstFlag==1)
                        {
                                last_view_ch2 = parameter_yaw*Gimbal_info.yaw;//
                                last_view_ch3 = parameter_pitch*Gimbal_info.pitch;//
                                view_ch2 = last_view_ch2;
                                view_ch3 = last_view_ch3;
                                view_ch2_interval=0;
                                view_ch3_interval=0;
                                time_interval=30;
                        }
                        else
                        {
                                last_view_ch2 = view_ch2;
                                last_view_ch3 = view_ch3;
                                view_ch2 = parameter_yaw*Gimbal_info.yaw;
                                view_ch3 = parameter_pitch*Gimbal_info.pitch;
                                view_ch2_interval=view_ch2-last_view_ch2;
                                view_ch3_interval=view_ch3-last_view_ch3;
                                time_interval=last_time_tick_1ms-last_last_time_tick_1ms;
                                if(time_interval<0)
                                    time_interval=time_interval+10000;
                                if(FirstFlag<=100)
                                    FirstFlag++;
                        }

                                time_count=0;
                    }
                 
                if(time_count<60)
                {
                    last_step_ch2=step_ch2;
                    last_step_ch3=step_ch3;
                    
                    step_ch2=    view_ch2 +view_ch2_interval/time_interval*time_count;//线性插值预测
                    step_ch3=    view_ch3 +view_ch3_interval/time_interval*time_count;//线性插值预测
                    
                    visual_yaw_pid_out=visual_yaw_kp*step_ch2+visual_yaw_kd*(step_ch2-last_step_ch2);//PD控制
                    visual_pitch_pid_out=visual_pitch_kp*step_ch3+visual_pitch_kd*(step_ch3-last_step_ch3);//PD控制
                    
                    
                        #ifdef gongkong
                            
                            if(FirstFlag<=20)
                            {
                                Gimbal_control.angle_yaw_set    -= fabs(visual_yaw_pid_out)< 2.0 ? visual_yaw_pid_out :(visual_yaw_pid_out>0 ? 2.0:-2.0);
                                Gimbal_control.angle_pitch_set  += fabs(visual_pitch_pid_out)<1.75? visual_pitch_pid_out :(visual_pitch_pid_out>0 ? 1.75 : -1.75);
                            }
                            else
                            {
                                Gimbal_control.angle_yaw_set    -= fabs(visual_yaw_pid_out)< 1.0? visual_yaw_pid_out :(visual_yaw_pid_out>0 ? 1.0:-1.0);
                                Gimbal_control.angle_pitch_set  += fabs(visual_pitch_pid_out)<5.0? visual_pitch_pid_out :(visual_pitch_pid_out>0 ? 5.0 : -5.0);
                            }
                            
                        #endif
                                
                        #ifdef miaosuan
                        if(auto_aim_flag == 0xFF && small_power_flag == 0x00)
                        {
                            if(FirstFlag<=20)
                            {
                                Gimbal_control.angle_yaw_set    -= fabs(visual_yaw_pid_out)< 1.0 ? visual_yaw_pid_out :(visual_yaw_pid_out>0 ? 1.0:-1.0);
                                Gimbal_control.angle_pitch_set  += fabs(visual_pitch_pid_out)<1.75? visual_pitch_pid_out :(visual_pitch_pid_out>0 ? 1.75 : -1.75);
                            }
                            else
                            {
                                Gimbal_control.angle_yaw_set    -= fabs(visual_yaw_pid_out)< 1.0? visual_yaw_pid_out :(visual_yaw_pid_out>0 ? 1.0:-1.0);
                                Gimbal_control.angle_pitch_set  += fabs(visual_pitch_pid_out)<5.0? visual_pitch_pid_out :(visual_pitch_pid_out>0 ? 5.0 : -5.0);
                            }
                        }else if(auto_aim_flag == 0x00 && small_power_flag == 0xFF){
                            if(FirstFlag<=20)
                            {
                                Gimbal_control.angle_yaw_set    -= fabs(visual_yaw_pid_out)< 3.0 ? visual_yaw_pid_out :(visual_yaw_pid_out>0 ? 3.0:-3.0);
                                Gimbal_control.angle_pitch_set  += fabs(visual_pitch_pid_out)<1.75? visual_pitch_pid_out :(visual_pitch_pid_out>0 ? 1.75 : -1.75);
                            }
                            else
                            {
                                Gimbal_control.angle_yaw_set    -= fabs(visual_yaw_pid_out)< 1.0? visual_yaw_pid_out :(visual_yaw_pid_out>0 ? 1.0:-1.0);
                                Gimbal_control.angle_pitch_set  += fabs(visual_pitch_pid_out)<5.0? visual_pitch_pid_out :(visual_pitch_pid_out>0 ? 5.0 : -5.0);
                            }
                            
                        }
                        #endif

                }
                    time_count++;
                
                 
                if(time_count>100)
                {
                     FirstFlag=1;
                }
                
                     if_newframe = 0;
                 
                }
             else if(auto_aim_flag == 0x00 &&small_power_flag == 0xFF)
             { 
                 if(if_newframe == 1)
                 {
                        Gimbal_control.angle_yaw_set    -= fabs(Gimbal_info.yaw)< 16.0?  Gimbal_info.yaw :( Gimbal_info.yaw >0 ? 16.0:-16.0);
                        Gimbal_control.angle_pitch_set  += fabs(Gimbal_info.pitch)<7.0? Gimbal_info.pitch :(Gimbal_info.pitch >0 ? 7.0 : -7.0);
                     
                     if_newframe = 0;
                    
                     if_rotate_ok = 1;
                 }     
             }
    #elif !defined(SP19)
  //如果没用小板子，利用串口2读取视觉数据
    if(unpackFrame(rx_buf,len,&fram) == 0)  //解包成功
    {    

        if(fram.timestamp != frame_ex.timestamp)//如果前一帧数据和当前帧时间戳一样,目标丢失,不作为
            {
                
                if_newframe = 1;            
                
            }
            
                frame_ex.timestamp = fram.timestamp;
     }
    if(auto_aim_flag == 0xFF && small_power_flag == 0x00)
        {
            if(if_newframe == 1)
         {
                 last_last_time_tick_1ms=last_time_tick_1ms;;
                last_time_tick_1ms=time_tick_1ms;
                if(FirstFlag==1)
                {
                        last_view_ch2 = parameter_yaw*fram.yaw;
                        last_view_ch3 = parameter_pitch*fram.pitch;
                        view_ch2 = last_view_ch2;
                        view_ch3 = last_view_ch3;
                        view_ch2_interval=0;
                        view_ch3_interval=0;
                      time_interval=30;
                }
                else
                {
                        last_view_ch2 = view_ch2;
                        last_view_ch3 = view_ch3;
                      view_ch2 = parameter_yaw*fram.yaw;
                        view_ch3 = parameter_pitch*fram.pitch;
                        view_ch2_interval=view_ch2-last_view_ch2;
                        view_ch3_interval=view_ch3-last_view_ch3;
                      time_interval=last_time_tick_1ms-last_last_time_tick_1ms;
                      if(time_interval<0)
                            time_interval=time_interval+10000;
            if(FirstFlag<=100)
                            FirstFlag++;
                }

                      time_count=0;
            }
         
             if(time_count<60)
        {
            last_step_ch2=step_ch2;
            last_step_ch3=step_ch3;
            
            step_ch2=    view_ch2 +view_ch2_interval/time_interval*time_count;//线性插值预测
            step_ch3=    view_ch3 +view_ch3_interval/time_interval*time_count;//线性插值预测
            
            visual_yaw_pid_out=visual_yaw_kp*step_ch2+visual_yaw_kd*(step_ch2-last_step_ch2);//PD控制
            visual_pitch_pid_out=visual_pitch_kp*step_ch3+visual_pitch_kd*(step_ch3-last_step_ch3);//PD控制
            
                #ifdef gongkong
                    if(FirstFlag<=20)
                    {
                        Gimbal_control.angle_yaw_set    -= fabs(visual_yaw_pid_out)< 3.0 ? visual_yaw_pid_out :(visual_yaw_pid_out>0 ? 3.0:-3.0);
                        Gimbal_control.angle_pitch_set  += fabs(visual_pitch_pid_out)<1.75? visual_pitch_pid_out :(visual_pitch_pid_out>0 ? 1.75 : -1.75);
                    }
                    else
                    {
                        Gimbal_control.angle_yaw_set    -= fabs(visual_yaw_pid_out)< 1.0? visual_yaw_pid_out :(visual_yaw_pid_out>0 ? 1.0:-1.0);
                        Gimbal_control.angle_pitch_set  += fabs(visual_pitch_pid_out)<5.0? visual_pitch_pid_out :(visual_pitch_pid_out>0 ? 5.0 : -5.0);
                    }
                #endif
                        
                #ifdef miaosuan
                    
                if(auto_aim_flag == 0xFF && small_power_flag == 0x00)
                {
                    if(FirstFlag<=20)
                    {
                        Gimbal_control.angle_yaw_set    -= fabs(visual_yaw_pid_out)< 1.0 ? visual_yaw_pid_out :(visual_yaw_pid_out>0 ? 1.0:-1.0);
                        Gimbal_control.angle_pitch_set  += fabs(visual_pitch_pid_out)<1.75? visual_pitch_pid_out :(visual_pitch_pid_out>0 ? 1.75 : -1.75);
                    }
                    else
                    {
                        Gimbal_control.angle_yaw_set    -= fabs(visual_yaw_pid_out)< 1.0? visual_yaw_pid_out :(visual_yaw_pid_out>0 ? 1.0:-1.0);
                        Gimbal_control.angle_pitch_set  += fabs(visual_pitch_pid_out)<5.0? visual_pitch_pid_out :(visual_pitch_pid_out>0 ? 5.0 : -5.0);
                    }
                }else if(auto_aim_flag == 0x00 && small_power_flag == 0xFF){
                    if(FirstFlag<=20)
                    {
                        Gimbal_control.angle_yaw_set    -= fabs(visual_yaw_pid_out)< 3.0 ? visual_yaw_pid_out :(visual_yaw_pid_out>0 ? 3.0:-3.0);
                        Gimbal_control.angle_pitch_set  += fabs(visual_pitch_pid_out)<1.75? visual_pitch_pid_out :(visual_pitch_pid_out>0 ? 1.75 : -1.75);
                    }
                    else
                    {
                        Gimbal_control.angle_yaw_set    -= fabs(visual_yaw_pid_out)< 1.0? visual_yaw_pid_out :(visual_yaw_pid_out>0 ? 1.0:-1.0);
                        Gimbal_control.angle_pitch_set  += fabs(visual_pitch_pid_out)<5.0? visual_pitch_pid_out :(visual_pitch_pid_out>0 ? 5.0 : -5.0);
                    }
                    
                }
                    
                #endif

        }
            time_count++;
        
         
        if(time_count>100)
        {
             FirstFlag=1;
        }
        
             if_newframe = 0;
         
        }
     else if(auto_aim_flag == 0x00 &&small_power_flag == 0xFF)
     { 
         if(if_newframe == 1)
         {
            Gimbal_control.angle_yaw_set -= fabs(fram.yaw)< 16.0?  fram.yaw :( fram.yaw>0 ? 16.0:-16.0);
            Gimbal_control.angle_pitch_set  += fabs(fram.pitch)<7.0? fram.pitch :(fram.pitch>0 ? 7.0 : -7.0);
             
             if_newframe = 0;
            
             if_rotate_ok = 1;
         }     
     }
    
    #else
    if(unpackFrame(rx_buf,len,&fram) == 0)  //解包成功
    {    
        if(fram.timestamp != frame_ex.timestamp)//如果前一帧数据和当前帧时间戳一样,目标丢失,不作为
        {
            if_newframe = 1;            
        }
        frame_ex.timestamp = fram.timestamp;
    }
//    if(auto_aim_flag == 0xFF && small_power_flag == 0x00)
//    {
//        if(if_newframe == 1)
//        {
//            last_last_time_tick_1ms = last_time_tick_1ms;;
//            last_time_tick_1ms = TASK_GetMicrosecond();
//            if(FirstFlag==1)
//            {
//                last_view_ch2 = parameter_yaw*fram.yaw;
//                last_view_ch3 = parameter_pitch*fram.pitch;
//                view_ch2 = last_view_ch2;
//                view_ch3 = last_view_ch3;
//                view_ch2_interval=0;
//                view_ch3_interval=0;
//                time_interval=30;
//            }
//            else
//            {
//                last_view_ch2 = view_ch2;
//                last_view_ch3 = view_ch3;
//                view_ch2 = parameter_yaw*fram.yaw;
//                view_ch3 = parameter_pitch*fram.pitch;
//                view_ch2_interval=view_ch2-last_view_ch2;
//                view_ch3_interval=view_ch3-last_view_ch3;
//                time_interval=last_time_tick_1ms-last_last_time_tick_1ms;
//                if(time_interval<0)
//                    time_interval=time_interval+10000;
//                if(FirstFlag<=100)
//                    FirstFlag++;
//            }

//            time_count=0;
//        }
//        if(time_count<60)
//        {
//            last_step_ch2=step_ch2;
//            last_step_ch3=step_ch3;
//            
//            step_ch2=    view_ch2 +view_ch2_interval/time_interval*time_count;//线性插值预测
//            step_ch3=    view_ch3 +view_ch3_interval/time_interval*time_count;//线性插值预测
//            
//            visual_yaw_pid_out=visual_yaw_kp*step_ch2+visual_yaw_kd*(step_ch2-last_step_ch2);//PD控制
//            visual_pitch_pid_out=visual_pitch_kp*step_ch3+visual_pitch_kd*(step_ch3-last_step_ch3);//PD控制
//            
//                #ifdef gongkong
//                    if(FirstFlag<=20)
//                    {
//                        Gimbal_control.angle_yaw_set    -= fabs(visual_yaw_pid_out)< 3.0 ? visual_yaw_pid_out :(visual_yaw_pid_out>0 ? 3.0:-3.0);
//                        Gimbal_control.angle_pitch_set  += fabs(visual_pitch_pid_out)<1.75? visual_pitch_pid_out :(visual_pitch_pid_out>0 ? 1.75 : -1.75);
//                    }
//                    else
//                    {
//                        Gimbal_control.angle_yaw_set    -= fabs(visual_yaw_pid_out)< 1.0? visual_yaw_pid_out :(visual_yaw_pid_out>0 ? 1.0:-1.0);
//                        Gimbal_control.angle_pitch_set  += fabs(visual_pitch_pid_out)<5.0? visual_pitch_pid_out :(visual_pitch_pid_out>0 ? 5.0 : -5.0);
//                    }
//                #endif
//                        
//                #ifdef miaosuan
//                    
//                if(auto_aim_flag == 0xFF && small_power_flag == 0x00)
//                {
//                    if(FirstFlag<=20)
//                    {
//                        Gimbal_control.angle_yaw_set    -= fabs(visual_yaw_pid_out)< 1.0 ? visual_yaw_pid_out :(visual_yaw_pid_out>0 ? 1.0:-1.0);
//                        Gimbal_control.angle_pitch_set  += fabs(visual_pitch_pid_out)<1.75? visual_pitch_pid_out :(visual_pitch_pid_out>0 ? 1.75 : -1.75);
//                    }
//                    else
//                    {
//                        Gimbal_control.angle_yaw_set    -= fabs(visual_yaw_pid_out)< 1.0? visual_yaw_pid_out :(visual_yaw_pid_out>0 ? 1.0:-1.0);
//                        Gimbal_control.angle_pitch_set  += fabs(visual_pitch_pid_out)<5.0? visual_pitch_pid_out :(visual_pitch_pid_out>0 ? 5.0 : -5.0);
//                    }
//                }else if(auto_aim_flag == 0x00 && small_power_flag == 0xFF){
//                    if(FirstFlag<=20)
//                    {
//                        Gimbal_control.angle_yaw_set    -= fabs(visual_yaw_pid_out)< 3.0 ? visual_yaw_pid_out :(visual_yaw_pid_out>0 ? 3.0:-3.0);
//                        Gimbal_control.angle_pitch_set  += fabs(visual_pitch_pid_out)<1.75? visual_pitch_pid_out :(visual_pitch_pid_out>0 ? 1.75 : -1.75);
//                    }
//                    else
//                    {
//                        Gimbal_control.angle_yaw_set    -= fabs(visual_yaw_pid_out)< 1.0? visual_yaw_pid_out :(visual_yaw_pid_out>0 ? 1.0:-1.0);
//                        Gimbal_control.angle_pitch_set  += fabs(visual_pitch_pid_out)<5.0? visual_pitch_pid_out :(visual_pitch_pid_out>0 ? 5.0 : -5.0);
//                    }
//                    
//                }
//                    
//                #endif
//        }
//        time_count++;
//        
//        if(time_count>100)
//        {
//             FirstFlag=1;
//        }
//        
//         if_newframe = 0;
//    }
//    else 
    if(auto_aim_flag == 0x00 && small_power_flag == 0xFF)
    { 
        if(if_newframe == 1) {
            spGIMBAL_Controller.user.update_target((fabs(fram.pitch)<7.0? fram.pitch :(fram.pitch>0 ? 7.0 : -7.0))/0.04394f,
                -(fabs(fram.yaw)< 16.0?  fram.yaw :( fram.yaw>0 ? 16.0:-16.0))/0.04394f); 
        }
    }
    #endif
}


static uint8_t __view_buffer[128];
UsartBuffer_t view_buffer = {
    .buffer = __view_buffer,
    .size = 128,
    .curr_ptr = 0,
    .last_ptr = 0
};
void Autoaim_USART_Interface(void) {
    uint16_t size = view_buffer.size - spDMA_USART2_rx_stream->NDTR;
    DMA_Restart(spDMA_USART2_rx_stream, (uint32_t)view_buffer.buffer, 
         (uint32_t)&USART2->DR, view_buffer.size);
    Auto_aim(view_buffer.buffer, size);
}

//        uint16_t size;
//        uint8_t buffer[128];
//        view_buffer.curr_ptr = view_buffer.size - spDMA_USART2_rx_stream->NDTR;
//        if(view_buffer.curr_ptr > view_buffer.last_ptr) {
//            size = view_buffer.curr_ptr - view_buffer.last_ptr;
//            DMA_CopyMem2Mem(
//                (uint32_t)buffer, 
//                (uint32_t)(&view_buffer.buffer[view_buffer.last_ptr]), 
//                size);
//        } else if(view_buffer.curr_ptr < view_buffer.last_ptr) {
//            size = view_buffer.size - view_buffer.last_ptr;
//            DMA_CopyMem2Mem(
//                (uint32_t)buffer, 
//                (uint32_t)(&view_buffer.buffer[view_buffer.last_ptr]), 
//                size);
//            DMA_CopyMem2Mem(
//                (uint32_t)(&buffer[size]), 
//                (uint32_t)(view_buffer.buffer), 
//                view_buffer.curr_ptr);
//            size += view_buffer.curr_ptr;
//        }
//        view_buffer.last_ptr = view_buffer.curr_ptr;

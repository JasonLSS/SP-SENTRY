/******************
裁判信息解读与透传数据帧封装程序
update: 2017.5.7
    视情况调用最后三个函数
    全局变量说明见uart2referee.h
    支持上传3个float数据
******************/

#include "sys.h"
#include "stdio.h"  // define NULL
#include "stdbool.h" 
#include "sp_conf.h"


extern uint8_t referee_buffer[128];

void float2bytes(float chosen_value, uint8_t * res_message);
float _bytes2float(uint8_t * chosen_Message);
void float2bytes(float chosen_value, uint8_t * res_message);
// flaot和字节互转
typedef union {
    float f;
    unsigned char b[4];
} Bytes2Float;

// float和uint32_t互转
typedef union {
    uint32_t uint32_t_value;
    unsigned char b[4];
} Bytes2U32;



// 位置信息(被其他结构体调用), 定位数据单位为米，小数点后两位为有效数据。
typedef packed_struct {
    uint8_t flag;
    float x;
    float y;
    float z;
    float compass;
}tLocData;

/*
 比赛状态数据（0x0001）, 发送频率1Hz。

*/

typedef packed_struct
{
uint8_t game_type : 4;
uint8_t game_progress : 4;
uint16_t stage_remain_time;
} ext_game_state_t;

/*
比赛结果数据：0x0002 
发送频率：比赛结束后发送
大小：1字节
说明：0：平局，1：红方胜利，2：蓝方胜利
*/
typedef packed_struct
{
	uint8_t winner;
} ext_game_result_t;


// 机器人存存活数据（0x0003）
typedef packed_struct
{
uint16_t robot_legion;
} ext_game_robot_survivors_t;


// //场地时事件数据（0x0101）
typedef packed_struct
{
uint32_t event_type;
} ext_event_data_t;

//补给站动作标识（0x0102）
typedef packed_struct
{
uint8_t supply_projectile_id;
uint8_t supply_robot_id;
uint8_t supply_projectile_step;
} ext_supply_projectile_action_t;



//补给站预约子弹（0x0103）
typedef packed_struct
{
uint8_t supply_projectile_id;
uint8_t supply_num;
} ext_supply_projectile_booking_t;

//比赛机器人状态(0x0201)
typedef packed_struct
{
uint8_t robot_id;
uint8_t robot_level;
uint16_t remain_HP;
uint16_t max_HP;
uint16_t shooter_heat0_cooling_rate;
uint16_t shooter_heat0_cooling_limit;
uint16_t shooter_heat1_cooling_rate;
uint16_t shooter_heat1_cooling_limit;
uint8_t mains_power_gimbal_output : 1;
uint8_t mains_power_chassis_output : 1;
uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state_t;

//实时功率热量数据（0x0202）
typedef packed_struct
{
uint16_t chassis_volt;
uint16_t chassis_current;
float chassis_power;
uint16_t chassis_power_buffer;
uint16_t shooter_heat0;
uint16_t shooter_heat1;
} ext_power_heat_data_t;

//机器人位置（0x0203）
typedef packed_struct
{
float x;
float y;
float z;
float yaw;
} ext_game_robot_pos_t;

//机器人增益（0x0204）
typedef packed_struct
{
uint8_t power_rune_buff;
}ext_buff_musk_t;

//空中机器人能量状态（0x0205）
typedef packed_struct
{
uint8_t energy_point;
uint8_t attack_time;
} aerial_robot_energy_t;

//伤害状态（0x0206）
typedef packed_struct
{
uint8_t armor_id : 4;
uint8_t hurt_type : 4;
} ext_robot_hurt_t;


//实时射击信息（0x0207）
typedef packed_struct
{
uint8_t bullet_type;
uint8_t bullet_freq;
float bullet_speed;
} ext_shoot_data_t;


//交互数据接收信息（0x0301）
typedef packed_struct
{
uint16_t data_cmd_id;
uint16_t send_ID;
uint16_t receiver_ID;
}ext_student_interactive_header_data_t;

////////////////////////////////////////////
//客户端自定义数据（0x0301），内容ID：data_cmd(0xD180)
typedef packed_struct
{
float data1;
float data2;
float data3;
uint8_t masks;
} client_custom_data_t;

//typedef packed_struct
//{
//uint8_t data[];
//} robot_interactive_data_t;


// 全局裁判信息字段定义
extern ext_game_state_t ext_game_state;// 比赛进程信息（0x0001）
extern ext_game_state_t                           ext_game_state;// 比赛状态数据（0x0001）
extern ext_game_result_t                          ext_game_result;//比赛结果数据(0x0002)
extern ext_game_robot_survivors_t                 ext_game_robot_survivors;//机器人存存活数据（0x0003）
extern ext_event_data_t                           ext_event_data;//场地时事件数据（0x0101）
extern ext_supply_projectile_action_t             ext_supply_projectile_action;//补给站动作标识（0x0102）
extern ext_supply_projectile_booking_t            ext_supply_projectile_booking;//补给站预约子弹（0x0103）
extern ext_game_robot_state_t                     ext_game_robot_state;//比赛机器人状态(0x0201)
extern ext_power_heat_data_t                      ext_power_heat_data;////实时功率热量数据（0x0202）
extern ext_game_robot_pos_t                       ext_game_robot_pos;//机器人位置（0x0203）
extern ext_buff_musk_t                            ext_buff_musk;//机器人增益（0x0204）
extern aerial_robot_energy_t                      aerial_robot_energy;//空中机器人能量状态（0x0205）
extern ext_robot_hurt_t                           ext_robot_hurt;//伤害状态（0x0206）
extern ext_shoot_data_t                           ext_shoot_data;//实时射击信息（0x0207）

extern ext_student_interactive_header_data_t      ext_student_interactive_header_data;//交互数据接收信息（0x0301）

// 使用前的初始化所有裁判信息相关结构体, 可自行添加定义初值
// 可不用，系统好像默认初值是0？？   有的好像显示不出来啊


// 使用完整数据帧立刻更新全部裁判信息相关结构体。(带校验)
uint8_t frame_interpret(uint8_t * frame);

// 读入单字节来更新全部裁判信息相关结构体, 
// 即仅累积字节为一完整数据包时 才调用frame_interpret函数 来更新相关结构体。
void referee_info_update(uint8_t single_byte);
// 自定义数据帧, 封装入数组头指针custom_frame，长度 = 5+2+12+2 = 21
// 调用前请确保全局变量MyData结构体已更新值, 
// 发送示例:
// for(i=0;i<21;i++) {
//     USART_SendData(USART2, custom_frame_test[i]);
//     while(USART_GetFlagStatus(USART2, USART_FLAG_TC) != SET);
// }


// debug用的全局变量
extern uint8_t referee_message[64];  // 完整数据帧存放, 理论44就够。
extern uint8_t cmdID;;
extern uint8_t blood_counter;  // (debug)被打计数

// 以下暂时不用
//// 校验数据帧, CRC8和CRC16
//uint8_t Verify_frame(uint8_t * frame);

extern void update_from_dma(void);
extern void update_from_dma2(void);
extern uint8_t seq_real;
extern uint8_t usart6_dma_flag;
extern int shoot_counter_referee;
void init_referee_info(void);

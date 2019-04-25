/******************
������Ϣ�����͸������֡��װ����
update: 2017.5.7
    ��������������������
    ȫ�ֱ���˵����uart2referee.h
    ֧���ϴ�3��float����
******************/

#include "sys.h"
#include "stdio.h"  // define NULL
#include "stdbool.h" 
#include "sp_conf.h"


extern uint8_t referee_buffer[128];

void float2bytes(float chosen_value, uint8_t * res_message);
float _bytes2float(uint8_t * chosen_Message);
void float2bytes(float chosen_value, uint8_t * res_message);
// flaot���ֽڻ�ת
typedef union {
    float f;
    unsigned char b[4];
} Bytes2Float;

// float��uint32_t��ת
typedef union {
    uint32_t uint32_t_value;
    unsigned char b[4];
} Bytes2U32;



// λ����Ϣ(�������ṹ�����), ��λ���ݵ�λΪ�ף�С�������λΪ��Ч���ݡ�
typedef packed_struct {
    uint8_t flag;
    float x;
    float y;
    float z;
    float compass;
}tLocData;

/*
 ����״̬���ݣ�0x0001��, ����Ƶ��1Hz��

*/

typedef packed_struct
{
uint8_t game_type : 4;
uint8_t game_progress : 4;
uint16_t stage_remain_time;
} ext_game_state_t;

/*
����������ݣ�0x0002 
����Ƶ�ʣ�������������
��С��1�ֽ�
˵����0��ƽ�֣�1���췽ʤ����2������ʤ��
*/
typedef packed_struct
{
	uint8_t winner;
} ext_game_result_t;


// �����˴������ݣ�0x0003��
typedef packed_struct
{
uint16_t robot_legion;
} ext_game_robot_survivors_t;


// //����ʱ�¼����ݣ�0x0101��
typedef packed_struct
{
uint32_t event_type;
} ext_event_data_t;

//����վ������ʶ��0x0102��
typedef packed_struct
{
uint8_t supply_projectile_id;
uint8_t supply_robot_id;
uint8_t supply_projectile_step;
} ext_supply_projectile_action_t;



//����վԤԼ�ӵ���0x0103��
typedef packed_struct
{
uint8_t supply_projectile_id;
uint8_t supply_num;
} ext_supply_projectile_booking_t;

//����������״̬(0x0201)
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

//ʵʱ�����������ݣ�0x0202��
typedef packed_struct
{
uint16_t chassis_volt;
uint16_t chassis_current;
float chassis_power;
uint16_t chassis_power_buffer;
uint16_t shooter_heat0;
uint16_t shooter_heat1;
} ext_power_heat_data_t;

//������λ�ã�0x0203��
typedef packed_struct
{
float x;
float y;
float z;
float yaw;
} ext_game_robot_pos_t;

//���������棨0x0204��
typedef packed_struct
{
uint8_t power_rune_buff;
}ext_buff_musk_t;

//���л���������״̬��0x0205��
typedef packed_struct
{
uint8_t energy_point;
uint8_t attack_time;
} aerial_robot_energy_t;

//�˺�״̬��0x0206��
typedef packed_struct
{
uint8_t armor_id : 4;
uint8_t hurt_type : 4;
} ext_robot_hurt_t;


//ʵʱ�����Ϣ��0x0207��
typedef packed_struct
{
uint8_t bullet_type;
uint8_t bullet_freq;
float bullet_speed;
} ext_shoot_data_t;


//�������ݽ�����Ϣ��0x0301��
typedef packed_struct
{
uint16_t data_cmd_id;
uint16_t send_ID;
uint16_t receiver_ID;
}ext_student_interactive_header_data_t;

////////////////////////////////////////////
//�ͻ����Զ������ݣ�0x0301��������ID��data_cmd(0xD180)
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


// ȫ�ֲ�����Ϣ�ֶζ���
extern ext_game_state_t ext_game_state;// ����������Ϣ��0x0001��
extern ext_game_state_t                           ext_game_state;// ����״̬���ݣ�0x0001��
extern ext_game_result_t                          ext_game_result;//�����������(0x0002)
extern ext_game_robot_survivors_t                 ext_game_robot_survivors;//�����˴������ݣ�0x0003��
extern ext_event_data_t                           ext_event_data;//����ʱ�¼����ݣ�0x0101��
extern ext_supply_projectile_action_t             ext_supply_projectile_action;//����վ������ʶ��0x0102��
extern ext_supply_projectile_booking_t            ext_supply_projectile_booking;//����վԤԼ�ӵ���0x0103��
extern ext_game_robot_state_t                     ext_game_robot_state;//����������״̬(0x0201)
extern ext_power_heat_data_t                      ext_power_heat_data;////ʵʱ�����������ݣ�0x0202��
extern ext_game_robot_pos_t                       ext_game_robot_pos;//������λ�ã�0x0203��
extern ext_buff_musk_t                            ext_buff_musk;//���������棨0x0204��
extern aerial_robot_energy_t                      aerial_robot_energy;//���л���������״̬��0x0205��
extern ext_robot_hurt_t                           ext_robot_hurt;//�˺�״̬��0x0206��
extern ext_shoot_data_t                           ext_shoot_data;//ʵʱ�����Ϣ��0x0207��

extern ext_student_interactive_header_data_t      ext_student_interactive_header_data;//�������ݽ�����Ϣ��0x0301��

// ʹ��ǰ�ĳ�ʼ�����в�����Ϣ��ؽṹ��, ��������Ӷ����ֵ
// �ɲ��ã�ϵͳ����Ĭ�ϳ�ֵ��0����   �еĺ�����ʾ��������


// ʹ����������֡���̸���ȫ��������Ϣ��ؽṹ�塣(��У��)
uint8_t frame_interpret(uint8_t * frame);

// ���뵥�ֽ�������ȫ��������Ϣ��ؽṹ��, 
// �����ۻ��ֽ�Ϊһ�������ݰ�ʱ �ŵ���frame_interpret���� ��������ؽṹ�塣
void referee_info_update(uint8_t single_byte);
// �Զ�������֡, ��װ������ͷָ��custom_frame������ = 5+2+12+2 = 21
// ����ǰ��ȷ��ȫ�ֱ���MyData�ṹ���Ѹ���ֵ, 
// ����ʾ��:
// for(i=0;i<21;i++) {
//     USART_SendData(USART2, custom_frame_test[i]);
//     while(USART_GetFlagStatus(USART2, USART_FLAG_TC) != SET);
// }


// debug�õ�ȫ�ֱ���
extern uint8_t referee_message[64];  // ��������֡���, ����44�͹���
extern uint8_t cmdID;;
extern uint8_t blood_counter;  // (debug)�������

// ������ʱ����
//// У������֡, CRC8��CRC16
//uint8_t Verify_frame(uint8_t * frame);

extern void update_from_dma(void);
extern void update_from_dma2(void);
extern uint8_t seq_real;
extern uint8_t usart6_dma_flag;
extern int shoot_counter_referee;
void init_referee_info(void);

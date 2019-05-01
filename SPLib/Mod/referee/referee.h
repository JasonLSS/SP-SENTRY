#ifndef __REFEREE_H
#define __REFEREE_H

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
#include "referee_crc.h"

#define MY_ROBOT_ID robotid_red_hero

/*--------------------------------------------------------------------------------*
 *                                Referee Constants                               *
 *--------------------------------------------------------------------------------*/
#ifndef MY_ROBOT_ID
    #error "Please define define your MY_ROBOT_ID, like #define MY_ROBOT_ID robotid_red_hero."
#endif
 
#define REFEREE_FRAME_HEADER_SOF                ((uint8_t)(0xA5))

#define REFEREE_STUDENT_ROBOT_MAX               ((uint16_t)(0x0200))
#define REFEREE_STUDENT_ROBOT_MIN               ((uint16_t)(0x02FF))

#define REFEREE_STUDENT_CLIENT_SOF              ((uint16_t)(0xD180))


/*--------------------------------------------------------------------------------*
 *                                   Referee Data                                 *
 *--------------------------------------------------------------------------------*/

/** 
  * @brief  Frame Headr
  */
typedef packed_struct {
    uint8_t     sof;                    /*!< Fixed value 0xA5 */
    uint16_t    data_length;            /*!< Length of next data pack */
    uint8_t     seq;                    /*!< Pack sequene id */
    uint8_t     crc8;                   /*!< CRC checksum for frame header pack */
} ext_frame_header_t;


/** 
  * @brief  Command ID Map
  */
typedef enum {
    game_state                  = 0x0001,     /*!< frequency = 1Hz */
    game_result                 = 0x0002,     /*!< send at game ending */
    game_robot_survivors        = 0x0003,     /*!< frequency = 1Hz */
    event_data                  = 0x0101,     /*!< send at event changing */
    supply_projectile_action    = 0x0102,     /*!< send at action */
    supply_projectile_booking   = 0x0103,     /*!< send by user, max frequency = 10Hz */
    game_robot_state            = 0x0201,     /*!< frequency = 10Hz */
    power_heat_data             = 0x0202,     /*!< frequency = 50Hz */
    game_robot_pos              = 0x0203,     /*!< frequency = 10Hz */
    buff_musk                   = 0x0204,     /*!< send at changing */
    aerial_robot_energy         = 0x0205,     /*!< frequency = 10Hz, only for aerial robot */
    robot_hurt                  = 0x0206,     /*!< send at hurting */
    shoot_data                  = 0x0207,     /*!< send at shooting */
    robot_interactive_data      = 0x0301,     /*!< send by user, max frequency = 10Hz */
} ext_cmd_id_t;


/** 
  * @brief  Game State Pack
  */
typedef enum {
    gametype_robomaster = 1,
    gametype_tournament = 2,
    gametype_cira = 3,
} ext_game_state_game_type_t;
typedef enum {
    gameprogress_unstarted = 0,
    gameprogress_preparing = 1,
    gameprogress_self_checking = 2,
    gameprogress_countdown = 3,
    gameprogress_gaming = 4,
    gameprogress_judging = 5,
} ext_game_state_game_progress_t;
typedef packed_struct {
    uint8_t     game_type : 4;          /*!< @ref ext_game_state_game_type_t */
    uint8_t     game_progress : 4;      /*!< @ref ext_game_state_game_progress_t */
    uint16_t    stage_remain_time;      /*!< remaining time at seconds */
} ext_game_state_t;


/** 
  * @brief  Game Result Pack
  */
typedef enum {
    winner_draw = 0,
    winner_red = 1,
    winner_blue = 2,
} ext_game_result_winner_t;
typedef packed_struct {
    uint8_t     winner;                 /*!< @ref ext_game_result_winner_t */
} ext_game_result_t;


/** 
  * @brief Robot Survivors Pack
  * @note  0=died/absent, 1=alive
  */
typedef packed_struct {
    uint16_t    red_hero: 1;
    uint16_t    red_engineer: 1;
    uint16_t    red_infantry_1: 1;
    uint16_t    red_infantry_2: 1;
    uint16_t    red_infantry_3: 1;
    uint16_t    red_aerial: 1;
    uint16_t    red_sentry: 1;
    uint16_t    : 1;
    
    uint16_t    blue_hero: 1;
    uint16_t    blue_engineer: 1;
    uint16_t    blue_infantry_1: 1;
    uint16_t    blue_infantry_2: 1;
    uint16_t    blue_infantry_3: 1;
    uint16_t    blue_aerial: 1;
    uint16_t    blue_sentry: 1;
    uint16_t    : 1;
} ext_game_robot_survivors_t;


// //场地时事件数据（0x0101）
typedef packed_struct {
    uint32_t    event_type;
} ext_event_data_t;


//补给站动作标识（0x0102）
typedef packed_struct {
    uint8_t     supply_projectile_id;
    uint8_t     supply_robot_id;
    uint8_t     supply_projectile_step;
    uint8_t     supply_projectile_num;
} ext_supply_projectile_action_t;


//补给站预约子弹（0x0103）
typedef packed_struct {
    uint8_t     supply_projectile_id;
    uint8_t     supply_robot_id;
    uint8_t     supply_num;
} ext_supply_projectile_booking_t;


//比赛机器人状态(0x0201)
typedef packed_struct {
    uint8_t     robot_id;
    uint8_t     robot_level;
    uint16_t    remain_HP;
    uint16_t    max_HP;
    uint16_t    shooter_heat0_cooling_rate;
    uint16_t    shooter_heat0_cooling_limit;
    uint16_t    shooter_heat1_cooling_rate;
    uint16_t    shooter_heat1_cooling_limit;
    uint8_t     mains_power_gimbal_output : 1;
    uint8_t     mains_power_chassis_output : 1;
    uint8_t     mains_power_shooter_output : 1;
} ext_game_robot_state_t;


//实时功率热量数据（0x0202）
typedef packed_struct {
    uint16_t    chassis_volt;
    uint16_t    chassis_current;
    float       chassis_power;
    uint16_t    chassis_power_buffer;
    uint16_t    shooter_heat0;
    uint16_t    shooter_heat1;
} ext_power_heat_data_t;


//机器人位置（0x0203）
typedef packed_struct {
    float       x;
    float       y;
    float       z;
    float       yaw;
} ext_game_robot_pos_t;


//机器人增益（0x0204）
typedef packed_struct {
    uint8_t     power_rune_buff;
}ext_buff_musk_t;


//空中机器人能量状态（0x0205）
typedef packed_struct {
    uint8_t     energy_point;
    uint8_t     attack_time;
} ext_aerial_robot_energy_t;


//伤害状态（0x0206）
typedef packed_struct {
    uint8_t     armor_id : 4;
    uint8_t     hurt_type : 4;
} ext_robot_hurt_t;


//实时射击信息（0x0207）
typedef packed_struct {
    uint8_t     bullet_type;
    uint8_t     bullet_freq;
    float       bullet_speed;
} ext_shoot_data_t;



/*--------------------------------------------------------------------------------*
 *                            Robot Exchange Data                                 *
 *--------------------------------------------------------------------------------*/
typedef enum {
    robotid_red_hero = 1,
    robotid_red_engineer = 2,
    robotid_red_infantry_1 = 3,
    robotid_red_infantry_2 = 4,
    robotid_red_infantry_3 = 5,
    robotid_red_aerial = 6,
    robotid_red_sentry = 7,
    robotid_blue_hero = 11,
    robotid_blue_engineer = 12,
    robotid_blue_infantry_1 = 13,
    robotid_blue_infantry_2 = 14,
    robotid_blue_infantry_3 = 15,
    robotid_blue_aerial = 16,
    robotid_blue_sentry = 17,

    clientid_red_hero = 0x0101,
    clientid_red_engineer = 0x0102,
    clientid_red_infantry_1 = 0x0103,
    clientid_red_infantry_2 = 0x0104,
    clientid_red_infantry_3 = 0x0105,
    clientid_red_aerial = 0x0106,
    clientid_blue_hero = 0x0111,
    clientid_blue_engineer = 0x0112,
    clientid_blue_infantry_1 = 0x0113,
    clientid_blue_infantry_2 = 0x0114,
    clientid_blue_infantry_3 = 0x0115,
    clientid_blue_aerial = 0x0116,
} ext_id_t;

//客户端自定义数据（0x0301）, 内容ID：data_cmd(0xD180)
typedef enum {
    clientled_red = 0,
    clientled_green = 1,
} ext_client_custom_data_led_t;

typedef packed_union {
    uint8_t     masks;
    packed_struct {
        uint8_t     led1 : 1;
        uint8_t     led2 : 1;
        uint8_t     led3 : 1;
        uint8_t     led4 : 1;
        uint8_t     led5 : 1;
        uint8_t     led6 : 1;
        uint8_t     : 2;
    } masks_bits;
} ext_client_custom_data_mask_t;
typedef packed_struct {
    ext_frame_header_t              header;
    uint16_t                        cmd_id;
    
    uint16_t                        data_id;            /*!< fixed value 0xD180 */
    uint16_t                        sender_id;
    uint16_t                        client_id;
    float                           data[3];
    ext_client_custom_data_mask_t   masks;
    
    uint16_t                        crc16;
} ext_client_custom_data_t;

//机器人交互数据(0x0301), 内容ID：data_cmd(0x0200~0x02FF)
typedef packed_struct {
    uint16_t            data_id;            /*!< range 0x200~0x2FF */
    uint16_t            sender_id;
    uint16_t            robot_id;
    uint8_t             data[113];          /*!< max data length = 13byte */
} ext_robot_interactive_data_t;


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
extern ext_aerial_robot_energy_t                  ext_aerial_robot_energy;//空中机器人能量状态（0x0205）
extern ext_robot_hurt_t                           ext_robot_hurt;//伤害状态（0x0206）
extern ext_shoot_data_t                           ext_shoot_data;//实时射击信息（0x0207）
extern ext_robot_interactive_data_t               ext_robot_interactive_data;


void referee_init(void);
bool referee_send_robot(uint16_t data_id, ext_id_t target_id, uint8_t *data, uint8_t size);
bool referee_send_client(ext_id_t target_id, float data[3], ext_client_custom_data_mask_t masks);

#endif // __REFEREE_H

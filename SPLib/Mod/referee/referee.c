/******************
裁判信息解读与透传数据帧封装程序
update: 2017.5.7
        视情况调用最后三个函数
        全局变量说明见RefereeInfo.h
        支持上传3个float数据
******************/

#include "referee.h"
#include "referee_crc.h"
#include "sp_utility.h"


ext_game_state_t                           ext_game_state;// 比赛状态数据（0x0001）
ext_game_result_t                          ext_game_result;//比赛结果数据(0x0002)
ext_game_robot_HP_t                        ext_game_robot_HP;//机器人存存活数据（0x0003）
ext_event_data_t                           ext_event_data;//场地时事件数据（0x0101）
ext_supply_projectile_action_t             ext_supply_projectile_action;//补给站动作标识（0x0102）
ext_supply_projectile_booking_t            ext_supply_projectile_booking;//补给站预约子弹（0x0103）
ext_referee_warning_t                      ext_referee_warning;//裁判警告信息（0x0104）
ext_game_robot_state_t                     ext_game_robot_state;//比赛机器人状态(0x0201)
ext_power_heat_data_t                      ext_power_heat_data;////实时功率热量数据（0x0202）
ext_game_robot_pos_t                       ext_game_robot_pos;//机器人位置（0x0203）
ext_buff_musk_t                            ext_buff_musk;//机器人增益（0x0204）
ext_aerial_robot_energy_t                  ext_aerial_robot_energy;//空中机器人能量状态（0x0205）
ext_robot_hurt_t                           ext_robot_hurt;//伤害状态（0x0206）
ext_shoot_data_t                           ext_shoot_data;//实时射击信息（0x0207）
ext_bullet_remaining_t                     ext_bullet_remaining;//子弹剩余发射数（0x0208）
ext_robot_interactive_data_t               ext_robot_interactive_data;


int count[8] = {0x00};
float attacked_time = 0;

// 比赛机器人状态（0x0001）, 发送频率为10Hz。
void ext_game_state_interpret(uint8_t * ext_game_state_Message) {
//    uint8_t *a;
//    memcpy(a,ext_game_state_Message,1);
//    ext_game_state.game_type=(*a)>>4;
//    ext_game_state.game_progress=(*a)&0x000f;
//    memcpy((uint8_t*)&ext_game_state.stage_remain_time,ext_game_state_Message+1,1);
    spDMA.mem2mem.copy((uint32_t)&ext_game_state, 
        (uint32_t)ext_game_state_Message, 
        sizeof(ext_game_state));
}

//比赛结果数据(0x0002)
void  ext_game_result_interpret(uint8_t * ext_game_result_t_Message) {
//    ext_game_result.winner=*ext_game_result_t_Message;
    spDMA.mem2mem.copy((uint32_t)&ext_game_result, 
        (uint32_t)ext_game_result_t_Message, 
        sizeof(ext_game_result));
}

//机器人存存活数据（0x0003）
void ext_game_robot_HP_interpret(uint8_t * ext_game_robot_HP_t_Message) {
//    memcpy((uint8_t*)&ext_game_robot_HP.robot_legion,ext_game_robot_HP_t_Message,2);
    spDMA.mem2mem.copy((uint32_t)&ext_game_robot_HP, 
        (uint32_t)ext_game_robot_HP_t_Message, 
        sizeof(ext_game_robot_HP));
}

//场地时事件数据（0x0101）...........这个得重写，差的太多
void ext_event_data_interpret(uint8_t * ext_event_data_t_Message) {
//    memcpy((uint8_t*)&ext_event_data.event_type,ext_event_data_t_Message,4);
    spDMA.mem2mem.copy((uint32_t)&ext_event_data, 
        (uint32_t)ext_event_data_t_Message,
        sizeof(ext_event_data));
}

//补给站动作标识（0x0102）
void ext_supply_projectile_action_interpret(uint8_t * ext_supply_projectile_action_Message) {
//    memcpy((uint8_t*)&ext_supply_projectile_action.supply_projectile_id,ext_supply_projectile_action_Message,1);
//    memcpy((uint8_t*)&ext_supply_projectile_action.supply_robot_id,ext_supply_projectile_action_Message+1,1);
//    memcpy((uint8_t*)&ext_supply_projectile_action.supply_projectile_step,ext_supply_projectile_action_Message+2,1);
    spDMA.mem2mem.copy((uint32_t)&ext_supply_projectile_action, 
        (uint32_t)ext_supply_projectile_action_Message,
        sizeof(ext_supply_projectile_action));
}

//补给站预约子弹（0x0103）
void ext_supply_projectile_booking_interpret(uint8_t * ext_supply_projectile_booking_Message) {
//    memcpy((uint8_t*)&ext_supply_projectile_booking.supply_projectile_id,ext_supply_projectile_booking_Message,1);
//    memcpy((uint8_t*)&ext_supply_projectile_booking.supply_num,ext_supply_projectile_booking_Message+1,1);
    spDMA.mem2mem.copy((uint32_t)&ext_supply_projectile_booking, 
        (uint32_t)ext_supply_projectile_booking_Message,
        sizeof(ext_supply_projectile_booking));
}

//比赛机器人状态(0x0201)
void ext_game_robot_state_interpret(uint8_t * ext_game_robot_state_Message) {
//    memcpy((uint8_t*)&ext_game_robot_state.robot_id,ext_game_robot_state_Message,1);
//    memcpy((uint8_t*)&ext_game_robot_state.robot_level,ext_game_robot_state_Message+1,1);
//    memcpy((uint8_t*)&ext_game_robot_state.remain_HP,ext_game_robot_state_Message+2,2);
//    memcpy((uint8_t*)&ext_game_robot_state.max_HP,ext_game_robot_state_Message+4,2);
//    memcpy((uint8_t*)&ext_game_robot_state.shooter_heat0_cooling_rate,ext_game_robot_state_Message+6,2);
//    memcpy((uint8_t*)&ext_game_robot_state.shooter_heat0_cooling_limit,ext_game_robot_state_Message+8,2);
//    memcpy((uint8_t*)&ext_game_robot_state.shooter_heat1_cooling_rate,ext_game_robot_state_Message+10,2);
//    memcpy((uint8_t*)&ext_game_robot_state.shooter_heat1_cooling_limit,ext_game_robot_state_Message+12,2);

//    uint8_t a;
//    memcpy(&a,ext_game_robot_state_Message+14,1);/////////////////

//    ext_game_robot_state.mains_power_gimbal_output=a>>7;
//    ext_game_robot_state.mains_power_chassis_output=(a>>6)&0x0001;
//    ext_game_robot_state.mains_power_shooter_output=(a>>5)&0x0001;
		
    if(spDMA.mem2mem.copy((uint32_t)&ext_game_robot_state, 
        (uint32_t)ext_game_robot_state_Message,
        sizeof(ext_game_robot_state))) {
					count[1] ++;
				}
}

///////实时功率热量数据（0x0202）
void ext_power_heat_data_interpret(uint8_t * ext_power_heat_data_Message) {
//    memcpy((uint8_t*)&ext_power_heat_data.chassis_volt,ext_power_heat_data_Message,2);
//    memcpy((uint8_t*)&ext_power_heat_data.chassis_current,ext_power_heat_data_Message+2,2);
//    memcpy((uint8_t*)&ext_power_heat_data.chassis_power,ext_power_heat_data_Message+4,4);
//    // float temp;
//    //ext_power_heat_data.chassis_power=_bytes2float(ext_power_heat_data_Message+4);
//    //ext_power_heat_data.chassis_power_buffer=_bytes2float(ext_power_heat_data_Message+8);
//    memcpy((uint8_t*)&ext_power_heat_data.chassis_power_buffer,(ext_power_heat_data_Message+8),2);
//    memcpy((uint8_t*)&ext_power_heat_data.shooter_heat0,ext_power_heat_data_Message+10,2);
//    memcpy((uint8_t*)&ext_power_heat_data.shooter_heat1,ext_power_heat_data_Message+12,2);
		
    if(spDMA.mem2mem.copy((uint32_t)&ext_power_heat_data, 
        (uint32_t)ext_power_heat_data_Message,
        sizeof(ext_power_heat_data))) {
						count[0] ++;
				}
}


////机器人位置（0x0203）
void ext_game_robot_pos_interpret(uint8_t * ext_game_robot_pos_Message) {
//    memcpy((uint8_t*)&ext_game_robot_pos.x,ext_game_robot_pos_Message,4);
//    memcpy((uint8_t*)&ext_game_robot_pos.y,ext_game_robot_pos_Message+4,4);
//    memcpy((uint8_t*)&ext_game_robot_pos.z,ext_game_robot_pos_Message+8,4);
//    memcpy((uint8_t*)&ext_game_robot_pos.yaw,ext_game_robot_pos_Message+12,4);
    spDMA.mem2mem.copy((uint32_t)&ext_game_robot_pos, 
        (uint32_t)ext_game_robot_pos_Message,
        sizeof(ext_game_robot_pos));
}

////机器人增益（0x0204）
void ext_buff_musk_interpret(uint8_t * ext_buff_musk_Message) {
//    memcpy((uint8_t*)&ext_buff_musk.power_rune_buff,ext_buff_musk_Message,1);
    spDMA.mem2mem.copy((uint32_t)&ext_buff_musk, 
        (uint32_t)ext_buff_musk_Message,
        sizeof(ext_buff_musk));
}

//空中机器人能量状态（0x0205）
void aerial_robot_energy_interpret(uint8_t * aerial_robot_energy_Message) {
//    memcpy((uint8_t*)&ext_aerial_robot_energy.energy_point,aerial_robot_energy_Message,1);
//    memcpy((uint8_t*)&ext_aerial_robot_energy.attack_time,aerial_robot_energy_Message+1,2);
    spDMA.mem2mem.copy((uint32_t)&ext_aerial_robot_energy, 
        (uint32_t)aerial_robot_energy_Message,
        sizeof(ext_aerial_robot_energy));
}

////伤害状态（0x0206）
void  ext_robot_hurt_interpret(uint8_t *  ext_robot_hurt_Message) {
//    uint8_t a;
//    memcpy((uint8_t*)&a,ext_robot_hurt_Message,1);
//    ext_robot_hurt.armor_id=a>>4;
//    ext_robot_hurt.hurt_type=a&0x000f;
    spDMA.mem2mem.copy((uint32_t)&ext_robot_hurt, 
        (uint32_t)ext_robot_hurt_Message,
        sizeof(ext_robot_hurt));
		if(ext_robot_hurt.hurt_type == 0x0){
				attacked_time = 200;
		}
}

////实时射击信息（0x0207）
void ext_shoot_data_interpret(uint8_t * ext_shoot_data_Message) {
//    memcpy((uint8_t*)&ext_shoot_data.bullet_type,ext_shoot_data_Message,1);
//    memcpy((uint8_t*)&ext_shoot_data.bullet_freq,ext_shoot_data_Message+1,1);
//    memcpy((uint8_t*)&ext_shoot_data.bullet_speed,ext_shoot_data_Message+2,4);
    spDMA.mem2mem.copy((uint32_t)&ext_shoot_data, 
        (uint32_t)ext_shoot_data_Message,
        sizeof(ext_shoot_data));
}

////交互数据接收信息（0x0301）
void ext_student_interactive_header_data_interpret(uint8_t * ext_student_interactive_header_data_Message) {
//    memcpy((uint8_t*)&ext_student_interactive_header_data.data_cmd_id,ext_student_interactive_header_data_Message,2);
//    memcpy((uint8_t*)&ext_student_interactive_header_data.send_ID,ext_student_interactive_header_data_Message+2,2);
//    memcpy((uint8_t*)&ext_student_interactive_header_data.receiver_ID,ext_student_interactive_header_data_Message+4,2);
    spDMA.mem2mem.copy((uint32_t)&ext_robot_interactive_data, 
        (uint32_t)ext_student_interactive_header_data_Message,
        sizeof(ext_robot_interactive_data));
}

void robot_interactive_data_interpret(uint8_t * robot_interactive_data_Message) {
//    memcpy(&robot_interactive_data.data,robot_interactive_data_Message,sizeof(data));
    spDMA.mem2mem.copy((uint32_t)&ext_robot_interactive_data, 
        (uint32_t)robot_interactive_data_Message,
        sizeof(ext_robot_interactive_data));
}


// 完整校验数据帧, CRC8和CRC16
bool vrerify_frame(uint8_t * frame) {
    /* frame length = header(5) + cmd_id(2) + data(n) + crc16(2) */
    uint16_t frame_length = 5 + 2 + _bytes2u16(&frame[1]) + 2;
    return ( Verify_CRC8_Check_Sum(frame, sizeof(ext_frame_header_t)) 
        && Verify_CRC16_Check_Sum(frame, frame_length) );
}

// 使用完整数据帧更新全部裁判信息相关结构体。(带校验)
void frame_interpret(uint8_t * _frame, uint16_t size) {
    for(uint16_t i=0; i<size; i++) {
        if(_frame[i]==REFEREE_FRAME_HEADER_SOF && vrerify_frame(&_frame[i])) {
//            static uint8_t seq_ex = 0;
//            uint8_t seq = frame[3];
//            if(frame[3]<=seq_ex && (seq_ex-frame[3])>=240) {
//                continue;
//            }
//            seq_ex = seq;
						uint8_t * frame = &_frame[i];
						uint16_t length = (uint16_t)(frame[1] | (frame[2]<<8));
						i += length;
            
            ext_cmd_id_t cmd_id = (ext_cmd_id_t)(frame[5] | (frame[6]<<8));
            /* Call corresponding function */
            switch(cmd_id) {
            case game_state:
                ext_game_state_interpret(&frame[7]);                        break;
            case game_result:
                ext_game_result_interpret(&frame[7]);                       break;
            case game_robot_HP:
                ext_game_robot_HP_interpret(&frame[7]);              break;
            case event_data:
                ext_event_data_interpret(&frame[7]);                        break;
            case supply_projectile_action:
                ext_supply_projectile_action_interpret(&frame[7]);          break;
            case supply_projectile_booking:
                ext_supply_projectile_booking_interpret(&frame[7]);         break;
            case game_robot_state:
                ext_game_robot_state_interpret(&frame[7]);                  break;
            case power_heat_data:
                ext_power_heat_data_interpret(&frame[7]);                   break;
            case game_robot_pos:
                ext_game_robot_pos_interpret(&frame[7]);                    break;
            case buff_musk:
                ext_buff_musk_interpret(&frame[7]);                         break;
            case aerial_robot_energy:
                aerial_robot_energy_interpret(&frame[7]);                   break;
            case robot_hurt:
                ext_robot_hurt_interpret(&frame[7]);                        break;
            case shoot_data:
                ext_shoot_data_interpret(&frame[7]);                        break;
            case robot_interactive_data:
                ext_student_interactive_header_data_interpret(&frame[7]);   break;
            default: ;
            }
        }
    }
}


uint8_t referee_buffer[256] = {0x00};
uint8_t dump_buffer[256];
void update_from_dma(void) {
    /* Clear IDLE flag bit */
    uint8_t bt = USART6->DR;
    spDMA.mem2mem.copy((uint32_t)dump_buffer, (uint32_t)referee_buffer, sizeof(referee_buffer));
		/* Reset DMA counter */
    spDMA.controller.reset_counter(spDMA_USART6_rx_stream, sizeof(referee_buffer));
    /* Search for frame and resolve data */
    frame_interpret(dump_buffer, sizeof(dump_buffer));
}


bool referee_send_robot(uint16_t data_id, ext_id_t target_id, uint8_t *data, uint8_t size) {
    /* Check data ID */
    if(data_id<REFEREE_STUDENT_ROBOT_MIN || data_id>REFEREE_STUDENT_ROBOT_MAX) {
        return false;
    }
    /* Limit data size */
    ext_robot_interactive_data_t robot_data;
    if (size>sizeof(robot_data.data)) size=sizeof(robot_data.data);
    robot_data.data_id = data_id;
    robot_data.sender_id = MY_ROBOT_ID;
    robot_data.robot_id = target_id;
    spDMA.mem2mem.copy((uint32_t)robot_data.data, (uint32_t)data, size);
    /* Send out data */
    spDMA.controller.start(spDMA_USART6_tx_stream, 
        (uint32_t)&robot_data, (uint32_t)USART6->DR, sizeof(robot_data));
    return true;
}


bool referee_send_client(ext_id_t target_id, float data[3], ext_client_custom_data_mask_t masks) {
    static ext_client_custom_data_t client_data;
    /* Header data */
    client_data.header.sof = REFEREE_FRAME_HEADER_SOF;
    client_data.header.seq ++;
    client_data.header.data_length = sizeof(client_data) 
        - sizeof(client_data.header) - sizeof(client_data.cmd_id) - sizeof(client_data.crc16);
    Append_CRC8_Check_Sum((uint8_t*)&client_data.header, sizeof(client_data.header));
    client_data.cmd_id = robot_interactive_data;
    /* Frame data */
    client_data.data_id = REFEREE_STUDENT_CLIENT_SOF;
    client_data.sender_id = MY_ROBOT_ID;
    client_data.client_id = target_id;
    client_data.data[0] = data[0];
    client_data.data[1] = data[1];
    client_data.data[2] = data[2];
    client_data.masks = masks;
    /* Calc CRC16 */
    Append_CRC16_Check_Sum((uint8_t*)&client_data, sizeof(client_data));
    /* Send out data */
    return spDMA.controller.start(spDMA_USART6_tx_stream, 
        (uint32_t)&client_data, (uint32_t)&USART6->DR, sizeof(client_data));
}


void referee_init(void) {
//    ext_game_robot_pos.x = 0;
//    ext_game_robot_pos.y = 0;
//    ext_game_robot_pos.z = 0;
//    ext_game_robot_pos.yaw = 0;
//    ext_power_heat_data.shooter_heat0 = 0;
//    ext_power_heat_data.shooter_heat1= 0;

    memset(referee_buffer, 0x00, sizeof(referee_buffer));

    // Start USART and DMA for send data
    USART_TX_Config(USART6, 115200);
    USART_RX_Config(USART6, 115200);
    DMA_USART_TX_Config(USART6);
    DMA_USART_RX_Config(USART6, (uint32_t)referee_buffer, sizeof(referee_buffer), true);

    DMA_ITConfig(spDMA_USART6_rx_stream, DMA_IT_TC, ENABLE);
    spIRQ.registe(DMA2_Stream1_IRQn, DMA_IT_TCIF1, update_from_dma);

    //    USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);
    //    spIRQ.registe(USART6_IRQn, USART_IT_IDLE, update_from_dma);
    
    USART_Cmd(USART6, ENABLE);
}


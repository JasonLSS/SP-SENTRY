/**
  ******************************************************************************
  * @file       sp_sentry.c
  * @author     LSS
  * @version    v0.0-alpha
  * @date       2019.Mar.12
  * @brief      shooting control
  * @usage      
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */
#include "sp_chasis.h"
#include "sp_sentry.h"
#include "sp_math.h"
#include "gimbal.h"
spSENTRY_Msg_t _sp_msg_rx_buffer, _sp_msg_tx_buffer;

#if  defined(BOARD_MODE) && (BOARD_MODE == 1)
struct _sentry_reg_t{
    spSENTRY_MSG_REG reg;
    int8_t last_address;           
    float time_stamp;             
    spSENTRY_Msg_t msg_recv;
} _sentry_reg;

int8_t MsgReg_Write(const uint8_t address, uint32_t value) {
    if(address<=sizeof(_sentry_reg.reg.__reg)/sizeof(*_sentry_reg.reg.__reg)) {
        _sentry_reg.reg.__reg[address] = value;
        _sentry_reg.last_address = address;
        return 0;
    }
    return -1;
}

int8_t MsgReg_Read(const uint8_t address, uint32_t* value) {
    if(address<=sizeof(_sentry_reg.reg.__reg)/sizeof(*_sentry_reg.reg.__reg)) {
        if(value) *value = _sentry_reg.reg.__reg[address];
        return 0;
    }
    return -1;
}

void Msg_Recv(CanRxMsg* msg, struct _sentry_reg_t* obj) {
    _sentry_reg.time_stamp = TASK_GetSecond();
    
    spSENTRY_Msg_t _sol_msg;
    memcpy(_sol_msg.data, msg->Data, sizeof(_sol_msg.data));
    spSENTRY_Msg_t ret = _sol_msg; 
    int8_t address = _sol_msg.frame.address;
		ret.frame.operation = feedback;
    switch(_sol_msg.frame.operation) {
    case read:
        if(MsgReg_Read(address, &(ret.frame.value_u32)) == 0) {
            ret.frame.appending = Succeed;
        } else {
            ret.frame.appending = Failed;
        }
        break;
    case write:
        if(MsgReg_Write(address, _sol_msg.frame.value_u32) == 0) {
            ret.frame.appending = Succeed;
        } else {
            ret.frame.appending = Failed;
        }
        break;
    default:
        ;
    }
    spSENTRY.communi.send(ret);
}

void Sentry_Looper(const uint32_t tick, RC_DataType* recv) {
//		float ltime = spSENTRY._system.time_stamp;
		float ctime = spSENTRY._system.time_stamp = TASK_GetSecond();
		spSENTRY.communi.online = ctime - _sentry_reg.time_stamp < 0.1f;
	
	  _sentry_reg.reg.state.shooter_heat0 = ext_power_heat_data.shooter_heat0;
		_sentry_reg.reg.state.ch[0] = (*recv).rc.ch0;
    _sentry_reg.reg.state.ch[1] = (*recv).rc.ch1;
    _sentry_reg.reg.state.ch[2] = (*recv).rc.ch2;
    _sentry_reg.reg.state.ch[3] = (*recv).rc.ch3;
	  _sentry_reg.reg.state.s1 = (*recv).rc.s1;
	  _sentry_reg.reg.state.s2 = (*recv).rc.s2;
		_sentry_reg.reg.state.yaw_angle = spGIMBAL_Controller._target.gimbal_yaw_motor->state.angle;
		frame_visual.yaw = - spSENTRY.communi.get_reg()->frame_yaw;
		if_if_newframe = spSENTRY.communi.get_reg()->frame_newframe;
		auto_aim_flag = spSENTRY.communi.get_reg()->auto_aim_flag;
}
#endif

#if defined(BOARD_MODE) && (BOARD_MODE == 2)
struct _sentry_reg_t{
    spSENTRY_MSG_REG reg;          
    float time_stamp;             
} _sentry_reg;

bool Msg_SendASK(spSENTRY_MSG_OPERATION ope, const uint8_t address, float value) {
    if(address == 0) {
        return false;
    }
    
    spSENTRY_Msg_t ask;
    ask.frame.address = address;                // Target address
    ask.frame.tag = RNG_GetRandomNumber();      // Random check value
    ask.frame.operation = ope;                  // Operation type
    if(ope == write) {
        ask.frame.value_f = value;              // Set value when in write mode
    } else {
        ask.frame.value_u32 = 0x00;
    }
    
    spSENTRY.communi.send(ask);
    return true;
}

void Msg_Recv(CanRxMsg* msg, struct _sentry_reg_t* obj) {
    spSENTRY_Msg_t *ret = (spSENTRY_Msg_t*)(msg->Data);
    // Succeed feedback
    if(ret->frame.operation==feedback && ret->frame.appending == Succeed) {
        uint8_t addr;
        spSENTRY.communi.get_addr( (void*)&spSENTRY.communi.get_reg()->state, &addr);
        if( ret->frame.address==addr ) {
            _sentry_reg.time_stamp = TASK_GetSecond();
        }
        _sentry_reg.reg.__reg[ret->frame.address] = ret->frame.value_u32;
    }
}

void Sentry_Looper(const uint32_t tick, RC_DataType* recv) {
		float ltime = spSENTRY._system.time_stamp;
		float ctime = spSENTRY._system.time_stamp = TASK_GetSecond();
		spSENTRY.communi.online = ctime - _sentry_reg.time_stamp < 0.1f;
//		spSENTRY.communi.get_reg()->robotMode = robotMode;
		if(spSENTRY.communi.online) {
			ext_power_heat_data.shooter_heat0 = spSENTRY.communi.get_reg()->state.shooter_heat0;
			spGIMBAL_Controller._target.gimbal_yaw_motor->state.angle = spSENTRY.communi.get_reg()->state.yaw_angle;
			(*recv).rc.ch0 = spSENTRY.communi.get_reg()->state.ch[0];
			(*recv).rc.ch1 = spSENTRY.communi.get_reg()->state.ch[1];
			(*recv).rc.ch2 = spSENTRY.communi.get_reg()->state.ch[2];
			(*recv).rc.ch3 = spSENTRY.communi.get_reg()->state.ch[3];
			(*recv).rc.s1 = spSENTRY.communi.get_reg()->state.s1;
			(*recv).rc.s2 = spSENTRY.communi.get_reg()->state.s2;
			 spSENTRY.communi.get_reg()->auto_aim_flag = auto_aim_flag;
//			if(tick%11 == 3){
//					uint8_t addr;
//					
//					if(spSENTRY.communi.get_addr( (void*)&spSENTRY.communi.get_reg()->robotMode, &addr) ) {
//                for(uint8_t _j=0; 
//                    _j<sizeof(spSENTRY.communi.get_reg()->robotMode)/sizeof(spSENTRY.communi.get_reg()->robotMode);
//                    _j++)
//                {
//                    spSENTRY.communi.send_request(write, addr + _j, spSENTRY.communi.get_reg()->robotMode);
//                }
//					}
//			}
		if(tick%5 == 0){
		 uint8_t addr;
        if(spSENTRY.communi.get_addr( (void*)&spSENTRY.communi.get_reg()->auto_aim_flag, &addr) ) {
                spSENTRY.communi.send_request(write, addr,  spSENTRY.communi.get_reg()->auto_aim_flag);
        }
					
    }
		}
		
		if(tick%5 == 1){
		 uint8_t addr;
        if(spSENTRY.communi.get_addr( (void*)&spSENTRY.communi.get_reg()->state, &addr) ) {
                spSENTRY.communi.send_request(read, addr + 3, 0);
        }
					
    }
		
		if(tick%5 == 2){
		 uint8_t addr;
        if(spSENTRY.communi.get_addr( (void*)&spSENTRY.communi.get_reg()->state, &addr) ) {
                spSENTRY.communi.send_request(read, addr, 0);
        }
					
    }
		
		if(tick%5 == 3){
		 uint8_t addr;
        if(spSENTRY.communi.get_addr( (void*)&spSENTRY.communi.get_reg()->state, &addr) ) {
                spSENTRY.communi.send_request(read, addr + 1, 0);	
				}
		}
		
		if(tick%5 == 4){
		 uint8_t addr;
        if(spSENTRY.communi.get_addr( (void*)&spSENTRY.communi.get_reg()->state, &addr) ) {
                spSENTRY.communi.send_request(read, addr + 2, 0);	
				}
		}

}



#endif

#if defined(SP_SENTRY) && defined(BOARD_MODE) && (BOARD_MODE == 2 || BOARD_MODE == 1)
void Sentry_Init(void){
		memset(&_sentry_reg, 0x00, sizeof(spSENTRY_MSG_REG));
    
    spSENTRY._system._can_rx.owner = &_sentry_reg;
#if defined(BOARD_MODE) && (BOARD_MODE == 2)
    spSENTRY._system._can_rx.std_id = SENTRY_UPPER_ID;
#elif defined(BOARD_MODE) && (BOARD_MODE == 1)
    spSENTRY._system._can_rx.std_id = SENTRY_BOTTOM_ID;
#endif
    spSENTRY._system._can_rx.rx.addr = _sp_msg_rx_buffer.data;
    spSENTRY._system._can_rx.rx.size = sizeof(spSENTRY_Msg_t);
    spSENTRY._system._can_rx.resolver = (ResolverCallback_t)Msg_Recv;
    
#if defined(BOARD_MODE) && (BOARD_MODE == 2)
    spSENTRY._system._can_tx.std_id = SENTRY_BOTTOM_ID;
#elif defined(BOARD_MODE) && (BOARD_MODE == 1)
    spSENTRY._system._can_tx.std_id = SENTRY_UPPER_ID;
		_sentry_reg.last_address = -1;
#endif
    spSENTRY._system._can_tx.tx.addr = _sp_msg_tx_buffer.data;
    spSENTRY._system._can_tx.tx.size = sizeof(spSENTRY_Msg_t);
    
    spCAN.user.registe_receiver(CAN2, &spSENTRY._system._can_rx);
//    spCAN.user.registe_transmitter(CAN2, &spSENTRY._system._can_tx);

}

void Sentry_Send(spSENTRY_Msg_t frame) {
    memcpy(spSENTRY._system._can_tx.tx.addr, frame.data, sizeof(frame.data));
    spCAN.user.send(CAN2, &spSENTRY._system._can_tx);
}

bool Sentry_GetMsgAddr(void* var, uint8_t* addr) {
    
    spSENTRY_MSG_REG* reg = &_sentry_reg.reg;
    
    uint8_t _addr = (uint32_t*)var - (uint32_t*)reg;
    if(_addr < sizeof(reg->__reg)/sizeof(*(reg->__reg))) {
        if(addr) *addr = _addr;
        return true;
    }
    
    if(addr) *addr = NULL;
    return false;
}

spSENTRY_MSG_REG* Sentry_GetMsgRegister(void) {
    return &_sentry_reg.reg;
}

RobotMode Sentry_GetRobotmode(void) {
    return robotMode;
}

struct __SENTRY_Manager_Type spSENTRY = {
    ._system = {
        .time_stamp = 0,
        .init = Sentry_Init,
        .looper = Sentry_Looper,
    },
    
    .get_state = Sentry_GetRobotmode,
    
    .communi = {
        .send = Sentry_Send,
        .get_addr = Sentry_GetMsgAddr,
        .get_reg = Sentry_GetMsgRegister,
        .online = false,
#if  defined(BOARD_MODE) && ( BOARD_MODE == 2)
				.send_request = Msg_SendASK,
#endif
    },
#if  defined(BOARD_MODE) && (BOARD_MODE == 2)

#elif  defined(BOARD_MODE) && (BOARD_MODE == 0 || BOARD_MODE == 1)
    
#endif
};
#endif

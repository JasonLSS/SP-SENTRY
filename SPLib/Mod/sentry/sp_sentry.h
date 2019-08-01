/**
  ******************************************************************************
  * @file       sp_sentry.h
  * @author     LSS
  * @version    v0.0-alpha
  * @date       2019.July.25
  * @brief      Special control operationss for sentry robot.
  * @usage      
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SP_SENTRY_H
#define __SP_SENTRY_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "sp_conf.h"
#include "sp_rc.h"
#include "referee.h"
#include "sp_shoot.h"
#include "Auto_aim.h"

#define SENTRY_UPPER_ID        0x00A
#define SENTRY_BOTTOM_ID       0x00D
#define MSG_REG_SIZE        ((1<<6)+1)

typedef enum {
    write=0,
    read=1,
    feedback=2,
} spSENTRY_MSG_OPERATION;

typedef enum {
    Succeed=0xaa,
    Failed=0xdd,
    Unknown=0xff,
} spSENTRY_MSG_APPENDING;


typedef packed_union {
    uint8_t data[8];
    packed_struct {
        uint8_t operation:2;
        uint8_t address:6;
        uint16_t tag;
        uint8_t appending;
        packed_union {
            uint8_t value_u8a[4];
            uint32_t value_u32;
            float value_f;
        };
    } frame;
} spSENTRY_Msg_t;

typedef packed_union_n(4) {
    uint32_t __value[3];
    packed_struct_n(4) {
        int16_t         ch[4];
				uint8_t         s1;
        uint8_t         s2;
				uint16_t        shooter_heat0;
				float        		yaw_angle;
    };
} spSENTRY_Msg_State_t;

typedef volatile packed_union_n(4) {
    uint32_t __reg[MSG_REG_SIZE];       //!< 128 bytes exchange register
    packed_struct_n(4) {
        uint32_t :32;
				spSENTRY_Msg_State_t state;
				float frame_yaw;
				float frame_newframe;
				float auto_aim_flag;

    };
} spSENTRY_MSG_REG;

extern struct __SENTRY_Manager_Type {
    struct {
        CAN_Transmitter _can_tx;
        CAN_Receiver _can_rx;
        
        float time_stamp;
        void (*init)(void);
			  void (*looper)(const uint32_t tick, RC_DataType* recv);
    } _system;
    
    RobotMode (*get_state)(void);

    struct {
        void (*send)(spSENTRY_Msg_t value);
        bool (*get_addr)(void* var, uint8_t* addr);
        spSENTRY_MSG_REG* (*get_reg)(void);
        bool online;
			  bool (*send_request)(spSENTRY_MSG_OPERATION ope, const uint8_t address, float value);     
    } communi;                              /*!< Communication interfaces */
} spSENTRY;



#ifdef __cplusplus
}
#endif
#endif /*__SP_SENTRY_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/

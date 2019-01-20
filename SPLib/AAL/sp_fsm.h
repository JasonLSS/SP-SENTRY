/**
  ******************************************************************************
  * @file       sp_fsm.h
  * @author     YTom
  * @version    v0.0-alpha
  * @date       2018.Nov.01
  * @brief      Framework for Finite State Machine
  * @usage      
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SP_FSM_H
#define __SP_FSM_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>

/* Exported types ------------------------------------------------------------*/
/** 
  * @brief  event type for FSM
  */
typedef struct {
    /** @brief  user-define event type
      * @note   must be unique for each FSM */
    uint8_t     event_type;
    /** @brief  user-defined event date for tansition judging
      * @note   this void-type pointer data point to user-defined data,
      *         please make sure that dates used for same purpose have
      *         the SAME data-type, or may cause some error. */
    void*       event_data;
} FSM_Event;

/** 
  * @brief  tansition type for FSM
  */
typedef struct __FSM_State FSM_State;
typedef struct {
    /** @brief  event type which can make this tansition enable */
    uint8_t                 event_type;
    /** @brief  user-defined date for tansition judging */
    void*                   condition;
    /** @brief  state which this tansition point to */
    FSM_State*              next_state;
    /** @brief  constrain for 'condition'
      * @param  condition: condition from this tansition for checking
      * @param  event: event that trigger this tansition
      * @note   may be NULL */
    uint8_t ( *constrain )( void *condition, FSM_Event *event );
    /** @brief  action that should be done by this tansition
      * @param  curr_data: data from the CURRENT STATE
      * @param  event: event that trigger this tansition
      * @param  next_data: data from the NEXT STATE
      * @note   may be NULL */
    void ( *action )( void *curr_data, FSM_Event *event, void *next_data );
} FSM_Transition;

/** 
  * @brief  state type for FSM
  */
typedef struct __FSM_State {
    /** @brief  transitions that point from and to current state
      * @note   this is the header of a/an ARRAY/LIST/QUENE/... */
    FSM_Transition*     transitions;
    /** @brief  the number of the total transitions */
    uint8_t             num_transitions;
    /** @brief  user-defined date for any purpose
      * @note    */
    void*       state_data;
    /** @brief  function called once entering this state
      * @param  state_data: the 'state_data' of this state
      * @param  event: event triggered the transition
      * @note   may be NULL
      *         will NOT be called if loop running from self state */
    void ( *entry )( void *state_data, FSM_Event *event );
    /** @brief  function called once leaving this state
      * @param  state_data: the 'state_data' of this state
      * @param  event: event triggered the transition
      * @note   may be NULL */
    void ( *exit )( void *state_data, FSM_Event *event );
} FSM_State;

/** 
  * @brief  Finite Satet Machine type
  */
typedef struct {
    /** @brief  current active state */
    FSM_State*   curretSate;
    /** @brief  previous state for looping checking */
    FSM_State*   previousSate;
    /** @brief  special state for error */
    FSM_State*   errorState;
} FSM_StateMachine;

/** 
  * @brief  FSM_EventHandler() return values
  */
enum FSM_StatusVal
{
   FSM_StatusVal_Exception = -2,
   FSM_StatusVal_ErrorState,
   FSM_StatusVal_Noop,
   FSM_StatusVal_Loop,
   FSM_StatusVal_Transit,
   FSM_StatusVal_FinalState,
};

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup FSM User Interfaces
  * @{
  */
/**
  * @brief  init a Finite State Machine
  * @note   
  *
  *
  *
  *
  * @param  fsm: the state machine to init
  * @param  init: the init state
  * @param  error: the error state
  */ 
inline void FSM_Init( FSM_StateMachine *fsm, FSM_State *init, FSM_State *error );

/**
  * @brief  FMS event handler
  * @note   transit to self will NOT trigger entry action
  * @param  fsm: the FSM to pass an event
  * @param  event: the event suspended
  * @retval @ref FSM_StatusVal
  */ 
inline enum FSM_StatusVal FSM_EventHandler( FSM_StateMachine *fsm, FSM_Event *event );

/**
  * @brief  get current state of this FSM
  * @param  stateMachine: the FSM to get state from
  * @retval @ref FSM_State
  */ 
inline FSM_State *FSM_GetCurrentState( FSM_StateMachine *fsm );

/**
  * @brief  get previous state of this FSM
  * @param  stateMachine: the FSM to get state from
  * @retval @ref FSM_State
  */ 
inline FSM_State *FSM_GetPreviousState( FSM_StateMachine *fsm );
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /*__SP_FSM_H */

/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/

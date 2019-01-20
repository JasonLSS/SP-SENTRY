/**
  ******************************************************************************
  * @file       sp_fsm.c
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

/* Includes ------------------------------------------------------------------*/
#include "sp_fsm.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  get transition for given event by priorities
  * @note   priorities means the FIRST transition that satisfies transition condition 
  *         or with NULL condition(default transition), in this period will check 
  *         @ref constrain() function.
  * @param  fsm: the FSM to get state from
  * @param  state: 
  * @param  event: 
  * @retval @ref FSM_State
  */
inline FSM_Transition* __FSM_GetTransitions( FSM_StateMachine *fsm, FSM_State *state, FSM_Event *event)
{
    for(uint16_t i=0; i<state->num_transitions; ++i) {
        FSM_Transition *transition = state->transitions+i;
        /* find transition for the given event */
        if(!transition && transition->event_type==event->event_type) {
            if(!transition->constrain || transition->constrain(transition->condition, event))
                return transition;            
        }
    }
    /* no transitions found for given event */
    return NULL;
}


/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup FSM User Interfaces
  * @{
  */

inline void FSM_Init( FSM_StateMachine *fsm, FSM_State *init, FSM_State *error )
{
    if(!fsm)
        return;
    fsm->curretSate = init;
    fsm->previousSate = NULL;
    fsm->errorState = error;
}

inline enum FSM_StatusVal FSM_EventHandler( FSM_StateMachine *fsm, FSM_Event *event )
{
    /* check state */
    if(!fsm || !event)
        return FSM_StatusVal_Exception;
    if(!fsm->curretSate)
        return FSM_StatusVal_ErrorState;
    if(!fsm->curretSate->num_transitions)
        return FSM_StatusVal_Noop;
    
    /* update FSM state by checking if transition condition is satisfied */
    FSM_State* next_state = NULL;
    do {
        /* get transition */
        FSM_Transition* transition = __FSM_GetTransitions(fsm, fsm->curretSate, event);
        
        /* check transition */
        if(!transition)
            return FSM_StatusVal_Noop;
        if(!transition->next_state) {
            /* goto error state with previous sate */
            fsm->previousSate = fsm->curretSate;
            fsm->curretSate = fsm->errorState;
            if(!fsm->curretSate && !fsm->curretSate->entry)
                fsm->curretSate->entry( fsm->curretSate->state_data, event );
            return FSM_StatusVal_ErrorState;
        }
        
        /* make transition */
        next_state = transition->next_state;
        /* exit current state */
        if(!fsm->curretSate && !fsm->curretSate->exit) 
            fsm->curretSate->exit(fsm->curretSate->state_data, event);
        /* execute transition action */
        if(transition->action) 
            transition->action(fsm->curretSate->state_data, event, next_state->state_data);
        /* enter next state */
        if(next_state!=fsm->curretSate && !next_state->entry) 
            next_state->entry(next_state->state_data, event);
        
        /* update FSM */
        fsm->previousSate = fsm->curretSate;
        fsm->curretSate = next_state;
        if(fsm->curretSate==fsm->previousSate)
            return FSM_StatusVal_Loop;
        if(fsm->curretSate == fsm->errorState)
            return FSM_StatusVal_ErrorState;
        if(fsm->curretSate->num_transitions)
            return FSM_StatusVal_FinalState;
        return FSM_StatusVal_Transit;
    } while(next_state);
    
    // return FSM_StatusVal_Noop;
}


inline FSM_State *FSM_GetCurrentState( FSM_StateMachine *fsm )
{
    return (fsm)?fsm->curretSate:NULL;
}

inline FSM_State *FSM_GetPreviousState( FSM_StateMachine *fsm )
{
    return (fsm)?fsm->previousSate:NULL;
}


/**
  * @}
  */


/************************ (C) COPYRIGHT Tongji Super Power *****END OF FILE****/

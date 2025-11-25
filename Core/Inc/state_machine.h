/*******************************************************************************
 * @file state_machine.h
 * @brief High level Maze Bot state machine.
 *******************************************************************************
 */

#ifndef MAZE_BOT__STATE_MACHINE_H
#define MAZE_BOT__STATE_MACHINE_H
#include <stdint.h>

/** Public types. *************************************************************/

typedef enum state_machine {
  STATE_IDLE,
  STATE_INIT,
  STATE_STANDBY,
  STATE_TURN,
  STATE_TURN_FOR_TICKS,
  STATE_ADVANCE_FOR_TICKS,
  STATE_ADVANCE_UNTIL_MM,
  STATE_CORRIDOR_FOR_TICKS,
  STATE_CORRIDOR_UNTIL_MM,
  STATE_PICKUP_PACKAGE,
  STATE_DROP_PACKAGE,
} state_t;

typedef struct playbook_act {
  state_t state;   // State machine state.
  float condition; // Generalized condition value.
  // (Type cast in state machine if required).
  uint16_t watchdog_max; // Maximum state soft-watchdog tick count.
} act_t;

/** Public variables. *********************************************************/

extern state_t bot_state;

/** Public functions. *********************************************************/

void run_state_machine(void);

#endif

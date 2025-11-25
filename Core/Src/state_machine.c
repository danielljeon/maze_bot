/*******************************************************************************
 * @file state_machine.c
 * @brief High level Maze Bot state machine.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "state_machine.h"
#include "configuration.h"
#include "controls.h"
#include "drive.h"
#include "h_bridge_control.h"
#include "maze_navigation.h"
#include "stm32l4xx_hal.h"
#include "vl53l4cd_runner.h"

/** Private variables. ********************************************************/

static uint16_t state_standby_counter = 0;

/** Definitions. **************************************************************/

#define STATE_MACHINE_PLAYBOOK_COUNT 7
#define STATE_MACHINE_INITIAL_STATE STATE_INIT
#define STATE_MACHINE_FINAL_STATE STATE_IDLE

/** Private variables. ********************************************************/

// State machine playbook.
uint8_t playbook_index = 0;
state_t playbook[STATE_MACHINE_PLAYBOOK_COUNT] = {
    STATE_MACHINE_INITIAL_STATE, // Start.
    STATE_STANDBY,
    STATE_CORRIDOR_UNTIL_MM,
    STATE_TURN_FOR_TICKS,
    STATE_CORRIDOR_UNTIL_MM,
    STATE_TURN_FOR_TICKS,
    STATE_MACHINE_FINAL_STATE, // End.
};

// Align state transition condition values to match state playbook_index.
static float condition[STATE_MACHINE_PLAYBOOK_COUNT] = {
    0,          // STATE_MACHINE_INITIAL_STATE.
    0,          // STATE_STANDBY.
    100,        // STATE_CORRIDOR_UNTIL_MM.
    -1.570796f, // STATE_TURN_FOR_TICKS.
    700,        // STATE_CORRIDOR_UNTIL_MM.
    -1.570796f, // STATE_TURN_FOR_TICKS.
    0,          // STATE_MACHINE_FINAL_STATE.
}; // Stored as float and cast within the respective state switch case.
static uint16_t condition_count = 0;

// Soft state watchdog timers.
static uint16_t watchdog[STATE_MACHINE_PLAYBOOK_COUNT] = {
    0,     // STATE_MACHINE_INITIAL_STATE.
    0,     // STATE_STANDBY.
    10000, // STATE_CORRIDOR_UNTIL_MM.
    1000,  // STATE_TURN_FOR_TICKS.
    5000,  // STATE_CORRIDOR_UNTIL_MM.
    1000,  // STATE_TURN_FOR_TICKS.
    0,     // STATE_MACHINE_FINAL_STATE.
};
static uint16_t watchdog_count = 0;

// Previously in TURN state flag for maze navigation code.
static bool previously_turning = false;

/** Public variables. *********************************************************/

state_t bot_state = STATE_MACHINE_INITIAL_STATE;

/** Private functions. *********************************************************/

static void idle(void) {
  // Coast drive base motors.
  h_bridge_1_command(GPIO_PIN_RESET, 0); // Directional, but no movement.
  h_bridge_2_command(GPIO_PIN_RESET, 0); // Directional, but no movement.
}

static void pickup_package(void) {}

static void drop_package(void) {}

static void init(void) {
  // Initialize claw.
  pickup_package();
}

static void next_state(void) {
  playbook_index += 1;

  // Clamp the playbook for safety in case of overrun.
  if (playbook_index >= STATE_MACHINE_PLAYBOOK_COUNT) {
    bot_state = STATE_IDLE;
  } else {
    bot_state = playbook[playbook_index]; // Increment playbook.
  }

  // Clear watchdog counter and condition counter.
  watchdog_count = 0;
  condition_count = 0;
}

/** Public functions. *********************************************************/

void run_state_machine(void) {
  const float left_mm = vl53l4cd_distance_mm[0];
  const float front_mm = vl53l4cd_distance_mm[1];
  const float right_mm = vl53l4cd_distance_mm[2];

  switch (bot_state) {

  case STATE_IDLE:
    idle();
    // TODO: SHOULD ADD A CASCADED STATE BASED PID HEADING CONTROL DISABLEMENT.
    break;

  case STATE_INIT:
    init();
    next_state();
    break;

  case STATE_STANDBY:
    if (state_standby_counter > 10) {
      // Initialize startup controls.
      zero_heading();
      next_state();
    } else if (front_mm > 100) {
      state_standby_counter++;
    }
    break;

  case STATE_TURN:
    set_relative_heading(condition[playbook_index]);
    next_state();
    break;

  case STATE_TURN_FOR_TICKS:
    if (watchdog_count == 0) {
      set_relative_heading(condition[playbook_index]);
      watchdog_count++;
    } else if (watchdog_count > watchdog[playbook_index]) {
      next_state();
    } else {
      watchdog_count++;
    }
    break;

  case STATE_ADVANCE_FOR_TICKS:
    if (watchdog_count > watchdog[playbook_index]) {
      h_bridge_linear = 0.0f;
      next_state();
    } else {
      h_bridge_linear = 0.35f;
      watchdog_count++;
    }
    break;

  case STATE_ADVANCE_UNTIL_MM:
    if (watchdog_count > watchdog[playbook_index] ||
        front_mm <= condition[playbook_index]) {
      h_bridge_linear = 0.0f;
      next_state();
    } else {
      h_bridge_linear = 0.35f;
      watchdog_count++;
    }
    break;

  case STATE_CORRIDOR_FOR_TICKS:
    if (watchdog_count > watchdog[playbook_index]) {
      corridor_stop();
      next_state();
    } else {
      corridor_straight();
      watchdog_count++;
    }
    break;

  case STATE_CORRIDOR_UNTIL_MM:
    if (watchdog_count > watchdog[playbook_index] ||
        front_mm <= condition[playbook_index]) {
      corridor_stop();
      next_state();
    } else {
      corridor_straight();
      watchdog_count++;
    }
    break;

  case STATE_PICKUP_PACKAGE:
    pickup_package();
    next_state();
    break;

  case STATE_DROP_PACKAGE:
    drop_package();
    next_state();
    break;

  default:
    break;
  }
}

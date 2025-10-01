/*******************************************************************************
 * @file state_machine.c
 * @brief High level Maze Bot state machine.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "state_machine.h"

#include "bno085_runner.h"
#include "controls.h"
#include "drive.h"
#include "h_bridge_control.h"
#include "servo_control.h"
#include "stm32l4xx_hal.h"
#include "vl53l4cd_runner.h"

/** Private variables. ********************************************************/

static uint16_t state_standby_counter = 0;

/** Definitions. **************************************************************/

#define PLAYBOOK_COUNT 6
#define INITIAL_STATE STATE_INIT

/** Private variables. ********************************************************/

// State machine playbook.
uint8_t playbook_index = 0;
state_t playbook[PLAYBOOK_COUNT] = {
    INITIAL_STATE, STATE_STANDBY,      STATE_TURN,
    STATE_ADVANCE, STATE_DROP_PACKAGE, STATE_IDLE,
};

// Align secondary state based arrays to match state playbook_index.
static float turns[PLAYBOOK_COUNT] = {0, 0, 0.45f, 0, 0, 0}; // Radians.
static uint16_t advance_counter_limits[PLAYBOOK_COUNT] = {0, 0, 0, 875, 0, 0};
static uint16_t current_advance_counter = 0;

/** Public variables. *********************************************************/

state_t bot_state = INITIAL_STATE;

/** Private functions. *********************************************************/

static void idle(void) {
  // Coast drive base motors.
  h_bridge_1_command(GPIO_PIN_RESET, GPIO_PIN_RESET, 0);
  h_bridge_2_command(GPIO_PIN_RESET, GPIO_PIN_RESET, 0);
}

static void pickup_package(void) { servo_command(1000); }

static void drop_package(void) { servo_command(2000); }

static void init(void) {
  // Initialize claw.
  pickup_package();
}

static void next_state(void) {
  playbook_index += 1;

  // Clamp the playbook for safety in case of overrun.
  if (playbook_index >= PLAYBOOK_COUNT) {
    bot_state = STATE_IDLE;
  } else {
    bot_state = playbook[playbook_index]; // Increment playbook.
  }
}

/** Public functions. *********************************************************/

// TODO(Maze bot): CURRENTLY HAS HARDCODED OVERRIDES.
void run_state_machine(void) {
  switch (bot_state) {
  case STATE_IDLE:
    idle();
    // TODO: SHOULD ADD A CASCADED STATE BASED PID HEADING CONTROL DISABLEMENT.
    break;

  case STATE_INIT:
    init();

    // Initialize and start the VL53L4CD.
    vl53l4cd_init();
    vl53l4cd_start();

    next_state();
    break;

  case STATE_STANDBY:
    if (state_standby_counter > 10) {
      // Stop the VL53L4CD.
      vl53l4cd_stop();

      // Initialize BNO085.
      bno085_reset();
      bno085_init();

      // Initialize startup controls.
      zero_heading();

      next_state();

    } else if (distance_mm > 100) {
      state_standby_counter++;
    }

    break;

  case STATE_MOMENTARY_SENSE:
    break;

  case STATE_TURN:

    set_relative_heading(turns[playbook_index]);

    next_state();
    break;

  case STATE_ADVANCE:
    if (current_advance_counter > advance_counter_limits[playbook_index]) {
      h_bridge_linear = 0.0f;

      next_state();
      current_advance_counter = 0; // Clear for next STATE_ADVANCE entrance.

    } else {
      h_bridge_linear = 0.01f;
      current_advance_counter++;
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

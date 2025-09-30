/*******************************************************************************
 * @file state_machine.c
 * @brief High level Maze Bot state machine.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "state_machine.h"
#include "controls.h"
#include "drive.h"
#include "h_bridge_control.h"
#include "servo_control.h"
#include "stm32l4xx_hal.h"

/** Private variables. ********************************************************/

static uint16_t state_advance_counter = 0;

/** Public variables. *********************************************************/

state_t bot_state = STATE_INIT;

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

/** Public functions. *********************************************************/

// TODO(Maze bot): CURRENTLY HAS HARDCODED OVERRIDES.
void run_state_machine(void) {
  switch (bot_state) {
  case STATE_IDLE:
    idle();
    break;
  case STATE_INIT:
    init();

    // Initialize startup controls.
    zero_heading();

    bot_state = STATE_TURN;
    break;
  case STATE_MOMENTARY_SENSE:
    break;
  case STATE_TURN:

    set_relative_heading(0.45f);

    bot_state = STATE_ADVANCE;
    break;
  case STATE_ADVANCE:
    if (state_advance_counter > 200) {
      h_bridge_linear = 0.0f;
      bot_state = STATE_DROP_PACKAGE;

    } else {
      h_bridge_linear = 0.01f;
      state_advance_counter++;
    }

    break;
  case STATE_PICKUP_PACKAGE:
    pickup_package();
    break;
  case STATE_DROP_PACKAGE:
    drop_package();

    bot_state = STATE_IDLE;
    break;
  default:
    break;
  }
}

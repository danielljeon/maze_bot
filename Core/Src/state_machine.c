/*******************************************************************************
 * @file state_machine.c
 * @brief High level Maze Bot state machine.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "state_machine.h"
#include "bno085_runner.h"
#include "h_bridge_control.h"
#include "servo_control.h"
#include "stm32l4xx_hal.h"

/** Public variables. *********************************************************/

state_t bot_state = STATE_INIT;

uint8_t angle_target_reach_count = 0;
uint16_t advance_reach_count = 0;

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

void run_state_machine(void) {
  switch (bot_state) {
  case STATE_IDLE:
    idle();
    break;
  case STATE_INIT:
    init();
    break;
  case STATE_MOMENTARY_SENSE:
    break;
  case STATE_TURN:
    break;
  case STATE_ADVANCE:
    break;
  case STATE_PICKUP_PACKAGE:
    pickup_package();
    break;
  case STATE_DROP_PACKAGE:
    drop_package();
    break;
  default:
    break;
  }
}

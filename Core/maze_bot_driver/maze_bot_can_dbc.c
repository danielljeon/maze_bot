/*******************************************************************************
 * @file maze_bot_can_dbc.c
 * @brief Auto-generated CAN message definitions from DBC file.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "maze_bot_can_dbc.h"

#include "maze_navigation.h"

/** Public variables. *********************************************************/

const can_message_t dbc_messages[] = {
    {
        .name = "msg1",
        .message_id = 256,
        .id_mask = 0xFFFFFFFF,
        .dlc = 8,
        .rx_handler = (can_rx_handler_t)process_msg1,
        .tx_handler = 0, // (can_tx_handler_t)my_tx_handler_func,
        .signal_count = 2,
        .signals =
            {
                {
                    .name = "v1",
                    .start_bit = 0,
                    .bit_length = 32,
                    .byte_order = CAN_LITTLE_ENDIAN,
                    .is_signed = false,
                    .scale = 1.0f,
                    .offset = 0.0f,
                    .min_value = -3.4e+38f,
                    .max_value = 3.4e+38f,
                },
                {
                    .name = "v2",
                    .start_bit = 32,
                    .bit_length = 32,
                    .byte_order = CAN_LITTLE_ENDIAN,
                    .is_signed = false,
                    .scale = 1.0f,
                    .offset = 0.0f,
                    .min_value = -3.4e+38f,
                    .max_value = 3.4e+38f,
                },
            },
    },
    {
        .name = "msg2",
        .message_id = 257,
        .id_mask = 0xFFFFFFFF,
        .dlc = 8,
        .rx_handler = (can_rx_handler_t)process_msg2,
        .tx_handler = 0, // (can_tx_handler_t)my_tx_handler_func,
        .signal_count = 2,
        .signals =
            {
                {
                    .name = "v3",
                    .start_bit = 0,
                    .bit_length = 32,
                    .byte_order = CAN_LITTLE_ENDIAN,
                    .is_signed = false,
                    .scale = 1.0f,
                    .offset = 0.0f,
                    .min_value = -3.4e+38f,
                    .max_value = 3.4e+38f,
                },
                {
                    .name = "v4",
                    .start_bit = 32,
                    .bit_length = 32,
                    .byte_order = CAN_LITTLE_ENDIAN,
                    .is_signed = false,
                    .scale = 1.0f,
                    .offset = 0.0f,
                    .min_value = -3.4e+38f,
                    .max_value = 3.4e+38f,
                },
            },
    },
    {
        .name = "state",
        .message_id = 257,
        .id_mask = 0xFFFFFFFF,
        .dlc = 1,
        .rx_handler = 0, // (can_rx_handler_t)my_rx_handler_func,
        .tx_handler = 0, // (can_tx_handler_t)my_tx_handler_func,
        .signal_count = 1,
        .signals =
            {
                {
                    .name = "system_state",
                    .start_bit = 0,
                    .bit_length = 8,
                    .byte_order = CAN_LITTLE_ENDIAN,
                    .is_signed = false,
                    .scale = 1.0f,
                    .offset = 0.0f,
                    .min_value = 0.0f,
                    .max_value = 255.0f,
                },
            },
    },
    {
        .name = "rtc",
        .message_id = 600,
        .id_mask = 0xFFFFFFFF,
        .dlc = 8,
        .rx_handler = 0, // (can_rx_handler_t)my_rx_handler_func,
        .tx_handler = 0, // (can_tx_handler_t)my_tx_handler_func,
        .signal_count = 8,
        .signals =
            {
                {
                    .name = "rtc_state",
                    .start_bit = 0,
                    .bit_length = 8,
                    .byte_order = CAN_LITTLE_ENDIAN,
                    .is_signed = false,
                    .scale = 1.0f,
                    .offset = 0.0f,
                    .min_value = 0.0f,
                    .max_value = 255.0f,
                },
                {
                    .name = "rtc_year",
                    .start_bit = 8,
                    .bit_length = 8,
                    .byte_order = CAN_LITTLE_ENDIAN,
                    .is_signed = false,
                    .scale = 1.0f,
                    .offset = 2000.0f,
                    .min_value = 2000.0f,
                    .max_value = 2099.0f,
                },
                {
                    .name = "rtc_month",
                    .start_bit = 16,
                    .bit_length = 8,
                    .byte_order = CAN_LITTLE_ENDIAN,
                    .is_signed = false,
                    .scale = 1.0f,
                    .offset = 0.0f,
                    .min_value = 1.0f,
                    .max_value = 12.0f,
                },
                {
                    .name = "rtc_day",
                    .start_bit = 24,
                    .bit_length = 8,
                    .byte_order = CAN_LITTLE_ENDIAN,
                    .is_signed = false,
                    .scale = 1.0f,
                    .offset = 0.0f,
                    .min_value = 1.0f,
                    .max_value = 31.0f,
                },
                {
                    .name = "rtc_weekday",
                    .start_bit = 32,
                    .bit_length = 8,
                    .byte_order = CAN_LITTLE_ENDIAN,
                    .is_signed = false,
                    .scale = 1.0f,
                    .offset = 0.0f,
                    .min_value = 1.0f,
                    .max_value = 7.0f,
                },
                {
                    .name = "rtc_hour",
                    .start_bit = 40,
                    .bit_length = 8,
                    .byte_order = CAN_LITTLE_ENDIAN,
                    .is_signed = false,
                    .scale = 1.0f,
                    .offset = 0.0f,
                    .min_value = 0.0f,
                    .max_value = 23.0f,
                },
                {
                    .name = "rtc_minute",
                    .start_bit = 48,
                    .bit_length = 8,
                    .byte_order = CAN_LITTLE_ENDIAN,
                    .is_signed = false,
                    .scale = 1.0f,
                    .offset = 0.0f,
                    .min_value = 0.0f,
                    .max_value = 59.0f,
                },
                {
                    .name = "rtc_second",
                    .start_bit = 56,
                    .bit_length = 8,
                    .byte_order = CAN_LITTLE_ENDIAN,
                    .is_signed = false,
                    .scale = 1.0f,
                    .offset = 0.0f,
                    .min_value = 0.0f,
                    .max_value = 59.0f,
                },
            },
    },
};

const int dbc_message_count = sizeof(dbc_messages) / sizeof(dbc_messages[0]);

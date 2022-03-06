/* UART functionality

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#pragma once
#include "driver/uart.h"

// UART pins
#define IO_TXD (4)
#define IO_RXD (5)
#define IO_RTS (6)
#define IO_CTS (7)

#define UART_PORT_NUM   (1)

extern uart_config_t uart_config;
extern int uart_flow_ctrl;

/**
 * @brief Preparations for UART operations
 *
 * Initialize the UART
 *
 */
void init_uart();

/**
 * @brief Configure UART
 *
 * Set UART comm settings
 *
 */
void set_uart_comm();

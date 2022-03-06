/* USB_lib_task and handlers

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#pragma once
#include "usb/cdc_acm_host.h"


extern cdc_acm_line_coding_t cdc_acm_line_coding;
extern bool CDC_connected;
extern cdc_acm_dev_hdl_t cdc_dev;

/**
 * @brief data_cb
 *
 * Data callback
 *
 */
void handle_rx(uint8_t *data, size_t data_len, void *arg);

/**
 * @brief event_cb
 *
 * Event callback
 *
 */
void handle_notify(cdc_acm_dev_hdl_t cdc_hdl, const cdc_acm_host_dev_event_data_t *event, void *user_ctx);

/**
 * @brief Preparations for USB operations
 *
 * Initialize the USB CDC/ACM
 *
 */
void init_cdc_acm();

/**
 * @brief Configure CDC
 *
 * Set CDC comm settings
 *
 */
void set_CDC_comm();

/**
 * @brief Try to open a CDC device
 *
 * Try to open first found CDC device
 *
 */
void CDC_try_open();

/**
 * @brief Write bytes over CDC
 *
 * Write bytes to CDC device
 *
 */
void CDC_write_bytes(const uint8_t *data, size_t data_len);

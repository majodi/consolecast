/* Common functions to establish Wi-Fi connection.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
 */

#pragma once
#include "stdbool.h"
#include "esp_err.h"

// AP scan list, code assumes only 1 digit (0-9) --> max setting here for scan list size is 10
#define DEFAULT_SCAN_LIST_SIZE (10)

extern bool isSTA;


/**
 * @brief Configure Wi-Fi, start AP, wait for AP started
 *
 * This helper function starts an AP
 *
 * @return ESP_OK on successful start of AP
 */
esp_err_t cc_startAP();

/**
 * @brief Switch to STA mode
 *
 * This helper function switches to station mode
 */
void switch_to_sta(void* server);

/**
 * @brief Configure Wi-Fi, connect to an AP, wait for IP address
 *
 * This helper function connects to an AP
 *
 * @return ESP_OK on successful connect
 */
esp_err_t cc_connectToAP(char *ssid, char *pass);


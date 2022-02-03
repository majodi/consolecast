/* Common functions to establish Wi-Fi connection.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
 */

#pragma once
#include "esp_https_server.h"

#ifdef __cplusplus
extern "C" {
#endif

static const size_t max_clients = 13;

/**
 * @brief Start webserver
 *
 * This helper function starts a webserver
 *
 * @return httpd_handle_t on successful start of webserver
 */
httpd_handle_t start_webserver(void);

/**
 * @brief Stop webserver
 *
 * This helper function stops a webserver
 *
 * @param server server handle
 */
void stop_webserver(httpd_handle_t server);

/**
 * @brief wss open connection handler
 *
 * This helper function adds the connection to the keep-alive table
 *
 * @param hd server handle
 * @param sockfd file descriptor
 *
 * @return httpd_handle_t on successful start of webserver
 */
esp_err_t wss_open_fd(httpd_handle_t hd, int sockfd);

/**
 * @brief wss close connection handler
 *
 * This helper function adds the connection to the keep-alive table
 *
 * @param hd server handle
 * @param sockfd file descriptor
 *
 * @return httpd_handle_t on successful start of webserver
 */
void wss_close_fd(httpd_handle_t hd, int sockfd);

#ifdef __cplusplus
}
#endif

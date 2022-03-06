/* WebSocket handler

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#pragma once

#include "esp_http_server.h"
#include "esp_wifi_types.h"
#include "cc_wifi.h"


#define USB_HOST_PRIORITY 20
#define CCPORT_NONE -1
#define CCPORT_UART 0
#define CCPORT_CDC 1


extern const httpd_uri_t ws;

extern int cc_port;                                         // ConsoleCast Port to use. 0 = UART, 1 = CDC, -1 = not set
extern bool switchSTA;                                      // should switch to Station mode (set to True will attempt to connect to an Access Point)

extern wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];    // holds the scanned AP's
extern uint16_t ap_count;                                   // AP's found after scan

extern char connectToSSID[32];                              // SSID to connect to in Station mode
extern char connectToPass[64];                              // PW


void init_power_direction();
void set_power_direction(uint32_t VBUS_level);
esp_err_t ws_handler(httpd_req_t *req);
esp_err_t send_pkt_to_all(uint8_t *message, size_t len);

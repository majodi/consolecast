/* WebSocket handler

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "cc_websocket.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include <esp_log.h>
#include "cc_webserver.h"
#include "cc_uart.h"
#include "cc_cdc_acm.h"
#include "cc_indicator.h"
#include "keep_alive.h"


#define ECHO (0)
#define VBUS_R GPIO_NUM_8
#define VBUS_IN 0
#define VBUS_OUT 1


static const char *TAG = "cc_websocket";

uint16_t ap_count = 0;                                          // AP's found after scan
wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
char connectToSSID[32] = "";                                    // SSID to connect to in Station mode
char connectToPass[64] = "";                                    // PW


const httpd_uri_t ws = {
    .uri        = "/ws",
    .method     = HTTP_GET,
    .handler    = ws_handler,
    .user_ctx   = NULL,
    .is_websocket = true,
    .handle_ws_control_frames = true
};

int cc_port = CCPORT_NONE;                                      // port to use not set
bool switchSTA = false;                                         // should switch to Station mode (set to True will attempt to connect to an Access Point)


static void send_pkt_async(void *arg)
{
    struct async_resp_arg *resp_arg = arg;
    httpd_handle_t hd = resp_arg->hd;
    int fd = resp_arg->fd;
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = resp_arg->payload;
    ws_pkt.len = resp_arg->len;
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    httpd_ws_send_frame_async(hd, fd, &ws_pkt);
    free(resp_arg->payload);
    free(resp_arg);
}

static void send_pkt_to_fd(int fd, uint8_t *message, size_t len)
{
    httpd_handle_t handle = globserver;
    struct async_resp_arg *resp_arg = malloc(sizeof(struct async_resp_arg));
    uint8_t *payload = calloc(1, len+1);
    memcpy(payload, message, len);
    resp_arg->hd = handle;
    resp_arg->fd = fd;
    resp_arg->payload = payload;
    resp_arg->len = len;
    if (httpd_queue_work(resp_arg->hd, send_pkt_async, resp_arg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_queue_work failed!");
    }
}

esp_err_t send_pkt_to_all(uint8_t *message, size_t len)
{
    httpd_handle_t handle = globserver;
    size_t clients = max_clients;
    int    client_fds[max_clients];
    if (httpd_get_client_list(handle, &clients, client_fds) == ESP_OK) {
        for (size_t i=0; i < clients; ++i) {
            int sock = client_fds[i];
            if (httpd_ws_get_fd_info(handle, sock) == HTTPD_WS_CLIENT_WEBSOCKET) {
                struct async_resp_arg *resp_arg = malloc(sizeof(struct async_resp_arg));
                uint8_t *payload = calloc(1, len+1);
                memcpy(payload, message, len);
                resp_arg->hd = handle;
                resp_arg->fd = sock;
                resp_arg->payload = payload;
                resp_arg->len = len;
                if (httpd_queue_work(resp_arg->hd, send_pkt_async, resp_arg) != ESP_OK) {
                    ESP_LOGE(TAG, "httpd_queue_work failed!");
                }
            }
        }
    } else {
        ESP_LOGE(TAG, "send to all httpd_get_client_list failed!");
        return ESP_FAIL;
    }
    return ESP_OK;
}

void init_power_direction()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1<<VBUS_R);
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 1;
    gpio_config(&io_conf);
    set_power_direction(VBUS_IN);
}

void set_power_direction(uint32_t VBUS_level)
{
    gpio_set_level(VBUS_R, VBUS_level);
}

// Websocket handler
esp_err_t ws_handler(httpd_req_t *req)
{
    char message[50];
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake done, new connection");
        flash(600);
        if (cc_port == CCPORT_CDC) {
            snprintf(message, 50, "§!1"); // send link is up message (CDC port set)
        } else if (cc_port == CCPORT_UART) {
            snprintf(message, 50, "§!0"); // send link is up message (RS232 port set)
        } else {
            snprintf(message, 50, "§!N"); // send link is up message (No port set yet)
        }
        send_pkt_to_fd(httpd_req_to_sockfd(req), (uint8_t *)message, strlen((char *)message));
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));

    // First receive the full ws message
    /* Set max_len = 0 to get the frame len */
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }
    if (ws_pkt.len) {
        buf = calloc(1, ws_pkt.len + 1);
        if (buf == NULL) {
            ESP_LOGE(TAG, "Failed to calloc memory for buf");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        /* Set max_len = ws_pkt.len to get the frame payload */
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }
    }

    // If it was a PONG, update the keep-alive
    if (ws_pkt.type == HTTPD_WS_TYPE_PONG) {
        ESP_LOGD(TAG, "Received PONG message");
        free(buf);
        return wss_keep_alive_client_is_active(httpd_get_global_user_ctx(req->handle),
                httpd_req_to_sockfd(req));

    // If it was a TEXT message, echo / handle
    } else if (ws_pkt.type == HTTPD_WS_TYPE_TEXT) {

        // echo for debug purposes if set
        if(ECHO) {
            send_pkt_to_all(ws_pkt.payload, ws_pkt.len);
        }

        // ESP_LOGI(TAG, "ws_pkt.len: %d", ws_pkt.len);
        // ESP_LOG_BUFFER_HEX("ws_pkt.payload", (char *)ws_pkt.payload, ws_pkt.len);

        // SSID list request
        if((ws_pkt.len > 4) && strncmp((char *)ws_pkt.payload, "§±", 4) == 0) {                                         // "get SSID list" request (pkt.len = bytes incl \0, compare uses characters and unicode chars are 2 bytes)
            if((*(ws_pkt.payload+4)-48 < ap_count) && (*(ws_pkt.payload+4)-48 < DEFAULT_SCAN_LIST_SIZE)) {              // next byte (only one digit) should be the index the request wants (ascii 48 = 0)
                snprintf(message, 50, "§±%d,%s,%d", *(ws_pkt.payload+4)-48, ap_info[*(ws_pkt.payload+4)-48].ssid, ap_info[*(ws_pkt.payload+4)-48].rssi);
                send_pkt_to_fd(httpd_req_to_sockfd(req), (uint8_t *)message, strlen((char *)message));
            }
        }

        // request to switch to station mode and connect to other AP
        else if((ws_pkt.len > 4) && strncmp((char *)ws_pkt.payload, "§¡", 4) == 0) {                                    // switch to AP
            sscanf((char *)ws_pkt.payload, "§¡%s , %s", connectToSSID, connectToPass);                                  // extract SSID/PWD
            switchSTA = true;                                                                                           // signal switch is required
        }

        // com settings
        else if((ws_pkt.len > 4) && (strncmp((char *)ws_pkt.payload, "§©", 4) == 0)) {                                  // com settings
            char _commType[2];
            char _baudrate[10];
            char _db[2];
            char _parity[2];
            char _sb[2];
            char _fc[2];
            sscanf((char *)ws_pkt.payload, "§©%[^,],%[^,],%[^,],%[^,],%[^,],%[^,]", _baudrate, _db, _parity, _sb, _fc, _commType);
            ESP_LOGI(TAG, "Baudrate: %s (%d), Databits: %s, Parity: %s, sb: %s, fc: %s", _baudrate, atoi(_baudrate), _db, _parity, _sb, _fc);
            if(strncmp(_commType, "0", 1) == 0) {                                                                   // 0 = uart
                cc_port = CCPORT_UART;                                                                              // save port choice
                set_power_direction(VBUS_IN);                                                                       // normal VBUS --> power in
                uart_config.baud_rate = atoi(_baudrate);
                uart_config.data_bits = atoi(_db)-5;
                uart_config.parity    = atoi(_parity);
                uart_config.stop_bits = atoi(_sb);
                uart_flow_ctrl = atoi(_fc);
                set_uart_comm();
                snprintf(message, 50, "RJ45 RS232\r\n");                                                            // send info
                send_pkt_to_all((uint8_t *)message, strlen((char *)message));
                snprintf(message, 50, "§®0");                                                                       // signal port choice to all
                send_pkt_to_all((uint8_t *)message, strlen((char *)message));
            } else {
                cc_port = CCPORT_CDC;                                                                               // save port choice
                set_power_direction(VBUS_OUT);                                                                      // VBUS Reversed --> power out
                cdc_acm_line_coding.dwDTERate = atoi(_baudrate);
                cdc_acm_line_coding.bDataBits = atoi(_db);
                cdc_acm_line_coding.bParityType = atoi(_parity) == 3 ? 1 : atoi(_parity);
                cdc_acm_line_coding.bCharFormat = atoi(_sb)-1;
                if(CDC_connected) {
                    set_CDC_comm();
                }
                snprintf(message, 50, "§®1");                                                                       // signal port choice to all
                send_pkt_to_all((uint8_t *)message, strlen((char *)message));
            }
        }

        // normal communication (no token detected)
        else {

            if(cc_port != CCPORT_NONE) {
                if((cc_port == CCPORT_CDC) && CDC_connected) {
                    CDC_write_bytes(ws_pkt.payload, ws_pkt.len);
                } else {
                    uart_write_bytes(UART_PORT_NUM, ws_pkt.payload, ws_pkt.len);
                }
            }

        }

        free(buf);
        return ret;
    }
    free(buf);
    return ESP_OK;
}

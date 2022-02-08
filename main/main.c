/* ConsoleCast Example Firmware

   This example code is in the Public Domain (or CC0 licensed, at your option)

   USB Host code partly by https://github.com/chegewara

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "mdns.h"
#include "cc_wifi.h"
#include "cc_webserver.h"
#include "keep_alive.h"
#include "driver/uart.h"
#include "hal/gpio_types.h"

#include "hcd.h"
#include "ctrl_pipe.h"
#include "usb_host_port.h"
#include "cdc_class.h"
#include "defines.h"

static QueueHandle_t uart1_queue;

// argument structure
struct async_resp_arg {
    httpd_handle_t hd;
    int fd;
    uint8_t *payload;
    size_t len;
};

// global
static const char *TAG = "** ccfw **";                  // tag for logging to console
static int indicator_mode = IND_OFF;                    // current indicator mode
static bool switchSTA = false;                          // should switch to Station mode (set to True will attempt to connect to an Access Point)
static bool isSTA = false;                              // did we switch to Station mode
wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];       // to hold the scanned AP's
static uint16_t ap_count = 0;                           // AP's found after scan
char connectToSSID[32];                                 // SSID to connect to in Station mode
char connectToPass[64];                                 // PW
bool useCDC = false;                                    // use USB port (default is Serial port)
bool CDCisConnected = false;                            // is there a physical connection (can we send to CDC)
bool commConnected = false;                             // communication channel is connected
httpd_handle_t globserver = NULL;                       // server handle
char _baudrate[10];                                     // Serial settings
char _db[2];
char _parity[2];
char _sb[2];
char VID[15];                                           // VID/PID of USB
char PID[15];

// CDC
uint8_t conf_num;
hcd_pipe_handle_t ctrl_pipe_hdl;
uint8_t conf_num = 0;
bool cbRegistered = false;
static bool ready = false;

static void switch_to_sta(httpd_handle_t server);                                           // forward ref, needed for dependency early on
static esp_err_t send_pkt_to_all(httpd_handle_t handle, uint8_t *message, size_t len);

//===================== CDC (USB port usage) =========================

void parse_cfg_descriptor(uint8_t* data_buffer, usb_transfer_status_t status, uint8_t len, uint8_t* conf_num);

static void utf16_to_utf8(char* in, char* out, uint8_t len)
{
    for (size_t i = 0; i < len; i++)
    {
        out[i/2] = in[i];
        i++;
    }
}

void usbh_ctrl_pipe_class_specific_cb(pipe_event_msg_t msg, usb_irp_t *irp)
{
    cdc_class_specific_ctrl_cb(irp);

    if (irp->data_buffer[0] == SET_VALUE && irp->data_buffer[1] == SET_CONTROL_LINE_STATE) {    // with this we know there is a connection
        char message[50];
        CDCisConnected = true;
        snprintf(message, 50, "USB CDC %s,%s (VID/PID)\r\n", VID, PID); // send info
        send_pkt_to_all(globserver, (uint8_t *)message, strlen((char *)message));
    }

    ready = true;
}

static void cdc_pipe_cb(pipe_event_msg_t msg, usb_irp_t *irp, void *context)
{
    ESP_LOGD("", "\t-> Pipe [%d] event: %d\n", (uint8_t)context, msg.pipe_event);

    switch (msg.pipe_event)
    {
        case HCD_PIPE_EVENT_NONE:
            break;

        case HCD_PIPE_EVENT_IRP_DONE:
            ESP_LOGD("Pipe cdc: ", "XFER status: %d, num bytes: %d, actual bytes: %d", irp->status, irp->num_bytes, irp->actual_num_bytes);
            // Data from USB connection
            send_pkt_to_all(globserver, irp->data_buffer, irp->actual_num_bytes); // send received data to all connected stations
            ready = true;
            break;

        case HCD_PIPE_EVENT_ERROR_XFER:
            ESP_LOGW("", "XFER error: %d", irp->status);
            hcd_pipe_command(msg.pipe_hdl, HCD_PIPE_CMD_RESET);
            break;
        
        case HCD_PIPE_EVENT_ERROR_STALL:
            ESP_LOGW("", "Device stalled: %s pipe, state: %d", "BULK", hcd_pipe_get_state(msg.pipe_hdl));
            hcd_pipe_command(msg.pipe_hdl, HCD_PIPE_CMD_RESET);
            break;
        
        default:
            ESP_LOGW("", "not handled pipe event: %d", msg.pipe_event);
            break;
    }
}

void usbh_get_device_desc_cb(uint8_t* data_buffer, size_t num_bytes, void* context)
{
    ESP_LOG_BUFFER_HEX_LEVEL("DEVICE descriptor", data_buffer, num_bytes, ESP_LOG_INFO);
    parse_cfg_descriptor(data_buffer, 0, num_bytes, &conf_num);

    usb_desc_devc_t* desc = (usb_desc_devc_t*)data_buffer;
    xfer_get_string(port_hdl, ctrl_pipe_hdl, desc->iManufacturer);
    xfer_get_string(port_hdl, ctrl_pipe_hdl, desc->iSerialNumber);
    xfer_get_string(port_hdl, ctrl_pipe_hdl, desc->iProduct);

    xfer_set_address(port_hdl, ctrl_pipe_hdl, DEVICE_ADDR);

    snprintf(VID, 15, "VID: 0x%04x", desc->idVendor);
    snprintf(PID, 15, "PID: 0x%04x", desc->idProduct);

}

void usbh_set_address_cb(uint16_t addr, void* context)
{
    if (ESP_OK != hcd_pipe_update_dev_addr(ctrl_pipe_hdl, DEVICE_ADDR))
        ESP_LOGE("", "failed to update device address");
    xfer_set_configuration(port_hdl, ctrl_pipe_hdl, 1);
}

void usbh_get_config_desc_cb(uint8_t* data_buffer, size_t num_bytes, void* context)
{
    parse_cfg_descriptor(data_buffer, 0, num_bytes, &conf_num);

    xfer_get_current_config(port_hdl, ctrl_pipe_hdl); 
}

void usbh_set_config_desc_cb(uint16_t data, void* context)
{
    xfer_get_desc(port_hdl, ctrl_pipe_hdl);
}

void usbh_get_string_cb(uint8_t* data, size_t num_bytes, void* context)
{
    char out[64] = {};
    utf16_to_utf8((char*)data, out, num_bytes);
    parse_cfg_descriptor(data, 0, num_bytes, &conf_num);
}

void usbh_ctrl_pipe_stalled_cb(usb_ctrl_req_t* ctrl)
{
    ESP_LOG_BUFFER_HEX_LEVEL("STALLED", ctrl, 8, ESP_LOG_WARN);
}

void usbh_ctrl_pipe_error_cb(usb_ctrl_req_t* ctrl)
{
    ESP_LOG_BUFFER_HEX_LEVEL("ERROR", ctrl, 8, ESP_LOG_WARN);
}

void usbh_get_configuration_cb(uint8_t addr, void* context)
{
    xfer_set_control_line(port_hdl, ctrl_pipe_hdl, 1, 1);
}

void usbh_port_connection_cb(port_event_msg_t msg)
{
    hcd_port_state_t state;
    ESP_LOGI("", "HCD_PORT_EVENT_CONNECTION");
    if (HCD_PORT_STATE_DISABLED == hcd_port_get_state(msg.port_hdl))
        ESP_LOGI("", "HCD_PORT_STATE_DISABLED");
    if (ESP_OK == hcd_port_command(msg.port_hdl, HCD_PORT_CMD_RESET))
        ESP_LOGI("", "USB device reset");
    else
        return;
    if (HCD_PORT_STATE_ENABLED == hcd_port_get_state(msg.port_hdl))
    {
        ESP_LOGI("", "HCD_PORT_STATE_ENABLED");
        allocate_ctrl_pipe(msg.port_hdl, &ctrl_pipe_hdl);
        xfer_get_device_desc(msg.port_hdl, ctrl_pipe_hdl);
        port_hdl = msg.port_hdl;
    }
}

void usbh_port_sudden_disconn_cb(port_event_msg_t msg)
{
    hcd_port_state_t state;
    if (HCD_PIPE_STATE_INVALID == hcd_pipe_get_state(ctrl_pipe_hdl))
    {                
        ESP_LOGW("", "pipe state: %d", hcd_pipe_get_state(ctrl_pipe_hdl));
        ready = false;
        delete_pipes();

        free_pipe_and_irp_list(ctrl_pipe_hdl);
        ctrl_pipe_hdl = NULL;

        esp_err_t err;
        if(HCD_PORT_STATE_RECOVERY == (state = hcd_port_get_state(msg.port_hdl))){
            if(ESP_OK != (err = hcd_port_recover(msg.port_hdl))) ESP_LOGE("recovery", "should be not powered state %d => (%d)", state, err);
        } else {
            ESP_LOGE("", "hcd_port_state_t: %d", state);
        }
        if(ESP_OK == hcd_port_command(msg.port_hdl, HCD_PORT_CMD_POWER_ON)) ESP_LOGI("", "Port powered ON");

        useCDC = false;
        CDCisConnected = false;
        // turn off vbus power out (electronic switch to reverse power)
        gpio_set_level(VBUS_R, 0);        

    }
}

extern void cdc_pipe_event_task(void* p);

//===================== Websocket and webserver =============================

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

static void send_pkt_to_fd(httpd_handle_t handle, int fd, uint8_t *message, size_t len)
{
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

static esp_err_t send_pkt_to_all(httpd_handle_t handle, uint8_t *message, size_t len)
{
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

//===================== HTTP GET handlers (served files are embedded in the firmware) =====================

static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    extern const char index_html_start[] asm("_binary_index_html_start");
    extern const char index_html_end[]   asm("_binary_index_html_end");
    httpd_resp_send(req, index_html_start, index_html_end - index_html_start);
    return ESP_OK;
}

static esp_err_t favicon_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "image/x-icon");
    extern const char favicon_ico_start[] asm("_binary_favicon_ico_start");
    extern const char favicon_ico_end[]   asm("_binary_favicon_ico_end");
    httpd_resp_send(req, favicon_ico_start, favicon_ico_end - favicon_ico_start);
    return ESP_OK;
}

static esp_err_t ccpng_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "image/png");
    extern const char ccpng_start[] asm("_binary_cc_png_start");
    extern const char ccpng_end[]   asm("_binary_cc_png_end");
    httpd_resp_send(req, ccpng_start, ccpng_end - ccpng_start);
    return ESP_OK;
}

static esp_err_t xterm_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/javascript; charset=UTF-8");
    extern const char xterm_js_start[] asm("_binary_xterm_js_start");
    extern const char xterm_js_end[]   asm("_binary_xterm_js_end");
    httpd_resp_send(req, xterm_js_start, xterm_js_end - xterm_js_start);
    return ESP_OK;
}

static const httpd_uri_t root = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler
};

static const httpd_uri_t favicon = {
    .uri       = "/favicon.ico",
    .method    = HTTP_GET,
    .handler   = favicon_get_handler
};

static const httpd_uri_t ccpng = {
    .uri       = "/cc.png",
    .method    = HTTP_GET,
    .handler   = ccpng_get_handler
};

static const httpd_uri_t xterm = {
    .uri       = "/xterm.js",
    .method    = HTTP_GET,
    .handler   = xterm_get_handler
};

// Websocket handler
static esp_err_t ws_handler(httpd_req_t *req)
{
    char message[50];
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake done, new connection");
        gpio_set_level(IO_TXD, 1);
        gpio_set_level(IO_RXD, 1);
        vTaskDelay(600 / portTICK_PERIOD_MS);
        gpio_set_level(IO_TXD, 0);
        gpio_set_level(IO_RXD, 0);

        if (commConnected) {
            if (useCDC) {
                snprintf(message, 50, "§!1"); // send link is up message (CDC)
            } else {
                snprintf(message, 50, "§!0"); // send link is up message (RS232)
            }
        } else {
            snprintf(message, 50, "§!N"); // send link is up message (None)
        }
        // snprintf(message, 50, "§!"); // send link is up message
        send_pkt_to_fd(req->handle, httpd_req_to_sockfd(req), (uint8_t *)message, strlen((char *)message));
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
        if(ECHO) {
            send_pkt_to_all(req->handle, ws_pkt.payload, ws_pkt.len);
        }
        if((ws_pkt.len > 4) && strncmp((char *)ws_pkt.payload, "§±", 4) == 0) {                                         // "get SSID list" request (pkt.len = bytes incl \0, compare uses characters and unicode chars are 2 bytes)
            if((*(ws_pkt.payload+4)-48 < ap_count) && (*(ws_pkt.payload+4)-48 < DEFAULT_SCAN_LIST_SIZE)) {              // next byte (only one digit) should be the index the request wants (ascii 48 = 0)
                snprintf(message, 50, "§±%d,%s,%d", *(ws_pkt.payload+4)-48, ap_info[*(ws_pkt.payload+4)-48].ssid, ap_info[*(ws_pkt.payload+4)-48].rssi);
                send_pkt_to_fd(req->handle, httpd_req_to_sockfd(req), (uint8_t *)message, strlen((char *)message));
            }
        }
        else if((ws_pkt.len > 4) && strncmp((char *)ws_pkt.payload, "§¨", 4) == 0) {                                    // switch to AP
            sscanf((char *)ws_pkt.payload, "§¨%s , %s", connectToSSID, connectToPass);
            switchSTA = true;
        }
        else if((ws_pkt.len > 4) && (strncmp((char *)ws_pkt.payload, "§©", 4) == 0)) {                                  // comm settings
            if (commConnected == false) {
                char _fc[2];
                char _commType[2];
                sscanf((char *)ws_pkt.payload, "§©%[^,],%[^,],%[^,],%[^,],%[^,],%[^,]", _baudrate, _db, _parity, _sb, _fc, _commType);
                ESP_LOGI(TAG, "Baudrate: %s (%d), Databits: %s, Parity: %s, sb: %s, fc: %s", _baudrate, atoi(_baudrate), _db, _parity, _sb, _fc);
                if(strncmp(_commType, "0", 1) == 0) {       // 0 = uart
                    useCDC = false;
                    gpio_set_level(VBUS_R, 0);
                    ESP_ERROR_CHECK(uart_set_baudrate(UART_PORT_NUM, atoi(_baudrate)));
                    ESP_ERROR_CHECK(uart_set_word_length(UART_PORT_NUM, atoi(_db)-5));
                    ESP_ERROR_CHECK(uart_set_parity(UART_PORT_NUM, atoi(_parity)));
                    ESP_ERROR_CHECK(uart_set_stop_bits(UART_PORT_NUM, atoi(_sb)));
                    if(strncmp(_fc, "1", 1) == 0) {
                        ESP_ERROR_CHECK(uart_set_hw_flow_ctrl(UART_PORT_NUM, UART_HW_FLOWCTRL_CTS_RTS, 120));
                    }
                    else if(strncmp(_fc, "2", 1) == 0) {
                        ESP_ERROR_CHECK(uart_set_sw_flow_ctrl(UART_PORT_NUM, true, 8, 120));
                    } else {
                        ESP_ERROR_CHECK(uart_set_sw_flow_ctrl(UART_PORT_NUM, false, 8, 120));
                        ESP_ERROR_CHECK(uart_set_hw_flow_ctrl(UART_PORT_NUM, UART_HW_FLOWCTRL_DISABLE, 120));
                    }
                    snprintf(message, 50, "RJ45 RS232\r\n"); // send info
                    send_pkt_to_all(globserver, (uint8_t *)message, strlen((char *)message));
                    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, IO_TXD, IO_RXD, IO_RTS, IO_CTS));

                    snprintf(message, 50, "§®0");
                    send_pkt_to_all(globserver, (uint8_t *)message, strlen((char *)message));

                } else {
                    useCDC = true;
                    // turn on vbus power out (electronic switch to reverse power)
                    gpio_set_level(VBUS_R, 1);

                    snprintf(message, 50, "§®1");
                    send_pkt_to_all(globserver, (uint8_t *)message, strlen((char *)message));

                }
                commConnected = true;
            }
        }
        else {
            if (useCDC && CDCisConnected) {
                // to CDC
                xfer_out_data(ws_pkt.payload, ws_pkt.len);
            } else {
                // to UART
                uart_write_bytes(UART_PORT_NUM, ws_pkt.payload, ws_pkt.len);
            }
        }

        free(buf);
        return ret;
    }
    free(buf);
    return ESP_OK;
}

static const httpd_uri_t ws = {
        .uri        = "/ws",
        .method     = HTTP_GET,
        .handler    = ws_handler,
        .user_ctx   = NULL,
        .is_websocket = true,
        .handle_ws_control_frames = true
};

static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        stop_webserver(*server);
        *server = NULL;
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        *server = start_webserver();
        globserver = *server;
        httpd_register_uri_handler(*server, &root);
        httpd_register_uri_handler(*server, &favicon);
        httpd_register_uri_handler(*server, &ccpng);
        httpd_register_uri_handler(*server, &xterm);
        httpd_register_uri_handler(*server, &ws);
    }
}

// static void switch_to_sta(httpd_handle_t server) {
static void switch_to_sta(void* server) {
    indicator_mode = IND_WORKING;
    ESP_LOGI(TAG, "----> switch to STA");
    ESP_ERROR_CHECK(esp_wifi_stop());

    esp_event_handler_instance_unregister(ESP_EVENT_ANY_BASE, ESP_EVENT_ANY_ID, NULL);
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, server, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, server, NULL));

    ESP_ERROR_CHECK(cc_connectToAP(connectToSSID, connectToPass));
    ESP_LOGI(TAG, "----> end switch to STA");
}

// LED's can be used only when UART communication is inactive (the RX/TX LEDs are used)
static void led_indicator_task(void *arg)
{
    gpio_reset_pin(IO_RXD);
    gpio_set_direction(IO_RXD, GPIO_MODE_OUTPUT);
    gpio_reset_pin(IO_TXD);
    gpio_set_direction(IO_TXD, GPIO_MODE_OUTPUT);
    static uint8_t rx_state = 0;
    static uint8_t tx_state = 0;
    static int period = 500;
    while(1) {
        switch (indicator_mode)
        {
        case IND_OFF:                       // 0 = off (both)
            if(period != 500) {
                rx_state = 1;
                tx_state = 1;
                period = 500;
            }
            break;
        case IND_ON:                        // 1 = on (both)
            if(period != 501) {
                rx_state = 0;
                tx_state = 0;
                period = 501;
            }
            break;
        case IND_WORKING:                   // 2 = alternating
            if(period != 200) {
                rx_state = 0;
                tx_state = 1;
                period = 200;
            }
            rx_state = !rx_state;
            tx_state = !tx_state;
            break;
        case IND_ERROR:                     // 3 = flashing
            if(period != 201) {
                rx_state = 0;
                tx_state = 0;
                period = 201;
            }
            rx_state = !rx_state;
            tx_state = !tx_state;
            break;
        
        default:
            break;
        }
        gpio_set_level(IO_RXD, rx_state);
        gpio_set_level(IO_TXD, tx_state);
        vTaskDelay(period / portTICK_PERIOD_MS);
    }
}

//===================== UART event task (Serial port usage) =========================

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(UART_BUF_SIZE);
    int UImessageSize = 35;
    uint8_t* UImessage = (uint8_t*) calloc(1, UImessageSize+1);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart1_queue, (void * )&event, (portTickType)portMAX_DELAY) && !useCDC) {
            bzero(dtmp, UART_BUF_SIZE);
            switch(event.type) {
                //Event of UART receving data
                case UART_DATA:
                    uart_read_bytes(UART_PORT_NUM, dtmp, event.size, portMAX_DELAY);
                    send_pkt_to_all(globserver, dtmp, event.size);
                    memset(UImessage, 0, UImessageSize);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    uart_flush_input(UART_PORT_NUM);
                    xQueueReset(uart1_queue);
                    strncpy((char*)UImessage, "\r\n** hw fifo overflow **\r\n", UImessageSize);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    uart_flush_input(UART_PORT_NUM);
                    xQueueReset(uart1_queue);
                    strncpy((char*)UImessage, "\r\n** ring buffer full **\r\n", UImessageSize);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    strncpy((char*)UImessage, "\r\n** uart rx break **\r\n", UImessageSize);
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    strncpy((char*)UImessage, "\r\n** uart parity error **\r\n", UImessageSize);
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    strncpy((char*)UImessage, "\r\n** uart frame error **\r\n", UImessageSize);
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
            if(strlen((char*)UImessage) > 0) {
                send_pkt_to_all(globserver, UImessage, strlen((char*)UImessage));
            }
        }
    }
    free(UImessage);
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

//===================== CDC task (USB port usage) =========================

static void cdc_listen_task(void *pvParameters)
{
    while(1) {
        vTaskDelay(10);
        if(ready && useCDC){
            ready = false;
            xfer_in_data();
        }
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    static httpd_handle_t server = NULL;

    if (xTaskCreate(led_indicator_task, "led_indicator_task", TASK_STACK_SIZE, NULL, 2, NULL) != pdTRUE) {
        ESP_LOGE(TAG, "could not start led indicator task");
    }

    indicator_mode = IND_WORKING;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // AP mode
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_AP_START, &connect_handler, &server, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_AP_STOP, &disconnect_handler, &server, NULL));
    ESP_ERROR_CHECK(cc_startAP());

    // scan APs
    memset(ap_info, 0, sizeof(ap_info));
    ESP_ERROR_CHECK(esp_wifi_scan_start(NULL, true));
    uint16_t list_size = DEFAULT_SCAN_LIST_SIZE;
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&list_size, ap_info));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
    ESP_LOGI(TAG, "Total APs scanned = %u", ap_count);
    for (int i = 0; (i < DEFAULT_SCAN_LIST_SIZE) && (i < ap_count); i++) {
        ESP_LOGI(TAG, "SSID \t\t%s\t%d", ap_info[i].ssid, ap_info[i].rssi);
    }

    // mdns
    ESP_ERROR_CHECK( mdns_init() );
    ESP_ERROR_CHECK( mdns_hostname_set("cc") );
    ESP_ERROR_CHECK( mdns_instance_name_set("ConsoleCast") );

    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 20, &uart1_queue, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    // ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, IO_TXD, IO_RXD, IO_RTS, IO_CTS));

    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 5, NULL);

    xTaskCreate(ctrl_pipe_event_task, "pipe_task", 4*1024, NULL, 5, NULL);
    xTaskCreate(cdc_pipe_event_task, "pipe_task", 4*1024, NULL, 5, NULL);
    xTaskCreate(cdc_listen_task, "pipe_task", 4*1024, NULL, 5, NULL);

    printf("USB host initialized\n");
    if(setup_usb_host()){
        if (!cbRegistered) {
            register_cdc_pipe_callback(cdc_pipe_cb);
            cbRegistered = true;
        }
    }

    indicator_mode = IND_OFF;
    printf("Heap free after init %d (%d).\n", heap_caps_get_free_size(MALLOC_CAP_8BIT), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1<<VBUS_R);
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 1;
    gpio_config(&io_conf);
    gpio_set_level(VBUS_R, 0);  // default: VBUS power incoming

    while(1) { // check for station-mode request now and then
        vTaskDelay(1500 / portTICK_PERIOD_MS);
        if(switchSTA && !isSTA) {
            isSTA = true;
            switch_to_sta(&server);
        }
    }

}

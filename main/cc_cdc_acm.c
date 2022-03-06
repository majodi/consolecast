/* USB_lib_task and handlers

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "cc_cdc_acm.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>
#include "usb/usb_host.h"
#include "cc_websocket.h"


bool CDC_connected = false;
cdc_acm_dev_hdl_t cdc_dev;


cdc_acm_line_coding_t cdc_acm_line_coding = {
    .dwDTERate = 9600,
    .bDataBits = 8,
    .bParityType = 0,
    .bCharFormat = 0
};

static const cdc_acm_host_device_config_t dev_config = {
    .connection_timeout_ms = 1000,
    .out_buffer_size = 64,
    .user_arg = NULL,
    .event_cb = handle_notify,
    .data_cb = handle_rx
};

// same as uart (ESP32S2 only has 1 core)
static const cdc_acm_host_driver_config_t cdc_acm_driver_config_default = {
    .driver_task_stack_size = 4096,
    .driver_task_priority = 10,
    .xCoreID = 0
};

static const char *TAG = "cc_cdc_acm";

void handle_rx(uint8_t *data, size_t data_len, void *arg)
{
    send_pkt_to_all(data, data_len);
}

void handle_notify(cdc_acm_dev_hdl_t cdc_hdl, const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    ESP_LOGI(TAG, "Event of type: %d\n", event->type);
    if(event->type == CDC_ACM_HOST_SERIAL_STATE) {
        CDC_connected = true;
        ESP_LOGI(TAG, "*** CDC Conected ***");
    }
    if(event->type == CDC_ACM_HOST_ERROR) {
        ESP_LOGI(TAG, "*** CDC Host Error ***");
    }
    if(event->type == CDC_ACM_HOST_DEVICE_DISCONNECTED) {
        cdc_acm_host_close(cdc_hdl);
        CDC_connected = false;
        ESP_LOGI(TAG, "*** CDC Disconected ***");
    }
}

static void usb_lib_task(void *arg)
{
    while (1) {
        //Start handling system events
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_LOGI(TAG, "All clients deregistered");
            ESP_ERROR_CHECK(usb_host_device_free_all());
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "All free");
            // break;
        }
    }

    //Clean up USB Host
    ESP_ERROR_CHECK(usb_host_uninstall());
    vTaskDelete(NULL);
}

void init_cdc_acm()
{
    ESP_LOGI(TAG, "Installing USB Host");
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    // Create a task that will handle USB library events
    xTaskCreate(usb_lib_task, "usb_lib", 4096, xTaskGetCurrentTaskHandle(), USB_HOST_PRIORITY, NULL);

    ESP_LOGI(TAG, "Installing CDC-ACM driver");
    ESP_ERROR_CHECK(cdc_acm_host_install(&cdc_acm_driver_config_default));
}

void set_CDC_comm() {
    char message[50];
    ESP_ERROR_CHECK(cdc_acm_host_line_coding_set(cdc_dev, &cdc_acm_line_coding));
    ESP_LOGI(TAG, "Line Set: Rate: %d, Stop bits: %d, Parity: %d, Databits: %d", cdc_acm_line_coding.dwDTERate,
                cdc_acm_line_coding.bCharFormat, cdc_acm_line_coding.bParityType, cdc_acm_line_coding.bDataBits);
    snprintf(message, 50, "USB CDC/ACM\r\n");                                                            // send info
    send_pkt_to_all((uint8_t *)message, strlen((char *)message));
    vTaskDelay(100);
}

void CDC_try_open() {
    ESP_LOGI(TAG, "Trying to open CDC ACM device");
    cdc_acm_host_open(0, 0, 0, &dev_config, &cdc_dev);
    if(cdc_dev) {
        cdc_acm_host_desc_print(cdc_dev);
        vTaskDelay(100);
        set_CDC_comm();
    }
}

void CDC_write_bytes(const uint8_t *data, size_t data_len) {
    // ESP_LOGI(TAG, "Data sent to external device");
    // ESP_LOG_BUFFER_HEXDUMP(TAG, data, data_len, ESP_LOG_INFO);
    ESP_ERROR_CHECK(cdc_acm_host_data_tx_blocking(cdc_dev, data, data_len, 1000));
}
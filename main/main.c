/* ConsoleCast Example Firmware by NickStick BV, 2022

   This example code is in the Public Domain (or CC0 licensed, at your option)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_event.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "mdns.h"

#include "cc_wifi.h"
#include "cc_webserver.h"
#include "cc_websocket.h"
#include "cc_indicator.h"
#include "cc_uart.h"
#include "cc_cdc_acm.h"


static const char *TAG = "** ccfw **";                  // tag for logging to console

void app_main(void)
{
    if (xTaskCreate(led_indicator_task, "led_indicator_task", 2048, NULL, tskIDLE_PRIORITY+1, NULL) != pdTRUE) {
        ESP_LOGE(TAG, "could not start led indicator task");
    }
    indicator_mode = IND_WORKING;
    init_power_direction();
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    static httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_AP_START, &connect_handler, &server, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_AP_STOP, &disconnect_handler, &server, NULL));
    ESP_ERROR_CHECK(cc_startAP());
    ESP_ERROR_CHECK( mdns_init() );
    ESP_ERROR_CHECK( mdns_hostname_set("cc") );
    ESP_ERROR_CHECK( mdns_instance_name_set("ConsoleCast") );
    init_uart();
    init_cdc_acm();
    printf("Heap free after init %d (%d).\n", heap_caps_get_free_size(MALLOC_CAP_8BIT), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    indicator_mode = IND_OFF;

    while(1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if(switchSTA && !isSTA) {
            switch_to_sta(&server);
        }
        if((cc_port == CCPORT_CDC) && !CDC_connected) {
            CDC_try_open();
        }
    }
}

/* UART functionality

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "cc_uart.h"

#include <esp_log.h>
#include "cc_websocket.h"


#define UART_DEFAULT_BAUD_RATE  (9600)
#define UART_BUF_SIZE   (512)

// same as cdc_acm (ESP32S2 only has 1 core)
#define UART_EVENT_TASK_PRIO 10
#define UART_EVENT_TASK_CORE 0


uart_config_t uart_config = {
    .baud_rate = UART_DEFAULT_BAUD_RATE,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB,
};
int uart_flow_ctrl = 0;

static const char *TAG = "cc_uart";
static QueueHandle_t uart1_queue;


static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(UART_BUF_SIZE);
    int UImessageSize = 35;
    uint8_t* UImessage = (uint8_t*) calloc(1, UImessageSize+1);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart1_queue, (void * )&event, (TickType_t)portMAX_DELAY) && (cc_port == 0)) {
            bzero(dtmp, UART_BUF_SIZE);
            switch(event.type) {
                //Event of UART receving data
                case UART_DATA:
                    uart_read_bytes(UART_PORT_NUM, dtmp, event.size, portMAX_DELAY);
                    send_pkt_to_all(dtmp, event.size);
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
            //Forward received to connected terminals
            if(strlen((char*)UImessage) > 0) {
                send_pkt_to_all(UImessage, strlen((char*)UImessage));
            }
        }
    }
    free(UImessage);
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void init_uart()
{
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 20, &uart1_queue, intr_alloc_flags));
    set_uart_comm();

    xTaskCreatePinnedToCore(uart_event_task, "uart_event_task", 2048, NULL, UART_EVENT_TASK_PRIO, NULL, UART_EVENT_TASK_CORE); //xTaskCreatePinnedToCore
}

void set_uart_comm()
{
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    if(uart_flow_ctrl == 1) {
        ESP_ERROR_CHECK(uart_set_hw_flow_ctrl(UART_PORT_NUM, UART_HW_FLOWCTRL_CTS_RTS, 120));
    }
    else if(uart_flow_ctrl == 2) {
        ESP_ERROR_CHECK(uart_set_sw_flow_ctrl(UART_PORT_NUM, true, 8, 120));
    } else {
        ESP_ERROR_CHECK(uart_set_sw_flow_ctrl(UART_PORT_NUM, false, 8, 120));
        ESP_ERROR_CHECK(uart_set_hw_flow_ctrl(UART_PORT_NUM, UART_HW_FLOWCTRL_DISABLE, 120));
    }
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, IO_TXD, IO_RXD, IO_RTS, IO_CTS));
}
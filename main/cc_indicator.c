/* Indicator facility for blinking LEDs

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
 */

#include "cc_indicator.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cc_uart.h"


int indicator_mode = 0;

// LED's can be used only when UART communication is inactive (the RX/TX LEDs are used)
void led_indicator_task(void *arg)
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

void flash(int duration)
{
    gpio_reset_pin(IO_RXD);
    gpio_set_direction(IO_RXD, GPIO_MODE_OUTPUT);
    gpio_reset_pin(IO_TXD);
    gpio_set_direction(IO_TXD, GPIO_MODE_OUTPUT);

    gpio_set_level(IO_TXD, 1);
    gpio_set_level(IO_RXD, 1);
    vTaskDelay(duration / portTICK_PERIOD_MS);
    gpio_set_level(IO_TXD, 0);
    gpio_set_level(IO_RXD, 0);

    set_uart_comm();
}

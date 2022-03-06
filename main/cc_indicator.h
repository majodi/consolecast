/* Indicator facility for blinking LEDs

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#pragma once

// indicator mode (flashing the LED's)
#define IND_OFF      (0)
#define IND_ON       (1)
#define IND_WORKING  (2)
#define IND_ERROR    (3)

extern int indicator_mode;                           // current indicator mode


/**
 * @brief LED indicators task
 *
 * this will act on indicator_mode, flashing the Rx/Tx LEDs
 *
 */
void led_indicator_task(void *arg);

/**
 * @brief LED on/off for short period
 *
 * flashing the Rx/Tx LEDs for a brief moment
 *
 * @param duration on duration in ms
 */
void flash(int duration);

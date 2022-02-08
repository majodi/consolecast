#pragma once

// UART pins
#define IO_TXD (4)
#define IO_RXD (5)
#define IO_RTS (6)
#define IO_CTS (7)

// indicator mode (flashing the LED's)
#define IND_OFF (0)
#define IND_ON  (1)
#define IND_WORKING (2)
#define IND_ERROR   (3)

// default UART settings
#define UART_PORT_NUM   (1)
#define UART_BAUD_RATE  (9600)
#define TASK_STACK_SIZE (2048)
#define UART_BUF_SIZE   (512)
#define ECHO (0)

// reverse VBUS pin (to signal host mode power path should be used)
#define VBUS_R GPIO_NUM_8

// code assumes only 1 digit (0-9) --> max setting here for scan list size is 10
#define DEFAULT_SCAN_LIST_SIZE (10)

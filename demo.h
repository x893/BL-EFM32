#ifndef __DEMO_H__
#define __DEMO_H__

#include <stdint.h>
#include "em_gpio.h"

#define GPIOA	gpio->P[0]
#define GPIOB	gpio->P[1]
#define GPIOC	gpio->P[2]
#define GPIOD	gpio->P[3]
#define GPIOE	gpio->P[4]

// #define BUTTON		gpioPortE, 0
#define LEDx_PORT	gpioPortE
#define LED1		LEDx_PORT, 1
#define LED2		LEDx_PORT, 2

#define HOST_TX		gpioPortD, 4
#define HOST_RX		gpioPortD, 5

#define BUTTON_SETTLE_TIME	10

#define RTC_FREQ    		32768

struct AppHeader_s {
	uint32_t * Base;
	uint32_t Limit;
	uint32_t Version;
	uint16_t CRC16;
};
typedef struct AppHeader_s AppHeader_t;

#endif

#ifndef RTC_H
#define RTC_H

#include <stdint.h>
#include "gpio.h"

// DS1302 Pin Definitions
#define RTC_CLK_PORT    GPIO_PORT_D
#define RTC_CLK_PIN     2              // Digital Pin 2

#define RTC_DAT_PORT    GPIO_PORT_D
#define RTC_DAT_PIN     3              // Digital Pin 3

#define RTC_RST_PORT    GPIO_PORT_D
#define RTC_RST_PIN     4              // Digital Pin 4

// DS1302 Register Addresses
#define DS1302_SECONDS      0x80
#define DS1302_MINUTES      0x82
#define DS1302_HOURS        0x84
#define DS1302_DATE         0x86
#define DS1302_MONTH        0x88
#define DS1302_DAY          0x8A
#define DS1302_YEAR         0x8C
#define DS1302_WP           0x8E

// Time structure
typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t day;
    uint8_t date;
    uint8_t month;
    uint8_t year;
} RTCTime_t;

// Function prototypes
void rtc_init(void);
void rtc_set_time(RTCTime_t *time);
uint8_t rtc_get_time(RTCTime_t *time);  // Returns 0 on success, 1 on error
uint8_t rtc_is_scheduled_time(RTCTime_t *time);
uint8_t bcd_to_decimal(uint8_t bcd);
uint8_t decimal_to_bcd(uint8_t decimal);

#endif
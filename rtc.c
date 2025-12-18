#include "rtc.h"
#include "gpio.h"
#include <util/delay.h>

// BCD conversion functions
uint8_t bcd_to_decimal(uint8_t bcd)
{
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

uint8_t decimal_to_bcd(uint8_t decimal)
{
    return ((decimal / 10) << 4) | (decimal % 10);
}

// DS1302 Low-level functions
static void ds1302_rst_high(void)
{
    gpio_write(RTC_RST_PORT, RTC_RST_PIN, GPIO_PIN_HIGH);
}

static void ds1302_rst_low(void)
{
    gpio_write(RTC_RST_PORT, RTC_RST_PIN, GPIO_PIN_LOW);
}

static void ds1302_clk_high(void)
{
    gpio_write(RTC_CLK_PORT, RTC_CLK_PIN, GPIO_PIN_HIGH);
}

static void ds1302_clk_low(void)
{
    gpio_write(RTC_CLK_PORT, RTC_CLK_PIN, GPIO_PIN_LOW);
}

static void ds1302_dat_high(void)
{
    gpio_write(RTC_DAT_PORT, RTC_DAT_PIN, GPIO_PIN_HIGH);
}

static void ds1302_dat_low(void)
{
    gpio_write(RTC_DAT_PORT, RTC_DAT_PIN, GPIO_PIN_LOW);
}

static void ds1302_dat_output(void)
{
    gpio_set_direction(RTC_DAT_PORT, RTC_DAT_PIN, GPIO_PIN_OUTPUT);
}

static void ds1302_dat_input(void)
{
    gpio_set_direction(RTC_DAT_PORT, RTC_DAT_PIN, GPIO_PIN_INPUT);
}

static uint8_t ds1302_dat_read(void)
{
    return gpio_read(RTC_DAT_PORT, RTC_DAT_PIN);
}

// Write a byte to DS1302 (LSB first)
static void ds1302_write_byte(uint8_t byte)
{
    ds1302_dat_output();

    for (uint8_t i = 0; i < 8; i++)
    {
        ds1302_clk_low();

        // Write LSB first
        if (byte & 0x01)
        {
            ds1302_dat_high();
        }
        else
        {
            ds1302_dat_low();
        }

        _delay_us(2); // Increased delay
        ds1302_clk_high();
        _delay_us(2); // Increased delay

        byte >>= 1;
    }

    ds1302_clk_low(); // Leave clock low after writing
}
static uint8_t ds1302_read_byte(void)
{
    uint8_t byte = 0;

    ds1302_dat_input();
    gpio_set_pullup(RTC_DAT_PORT, RTC_DAT_PIN, 1); // Enable pull-up

    for (uint8_t i = 0; i < 8; i++)
    {
        ds1302_clk_low(); // Clock low first
        _delay_us(2);

        // NOW read the data (after clock is low and data is stable)
        if (ds1302_dat_read())
        {
            byte |= (1 << i);
        }

        ds1302_clk_high(); // Clock high to prepare for next bit
        _delay_us(2);
    }

    return byte;
}

// Write to DS1302 register
static void ds1302_write_register(uint8_t reg, uint8_t value)
{
    ds1302_rst_low();
    ds1302_clk_low();
    _delay_us(4); // Increased from 2 to 4

    ds1302_rst_high();
    _delay_us(4); // Increased from 2 to 4 - critical for RST setup time

    ds1302_write_byte(reg);
    ds1302_write_byte(value);

    ds1302_rst_low();
    _delay_us(4); // Increased from 2 to 4
}

// Read from DS1302 register
static uint8_t ds1302_read_register(uint8_t reg)
{
    uint8_t value;

    ds1302_rst_low();
    ds1302_clk_low();
    _delay_us(4); // Increased from 2 to 4

    ds1302_rst_high();
    _delay_us(4); // Increased from 2 to 4 - critical for RST setup time

    ds1302_write_byte(reg | 0x01); // Set read bit
    value = ds1302_read_byte();

    ds1302_rst_low();
    _delay_us(4); // Increased from 2 to 4

    return value;
}

// Initialize RTC
void rtc_init(void)
{
    // Set pins as outputs
    gpio_set_direction(RTC_CLK_PORT, RTC_CLK_PIN, GPIO_PIN_OUTPUT);
    gpio_set_direction(RTC_DAT_PORT, RTC_DAT_PIN, GPIO_PIN_OUTPUT);
    gpio_set_direction(RTC_RST_PORT, RTC_RST_PIN, GPIO_PIN_OUTPUT);

    ds1302_rst_low();
    ds1302_clk_low();
    ds1302_dat_low();

    _delay_ms(50);

    // Disable write protection
    ds1302_write_register(DS1302_WP, 0x00);
    _delay_ms(10);

    // Check if oscillator is running
    uint8_t sec = ds1302_read_register(DS1302_SECONDS);

    // If CH bit is set (bit 7), oscillator is stopped
    if (sec & 0x80)
    {
        // Clear CH bit to start oscillator
        ds1302_write_register(DS1302_SECONDS, sec & 0x7F);
        _delay_ms(10);
    }
}

// Set time
void rtc_set_time(RTCTime_t *time)
{
    // Disable write protection
    ds1302_write_register(DS1302_WP, 0x00);
    _delay_ms(5);

    // Write time registers
    ds1302_write_register(DS1302_SECONDS, decimal_to_bcd(time->seconds) & 0x7F);
    ds1302_write_register(DS1302_MINUTES, decimal_to_bcd(time->minutes));
    ds1302_write_register(DS1302_HOURS, decimal_to_bcd(time->hours));
    ds1302_write_register(DS1302_DATE, decimal_to_bcd(time->date));
    ds1302_write_register(DS1302_MONTH, decimal_to_bcd(time->month));
    ds1302_write_register(DS1302_DAY, decimal_to_bcd(time->day));
    ds1302_write_register(DS1302_YEAR, decimal_to_bcd(time->year));

    _delay_ms(5);

    // Re-enable write protection
    ds1302_write_register(DS1302_WP, 0x80);
}

// Get time - returns 0 on success, 1 on error
uint8_t rtc_get_time(RTCTime_t *time)
{
    // Read all registers
    uint8_t sec_bcd = ds1302_read_register(DS1302_SECONDS);
    uint8_t min_bcd = ds1302_read_register(DS1302_MINUTES);
    uint8_t hour_bcd = ds1302_read_register(DS1302_HOURS);
    uint8_t date_bcd = ds1302_read_register(DS1302_DATE);
    uint8_t month_bcd = ds1302_read_register(DS1302_MONTH);
    uint8_t day_bcd = ds1302_read_register(DS1302_DAY);
    uint8_t year_bcd = ds1302_read_register(DS1302_YEAR);

    // Check for totally invalid data (all 0xFF or all 0x00)
    if ((sec_bcd == 0xFF && min_bcd == 0xFF) ||
        (sec_bcd == 0x00 && min_bcd == 0x00 && hour_bcd == 0x00))
    {
        time->hours = 0;
        time->minutes = 0;
        time->seconds = 0;
        time->date = 1;
        time->month = 1;
        time->day = 1;
        time->year = 24;
        return 1; // Error
    }

    // Convert BCD to decimal
    time->seconds = bcd_to_decimal(sec_bcd & 0x7F);
    time->minutes = bcd_to_decimal(min_bcd & 0x7F);
    time->hours = bcd_to_decimal(hour_bcd & 0x3F);
    time->date = bcd_to_decimal(date_bcd & 0x3F);
    time->month = bcd_to_decimal(month_bcd & 0x1F);
    time->day = bcd_to_decimal(day_bcd & 0x07);
    time->year = bcd_to_decimal(year_bcd);

    // Sanity check
    if (time->hours > 23 || time->minutes > 59 || time->seconds > 59 ||
        time->month > 12 || time->date > 31)
    {
        time->hours = 0;
        time->minutes = 0;
        time->seconds = 0;
        return 1; // Error
    }

    return 0; // Success
}

uint8_t rtc_is_scheduled_time(RTCTime_t *time)
{
    if (time->minutes == 0 && time->seconds == 0)
    {
        return 1;
    }
    return 0;
}
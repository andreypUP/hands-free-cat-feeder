#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "sensor.h"
#include "motor.h"
#include "lcd.h"
#include "rtc.h"

// ============================================================================
// ======================== FEATURE ENABLE/DISABLE ============================
// ============================================================================

#define ENABLE_UART_DEBUG       0  // Set to 0 to disable UART debugging
#define ENABLE_RTC              1  // Set to 0 to disable RTC time display
#define ENABLE_SCHEDULED_FEED   1  // Set to 0 to disable scheduled feeding
#define ENABLE_SENSOR_FEED      1  // Set to 0 to disable sensor-based feeding

// ============================================================================
// ============================= CONFIGURATION ================================
// ============================================================================

#define SCHEDULE_1_HOUR 6
#define SCHEDULE_1_MINUTE 0

#define SCHEDULE_2_HOUR 12
#define SCHEDULE_2_MINUTE 0

#define SCHEDULE_3_HOUR 18
#define SCHEDULE_3_MINUTE 0

#define SCHEDULE_4_HOUR 0
#define SCHEDULE_4_MINUTE 0

#define SCHEDULED_DISPENSE_SECONDS 7
#define SENSOR_DETECTION_DISTANCE_CM 20
#define SENSOR_DISPENSE_SECONDS 3
#define SENSOR_COOLDOWN_MS 3000
#define SENSOR_CHECK_INTERVAL_MS 500
#define RTC_CHECK_INTERVAL_MS 1000

// ============================================================================
// ========================== CONDITIONAL INCLUDES ============================
// ============================================================================

#if ENABLE_UART_DEBUG
    #include "uart.h"
    #define DEBUG_PRINT(x)      uart_print(x)
    #define DEBUG_PRINT_NUM(x)  uart_print_num(x)
    #define DEBUG_PRINTLN(x)    uart_println(x)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINT_NUM(x)
    #define DEBUG_PRINTLN(x)
#endif

// ============================================================================
// ============================== DEFINITIONS =================================
// ============================================================================

#define MS_TO_TICKS(ms) ((ms) / 10)
#define SECONDS_TO_TICKS(s) ((s) * 100)

#define SENSOR_COOLDOWN_TICKS MS_TO_TICKS(SENSOR_COOLDOWN_MS)
#define SCHEDULED_DISPENSE_TICKS SECONDS_TO_TICKS(SCHEDULED_DISPENSE_SECONDS)
#define SENSOR_DISPENSE_TICKS SECONDS_TO_TICKS(SENSOR_DISPENSE_SECONDS)
#define SENSOR_CHECK_TICKS MS_TO_TICKS(SENSOR_CHECK_INTERVAL_MS)
#define RTC_CHECK_TICKS MS_TO_TICKS(RTC_CHECK_INTERVAL_MS)

typedef enum
{
    IDLE,
    CAT_DETECTED,
    DISPENSING_SCHEDULED,
    DISPENSING_SENSOR
} CatFeederState;

// ============================================================================
// ============================ GLOBAL VARIABLES ==============================
// ============================================================================

volatile CatFeederState state = IDLE;
volatile uint32_t system_ticks = 0;
volatile uint8_t measurement_request = 0;
volatile uint8_t rtc_check_request = 0;

uint32_t sensor_cooldown_start = 0;
uint32_t dispense_start = 0;
uint32_t cat_detected_start = 0;
uint8_t scheduled_dispense_done_flags = 0;

// ============================================================================
// ========================= INTERRUPT SERVICE ROUTINE ========================
// ============================================================================

ISR(TIMER0_COMPA_vect)
{
    system_ticks++;

#if ENABLE_SENSOR_FEED
    if (system_ticks % SENSOR_CHECK_TICKS == 0)
    {
        measurement_request = 1;
    }
#endif

#if ENABLE_RTC
    if (system_ticks % RTC_CHECK_TICKS == 0)
    {
        rtc_check_request = 1;
    }
#endif
}

// ============================================================================
// ========================== UTILITY FUNCTIONS ===============================
// ============================================================================

uint32_t get_elapsed_ticks(uint32_t start_tick)
{
    return system_ticks - start_tick;
}

void timer0_init(void)
{
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS02) | (1 << CS00);
    OCR0A = 156;
    TIMSK0 = (1 << OCIE0A);
}

// ============================================================================
// ========================== LCD DISPLAY FUNCTIONS ===========================
// ============================================================================

#if ENABLE_RTC
void lcd_display_time(RTCTime_t *time)
{
    lcd_set_cursor(0, 0);

    if (time->hours < 10) lcd_print("0");
    lcd_print_num(time->hours);
    lcd_print(":");

    if (time->minutes < 10) lcd_print("0");
    lcd_print_num(time->minutes);
    lcd_print(":");

    if (time->seconds < 10) lcd_print("0");
    lcd_print_num(time->seconds);

    lcd_print("      ");
}
#endif

void lcd_display_state(CatFeederState current_state, uint16_t distance, uint8_t is_cooldown_active)
{
    lcd_set_cursor(1, 0);
    lcd_print("                ");
    lcd_set_cursor(1, 0);

    switch (current_state)
    {
    case IDLE:
        if (is_cooldown_active)
        {
            lcd_print("Next Meal");
        }
        else
        {
            lcd_print("Snack Ready");
        }
        // Distance display commented out
        // #if ENABLE_SENSOR_FEED
        //     if (distance < 10) lcd_print(" ");
        //     lcd_print_num(distance);
        //     lcd_print("cm");
        // #endif
        break;

    case CAT_DETECTED:
        lcd_print("Detected <20cm");
        break;

    case DISPENSING_SCHEDULED:
        lcd_print("Dispensing...");
        break;

    case DISPENSING_SENSOR:
        lcd_print("Dispensing...");
        break;
    }
}

// ============================================================================
// ====================== SCHEDULED FEEDING FUNCTIONS =========================
// ============================================================================

#if ENABLE_SCHEDULED_FEED

void reset_daily_schedule_flags(uint8_t current_hour)
{
    static uint8_t last_reset_day = 0xFF;

    if (current_hour == 1 && last_reset_day != current_hour)
    {
        scheduled_dispense_done_flags = 0;
        last_reset_day = current_hour;
        DEBUG_PRINTLN("Daily schedule reset");
    }
}

uint8_t is_scheduled_time(uint8_t hour, uint8_t minute)
{
    return ((hour == SCHEDULE_1_HOUR && minute == SCHEDULE_1_MINUTE) ||
            (hour == SCHEDULE_2_HOUR && minute == SCHEDULE_2_MINUTE) ||
            (hour == SCHEDULE_3_HOUR && minute == SCHEDULE_3_MINUTE) ||
            (hour == SCHEDULE_4_HOUR && minute == SCHEDULE_4_MINUTE));
}

uint8_t get_schedule_flag(uint8_t hour, uint8_t minute)
{
    if (hour == SCHEDULE_1_HOUR && minute == SCHEDULE_1_MINUTE)
        return 0x01;
    if (hour == SCHEDULE_2_HOUR && minute == SCHEDULE_2_MINUTE)
        return 0x02;
    if (hour == SCHEDULE_3_HOUR && minute == SCHEDULE_3_MINUTE)
        return 0x04;
    if (hour == SCHEDULE_4_HOUR && minute == SCHEDULE_4_MINUTE)
        return 0x08;
    return 0x00;
}

#endif // ENABLE_SCHEDULED_FEED

// ============================================================================
// ========================== RTC TIME FUNCTIONS ==============================
// ============================================================================

#if ENABLE_RTC && ENABLE_UART_DEBUG

void print_time_2digit(uint8_t value)
{
    if (value < 10)
        uart_print("0");
    uart_print_num(value);
}

// Auto-set time from compile time
#define COMPILE_YEAR ((__DATE__[9] - '0') * 10 + (__DATE__[10] - '0'))
#define COMPILE_MONTH (                                 \
    __DATE__[2] == 'n'   ? (__DATE__[1] == 'a' ? 1 : 6) \
    : __DATE__[2] == 'b' ? 2                            \
    : __DATE__[2] == 'r' ? (__DATE__[0] == 'M' ? 3 : 4) \
    : __DATE__[2] == 'y' ? 5                            \
    : __DATE__[2] == 'l' ? 7                            \
    : __DATE__[2] == 'g' ? 8                            \
    : __DATE__[2] == 'p' ? 9                            \
    : __DATE__[2] == 't' ? 10                           \
    : __DATE__[2] == 'v' ? 11                           \
                         : 12)
#define COMPILE_DAY ( \
    (__DATE__[4] == ' ' ? 0 : __DATE__[4] - '0') * 10 + (__DATE__[5] - '0'))
#define COMPILE_HOUR ((__TIME__[0] - '0') * 10 + (__TIME__[1] - '0'))
#define COMPILE_MINUTE ((__TIME__[3] - '0') * 10 + (__TIME__[4] - '0'))
#define COMPILE_SECOND ((__TIME__[6] - '0') * 10 + (__TIME__[7] - '0'))

void auto_set_rtc_time(void)
{
    RTCTime_t compile_time;

    compile_time.year = COMPILE_YEAR;
    compile_time.month = COMPILE_MONTH;
    compile_time.date = COMPILE_DAY;
    compile_time.hours = COMPILE_HOUR;
    compile_time.minutes = COMPILE_MINUTE;
    compile_time.seconds = COMPILE_SECOND;
    compile_time.day = 1;

    rtc_set_time(&compile_time);

    uart_println("RTC set to compile time:");
    uart_print("  20");
    uart_print_num(compile_time.year);
    uart_print("-");
    print_time_2digit(compile_time.month);
    uart_print("-");
    print_time_2digit(compile_time.date);
    uart_print(" ");
    print_time_2digit(compile_time.hours);
    uart_print(":");
    print_time_2digit(compile_time.minutes);
    uart_print(":");
    print_time_2digit(compile_time.seconds);
    uart_println("");
}

#endif // ENABLE_RTC && ENABLE_UART_DEBUG

// ============================================================================
// ================================ MAIN FUNCTION =============================
// ============================================================================

int main(void)
{
    // ========== INITIALIZATION ==========
#if ENABLE_UART_DEBUG
    uart_init(9600);
#endif

#if ENABLE_SENSOR_FEED
    sensor_init();
#endif

    motor_init();
    lcd_init();

#if ENABLE_RTC
    rtc_init();
#endif

    timer0_init();
    sei();

    // ========== WELCOME SCREEN ==========
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Cat Feeder v2.0");
    lcd_set_cursor(1, 0);
    lcd_print("Starting...");
    _delay_ms(1500);

    DEBUG_PRINTLN("=== Cat Feeder v2.0 ===");

#if ENABLE_RTC && ENABLE_UART_DEBUG
    auto_set_rtc_time();
#endif

    DEBUG_PRINTLN("System ready!");
    DEBUG_PRINTLN("=======================");

    // ========== VARIABLE DECLARATIONS ==========
#if ENABLE_RTC
    RTCTime_t current_time;
#endif

#if ENABLE_SENSOR_FEED
    uint16_t pulse_width = 0;
    uint16_t distance_cm = 0;
#endif

    CatFeederState last_state = IDLE;
    uint8_t last_cooldown_status = 0xFF;

    lcd_clear();

    // ============================================================================
    // ============================== MAIN LOOP ===================================
    // ============================================================================

    while (1)
    {
        // ========== RTC TIME CHECK ==========
#if ENABLE_RTC
        if (rtc_check_request)
        {
            rtc_check_request = 0;

            if (rtc_get_time(&current_time) == 0)
            {
                lcd_display_time(&current_time);
            }
            else
            {
                lcd_set_cursor(0, 0);
                lcd_print("RTC ERROR!      ");
            }

#if ENABLE_SCHEDULED_FEED
            reset_daily_schedule_flags(current_time.hours);

            if (current_time.seconds == 0 &&
                is_scheduled_time(current_time.hours, current_time.minutes))
            {
                uint8_t flag = get_schedule_flag(current_time.hours, current_time.minutes);

                if (!(scheduled_dispense_done_flags & flag) && state == IDLE)
                {
                    scheduled_dispense_done_flags |= flag;
                    DEBUG_PRINTLN("Scheduled feeding!");
                    state = DISPENSING_SCHEDULED;
                    dispense_start = system_ticks;
                }
            }
#endif // ENABLE_SCHEDULED_FEED
        }
#endif // ENABLE_RTC

        // ========== ULTRASONIC SENSOR CHECK ==========
#if ENABLE_SENSOR_FEED
        if (measurement_request && (state == IDLE || state == CAT_DETECTED))
        {
            measurement_request = 0;
            sensor_trigger();
            _delay_us(20);

            if (sensor_is_measurement_complete())
            {
                pulse_width = sensor_get_pulse_width();
                distance_cm = pulse_width / 116;
            }

            // Cat detection logic
            if (distance_cm > 0 && distance_cm < SENSOR_DETECTION_DISTANCE_CM)
            {
                if (get_elapsed_ticks(sensor_cooldown_start) >= SENSOR_COOLDOWN_TICKS)
                {
                    if (state == IDLE)
                    {
                        DEBUG_PRINT("Cat detected at ");
                        DEBUG_PRINT_NUM(distance_cm);
                        DEBUG_PRINTLN(" cm");

                        state = CAT_DETECTED;
                        cat_detected_start = system_ticks;
                    }
                }
            }
            else
            {
                if (state == CAT_DETECTED)
                {
                    state = IDLE;
                }
            }
        }
#endif // ENABLE_SENSOR_FEED

        // ========== STATE MACHINE ==========
        switch (state)
        {
        case IDLE:
            break;

        case CAT_DETECTED:
#if ENABLE_SENSOR_FEED
            if (get_elapsed_ticks(cat_detected_start) >= 100)
            {
                DEBUG_PRINTLN("Starting snack dispense...");
                state = DISPENSING_SENSOR;
                dispense_start = system_ticks;
            }
#endif
            break;

        case DISPENSING_SCHEDULED:
            motor_step_forward();

            if (get_elapsed_ticks(dispense_start) >= SCHEDULED_DISPENSE_TICKS)
            {
                motor_stop();
                DEBUG_PRINTLN("Scheduled feed complete!");
                state = IDLE;
#if ENABLE_SENSOR_FEED
                distance_cm = 0;
#endif
            }
            break;

        case DISPENSING_SENSOR:
            motor_step_forward();

            if (get_elapsed_ticks(dispense_start) >= SENSOR_DISPENSE_TICKS)
            {
                motor_stop();
                DEBUG_PRINTLN("Snack complete! Cooldown active.");

                state = IDLE;
                sensor_cooldown_start = system_ticks;
#if ENABLE_SENSOR_FEED
                distance_cm = 0;
#endif
            }
            break;
        }

        // ========== LCD UPDATE ==========
        uint8_t is_cooldown_active = (get_elapsed_ticks(sensor_cooldown_start) < SENSOR_COOLDOWN_TICKS);

        if (state != last_state || is_cooldown_active != last_cooldown_status)
        {
#if ENABLE_SENSOR_FEED
            lcd_display_state(state, distance_cm, is_cooldown_active);
#else
            lcd_display_state(state, 0, is_cooldown_active);
#endif
            last_state = state;
            last_cooldown_status = is_cooldown_active;
        }

        // ========== UPDATE DISTANCE IN IDLE (COMMENTED OUT) ==========
        // Distance reading update is disabled to prevent LCD clutter
        // #if ENABLE_SENSOR_FEED
        //     if (state == IDLE && distance_cm > 0)
        //     {
        //         lcd_set_cursor(1, 12);
        //         if (distance_cm < 10)
        //         {
        //             lcd_print(" ");
        //         }
        //         lcd_print_num(distance_cm);
        //         lcd_print("cm");
        //     }
        // #endif
    }
}

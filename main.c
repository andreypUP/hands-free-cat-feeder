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

#define ENABLE_UART_DEBUG 1
#define ENABLE_RTC 1
#define ENABLE_SCHEDULED_FEED 1
#define ENABLE_SENSOR_FEED 1

// ============================================================================
// ============================= CONFIGURATION ================================
// ============================================================================

#define SCHEDULE_1_HOUR 6
#define SCHEDULE_1_MINUTE 0
#define SCHEDULE_2_HOUR 13
#define SCHEDULE_2_MINUTE 11
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
#define RTC_SYNC_INTERVAL_MS 600000

// ============================================================================
// ========================== CONDITIONAL INCLUDES ============================
// ============================================================================

#if ENABLE_UART_DEBUG
#include "uart.h"
#define DEBUG_PRINT(x) uart_print(x)
#define DEBUG_PRINT_NUM(x) uart_print_num(x)
#define DEBUG_PRINTLN(x) uart_println(x)
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
#define SENSOR_CHECK_TICKS MS_TO_TICKS(SENSOR_CHECK_INTERVAL_MS)
#define RTC_CHECK_TICKS MS_TO_TICKS(RTC_CHECK_INTERVAL_MS)
#define RTC_SYNC_TICKS MS_TO_TICKS(RTC_SYNC_INTERVAL_MS)
#define CAT_CONFIRMATION_TICKS 100  // 1 second

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

// Event flags from ISRs
volatile uint8_t sensor_measurement_ready = 0;
volatile uint8_t motor_dispense_complete = 0;
volatile uint8_t rtc_display_request = 0;
volatile uint8_t rtc_sync_request = 0;
volatile uint8_t schedule_dispense_trigger = 0;

// Sensor data
volatile uint16_t latest_distance_cm = 0;

// Software RTC
volatile uint8_t soft_rtc_hours = 0;
volatile uint8_t soft_rtc_minutes = 0;
volatile uint8_t soft_rtc_seconds = 0;

// Schedule flags
volatile uint8_t scheduled_dispense_done_flags = 0;

// Timing variables
volatile uint32_t sensor_cooldown_end_tick = 0;
volatile uint32_t cat_detected_tick = 0;

// ============================================================================
// ======================== MOTOR COMPLETION CALLBACK =========================
// ============================================================================

// This function is called from motor.c Timer2 ISR when dispensing completes
void motor_dispense_complete_callback(void)
{
    motor_dispense_complete = 1;
}

// ============================================================================
// ======================= SENSOR MEASUREMENT CALLBACK ========================
// ============================================================================

// This function is called from sensor.c PCINT ISR when measurement completes
void sensor_measurement_complete_callback(uint16_t distance_cm)
{
    latest_distance_cm = distance_cm;
    sensor_measurement_ready = 1;
}

// ============================================================================
// ========================= INTERRUPT SERVICE ROUTINE ========================
// ============================================================================

#if ENABLE_SCHEDULED_FEED

static inline uint8_t is_scheduled_time_isr(uint8_t hour, uint8_t minute)
{
    return ((hour == SCHEDULE_1_HOUR && minute == SCHEDULE_1_MINUTE) ||
            (hour == SCHEDULE_2_HOUR && minute == SCHEDULE_2_MINUTE) ||
            (hour == SCHEDULE_3_HOUR && minute == SCHEDULE_3_MINUTE) ||
            (hour == SCHEDULE_4_HOUR && minute == SCHEDULE_4_MINUTE));
}

static inline uint8_t get_schedule_flag_isr(uint8_t hour, uint8_t minute)
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

#endif

ISR(TIMER0_COMPA_vect)
{
    system_ticks++;

#if ENABLE_SENSOR_FEED
    // Trigger sensor measurement automatically every 500ms
    if (system_ticks % SENSOR_CHECK_TICKS == 0)
    {
        // Only trigger if in IDLE or CAT_DETECTED state
        if (state == IDLE || state == CAT_DETECTED)
        {
            sensor_trigger();  // Sensor is interrupt-driven, will callback when done
        }
    }
#endif

#if ENABLE_RTC
    // Update software RTC every second
    if (system_ticks % RTC_CHECK_TICKS == 0)
    {
        rtc_display_request = 1;
        
        soft_rtc_seconds++;
        if (soft_rtc_seconds >= 60)
        {
            soft_rtc_seconds = 0;
            soft_rtc_minutes++;
            
            if (soft_rtc_minutes >= 60)
            {
                soft_rtc_minutes = 0;
                soft_rtc_hours++;
                
                if (soft_rtc_hours >= 24)
                {
                    soft_rtc_hours = 0;
                }
                
#if ENABLE_SCHEDULED_FEED
                if (soft_rtc_hours == 1 && soft_rtc_minutes == 0)
                {
                    scheduled_dispense_done_flags = 0;
                }
#endif
            }
            
#if ENABLE_SCHEDULED_FEED
            if (soft_rtc_seconds == 0 && 
                is_scheduled_time_isr(soft_rtc_hours, soft_rtc_minutes))
            {
                uint8_t flag = get_schedule_flag_isr(soft_rtc_hours, soft_rtc_minutes);
                
                if (!(scheduled_dispense_done_flags & flag) && state == IDLE)
                {
                    scheduled_dispense_done_flags |= flag;
                    schedule_dispense_trigger = 1;
                }
            }
#endif
        }
    }
    
    if (system_ticks % RTC_SYNC_TICKS == 0)
    {
        rtc_sync_request = 1;
    }
#endif

    // Check cat detection timeout
    if (state == CAT_DETECTED)
    {
        if ((system_ticks - cat_detected_tick) >= CAT_CONFIRMATION_TICKS)
        {
            // Cat confirmed for 1 second, trigger snack dispense
            state = DISPENSING_SENSOR;
            motor_start_dispensing(SENSOR_DISPENSE_SECONDS * 1000);
        }
    }
}

// ============================================================================
// ========================== UTILITY FUNCTIONS ===============================
// ============================================================================

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
void lcd_display_time_from_soft_rtc(void)
{
    uint8_t hours, minutes, seconds;
    cli();
    hours = soft_rtc_hours;
    minutes = soft_rtc_minutes;
    seconds = soft_rtc_seconds;
    sei();
    
    lcd_set_cursor(0, 0);

    if (hours < 10)
        lcd_print("0");
    lcd_print_num(hours);
    lcd_print(":");

    if (minutes < 10)
        lcd_print("0");
    lcd_print_num(minutes);
    lcd_print(":");

    if (seconds < 10)
        lcd_print("0");
    lcd_print_num(seconds);

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
// ========================== RTC TIME FUNCTIONS ==============================
// ============================================================================

#if ENABLE_RTC

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
    _delay_ms(100);
    
    cli();
    soft_rtc_hours = compile_time.hours;
    soft_rtc_minutes = compile_time.minutes;
    soft_rtc_seconds = compile_time.seconds;
    sei();

#if ENABLE_UART_DEBUG
    uart_println("RTC set to compile time:");
    uart_print("  20");
    uart_print_num(compile_time.year);
    uart_print("-");
    if (compile_time.month < 10) uart_print("0");
    uart_print_num(compile_time.month);
    uart_print("-");
    if (compile_time.date < 10) uart_print("0");
    uart_print_num(compile_time.date);
    uart_print(" ");
    if (compile_time.hours < 10) uart_print("0");
    uart_print_num(compile_time.hours);
    uart_print(":");
    if (compile_time.minutes < 10) uart_print("0");
    uart_print_num(compile_time.minutes);
    uart_print(":");
    if (compile_time.seconds < 10) uart_print("0");
    uart_print_num(compile_time.seconds);
    uart_println("");
    uart_println("Software RTC initialized");
#endif
}

void sync_soft_rtc_with_hardware(void)
{
    RTCTime_t hw_time;
    
    if (rtc_get_time(&hw_time) == 0)
    {
        cli();
        soft_rtc_hours = hw_time.hours;
        soft_rtc_minutes = hw_time.minutes;
        soft_rtc_seconds = hw_time.seconds;
        sei();
        
        DEBUG_PRINTLN("Software RTC synced with hardware");
    }
    else
    {
        DEBUG_PRINTLN("RTC sync failed!");
    }
}

#endif

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
    lcd_print("Cat Feeder v3.0");
    lcd_set_cursor(1, 0);
    lcd_print("Fully Interrupt!");
    _delay_ms(1500);

    DEBUG_PRINTLN("=== Cat Feeder v3.0 ===");
    DEBUG_PRINTLN("FULLY INTERRUPT-DRIVEN");

#if ENABLE_RTC
    auto_set_rtc_time();
#endif

    DEBUG_PRINTLN("System ready!");
    DEBUG_PRINTLN("=======================");

    // ========== VARIABLE DECLARATIONS ==========
    CatFeederState last_state = IDLE;
    uint8_t last_cooldown_status = 0xFF;
    uint16_t display_distance = 0;

    lcd_clear();

    // ============================================================================
    // ===================== MAIN LOOP - EVENT DRIVEN ONLY ========================
    // ============================================================================

    while (1)
    {
        // ========== RTC DISPLAY UPDATE (from ISR) ==========
#if ENABLE_RTC
        if (rtc_display_request)
        {
            rtc_display_request = 0;
            lcd_display_time_from_soft_rtc();
        }
        
        if (rtc_sync_request)
        {
            rtc_sync_request = 0;
            sync_soft_rtc_with_hardware();
        }
#endif

        // ========== SCHEDULED DISPENSE (from ISR) ==========
#if ENABLE_SCHEDULED_FEED
        if (schedule_dispense_trigger && state == IDLE)
        {
            schedule_dispense_trigger = 0;
            
            DEBUG_PRINTLN("Scheduled feeding triggered!");
            
            state = DISPENSING_SCHEDULED;
            motor_start_dispensing(SCHEDULED_DISPENSE_SECONDS * 1000);
        }
#endif

        // ========== SENSOR MEASUREMENT READY (from ISR callback) ==========
#if ENABLE_SENSOR_FEED
        if (sensor_measurement_ready)
        {
            sensor_measurement_ready = 0;
            
            uint16_t distance_cm = latest_distance_cm;
            display_distance = distance_cm;
            
            DEBUG_PRINT("Distance: ");
            DEBUG_PRINT_NUM(distance_cm);
            DEBUG_PRINTLN(" cm");
            
            // Cat detection logic
            if (distance_cm > 0 && distance_cm < SENSOR_DETECTION_DISTANCE_CM)
            {
                // Check if cooldown expired
                if (system_ticks >= sensor_cooldown_end_tick)
                {
                    if (state == IDLE)
                    {
                        DEBUG_PRINT("Cat detected at ");
                        DEBUG_PRINT_NUM(distance_cm);
                        DEBUG_PRINTLN(" cm - confirming...");
                        
                        state = CAT_DETECTED;
                        cat_detected_tick = system_ticks;
                        // Timer0 ISR will trigger dispensing after 1 second
                    }
                }
                else
                {
                    DEBUG_PRINTLN("Cooldown active, ignoring");
                }
            }
            else
            {
                // Cat moved away
                if (state == CAT_DETECTED)
                {
                    DEBUG_PRINTLN("Cat moved away");
                    state = IDLE;
                }
            }
        }
#endif

        // ========== MOTOR DISPENSE COMPLETE (from ISR callback) ==========
        if (motor_dispense_complete)
        {
            motor_dispense_complete = 0;
            
            if (state == DISPENSING_SCHEDULED)
            {
                DEBUG_PRINTLN("Scheduled feed complete!");
                state = IDLE;
            }
            else if (state == DISPENSING_SENSOR)
            {
                DEBUG_PRINTLN("Snack complete! Cooldown active.");
                state = IDLE;
                sensor_cooldown_end_tick = system_ticks + SENSOR_COOLDOWN_TICKS;
            }
            
            display_distance = 0;
        }

        // ========== LCD UPDATE (only when state changes) ==========
        uint8_t is_cooldown_active = (system_ticks < sensor_cooldown_end_tick);

        if (state != last_state || is_cooldown_active != last_cooldown_status)
        {
            lcd_display_state(state, display_distance, is_cooldown_active);
            last_state = state;
            last_cooldown_status = is_cooldown_active;
        }
    }
}
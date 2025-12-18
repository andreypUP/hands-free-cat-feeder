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

#define ENABLE_UART_DEBUG 1     // Set to 1 for debugging sensor
#define ENABLE_RTC 1            // Set to 0 to disable RTC time display
#define ENABLE_SCHEDULED_FEED 1 // Set to 0 to disable scheduled feeding
#define ENABLE_SENSOR_FEED 1    // Set to 0 to disable sensor-based feeding

// ============================================================================
// ============================= CONFIGURATION ================================
// ============================================================================

#define SCHEDULE_1_HOUR 6
#define SCHEDULE_1_MINUTE 0

#define SCHEDULE_2_HOUR 12
#define SCHEDULE_2_MINUTE 54

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
#define RTC_SYNC_INTERVAL_MS 600000  // Sync with hardware RTC every 10 minutes

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
#define SCHEDULED_DISPENSE_TICKS SECONDS_TO_TICKS(SCHEDULED_DISPENSE_SECONDS)
#define SENSOR_DISPENSE_TICKS SECONDS_TO_TICKS(SENSOR_DISPENSE_SECONDS)
#define SENSOR_CHECK_TICKS MS_TO_TICKS(SENSOR_CHECK_INTERVAL_MS)
#define RTC_CHECK_TICKS MS_TO_TICKS(RTC_CHECK_INTERVAL_MS)
#define RTC_SYNC_TICKS MS_TO_TICKS(RTC_SYNC_INTERVAL_MS)

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
volatile uint8_t rtc_display_request = 0;
volatile uint8_t rtc_sync_request = 0;

// Software RTC maintained in ISR
volatile uint8_t soft_rtc_hours = 0;
volatile uint8_t soft_rtc_minutes = 0;
volatile uint8_t soft_rtc_seconds = 0;

// Schedule flags and trigger
volatile uint8_t scheduled_dispense_done_flags = 0;
volatile uint8_t schedule_dispense_trigger = 0;  // ISR sets this to trigger dispensing

uint32_t sensor_cooldown_start = 0;
uint32_t dispense_start = 0;
uint32_t cat_detected_start = 0;

#if ENABLE_SENSOR_FEED
// Sensor measurement state
uint8_t sensor_triggered = 0;
uint32_t sensor_trigger_time = 0;
#endif

// ============================================================================
// ========================= INTERRUPT SERVICE ROUTINE ========================
// ============================================================================

#if ENABLE_SCHEDULED_FEED

// Helper function to check if current time matches schedule (inline for speed)
static inline uint8_t is_scheduled_time_isr(uint8_t hour, uint8_t minute)
{
    return ((hour == SCHEDULE_1_HOUR && minute == SCHEDULE_1_MINUTE) ||
            (hour == SCHEDULE_2_HOUR && minute == SCHEDULE_2_MINUTE) ||
            (hour == SCHEDULE_3_HOUR && minute == SCHEDULE_3_MINUTE) ||
            (hour == SCHEDULE_4_HOUR && minute == SCHEDULE_4_MINUTE));
}

// Helper function to get schedule flag (inline for speed)
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

#endif // ENABLE_SCHEDULED_FEED

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
    // Update software RTC every second
    if (system_ticks % RTC_CHECK_TICKS == 0)
    {
        rtc_display_request = 1;  // Request LCD update in main loop
        
        // Increment software RTC
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
                // Reset schedule flags at 1:00 AM
                if (soft_rtc_hours == 1 && soft_rtc_minutes == 0)
                {
                    scheduled_dispense_done_flags = 0;
                }
#endif
            }
            
#if ENABLE_SCHEDULED_FEED
            // Check for scheduled feeding time (at top of minute, seconds == 0)
            if (soft_rtc_seconds == 0 && 
                is_scheduled_time_isr(soft_rtc_hours, soft_rtc_minutes))
            {
                uint8_t flag = get_schedule_flag_isr(soft_rtc_hours, soft_rtc_minutes);
                
                // Check if not already dispensed and system is idle
                if (!(scheduled_dispense_done_flags & flag) && state == IDLE)
                {
                    scheduled_dispense_done_flags |= flag;
                    schedule_dispense_trigger = 1;  // Signal main loop to start dispensing
                }
            }
#endif // ENABLE_SCHEDULED_FEED
        }
    }
    
    // Request hardware RTC sync every 10 minutes
    if (system_ticks % RTC_SYNC_TICKS == 0)
    {
        rtc_sync_request = 1;
    }
#endif // ENABLE_RTC
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
void lcd_display_time_from_soft_rtc(void)
{
    // Safely read volatile variables
    uint8_t hours, minutes, seconds;
    cli();  // Disable interrupts for atomic read
    hours = soft_rtc_hours;
    minutes = soft_rtc_minutes;
    seconds = soft_rtc_seconds;
    sei();  // Re-enable interrupts
    
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

// Compile-time macros to extract date/time from __DATE__ and __TIME__
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
    
    _delay_ms(100); // Give RTC time to stabilize
    
    // Initialize software RTC with same time
    cli();  // Disable interrupts for atomic write
    soft_rtc_hours = compile_time.hours;
    soft_rtc_minutes = compile_time.minutes;
    soft_rtc_seconds = compile_time.seconds;
    sei();  // Re-enable interrupts

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
        // Update software RTC with hardware RTC time
        cli();  // Disable interrupts for atomic write
        soft_rtc_hours = hw_time.hours;
        soft_rtc_minutes = hw_time.minutes;
        soft_rtc_seconds = hw_time.seconds;
        sei();  // Re-enable interrupts
        
        DEBUG_PRINTLN("Software RTC synced with hardware");
    }
    else
    {
        DEBUG_PRINTLN("RTC sync failed!");
    }
}

#endif // ENABLE_RTC

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
    DEBUG_PRINTLN("Interrupt-Driven Schedule");

#if ENABLE_RTC
    auto_set_rtc_time();
#endif

    DEBUG_PRINTLN("System ready!");
    DEBUG_PRINTLN("Sensor enabled!");
    DEBUG_PRINTLN("=======================");

    // ========== VARIABLE DECLARATIONS ==========
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
        // ========== RTC DISPLAY UPDATE ==========
#if ENABLE_RTC
        if (rtc_display_request)
        {
            rtc_display_request = 0;
            lcd_display_time_from_soft_rtc();
        }
        
        // ========== RTC HARDWARE SYNC ==========
        if (rtc_sync_request)
        {
            rtc_sync_request = 0;
            sync_soft_rtc_with_hardware();
        }
#endif // ENABLE_RTC

        // ========== SCHEDULED DISPENSE TRIGGER (FROM ISR) ==========
#if ENABLE_SCHEDULED_FEED
        if (schedule_dispense_trigger && state == IDLE)
        {
            schedule_dispense_trigger = 0;  // Clear trigger flag
            
            DEBUG_PRINTLN("Scheduled feeding triggered by ISR!");
            
            state = DISPENSING_SCHEDULED;
            dispense_start = system_ticks;
        }
#endif // ENABLE_SCHEDULED_FEED

        // ========== ULTRASONIC SENSOR CHECK ==========
#if ENABLE_SENSOR_FEED
        if (measurement_request && (state == IDLE || state == CAT_DETECTED))
        {
            measurement_request = 0;
            
            // If we haven't triggered yet, trigger the sensor
            if (!sensor_triggered)
            {
                sensor_trigger();
                sensor_triggered = 1;
                sensor_trigger_time = system_ticks;
                DEBUG_PRINTLN("Sensor triggered");
            }
            // If we triggered, wait at least 3 ticks (~30ms) then check if measurement is complete
            else if (get_elapsed_ticks(sensor_trigger_time) >= 3)
            {
                // Check if measurement completed
                if (sensor_is_measurement_complete())
                {
                    pulse_width = sensor_get_pulse_width();
                    distance_cm = pulse_width / 116;
                    
                    // Debug output
                    DEBUG_PRINT("Distance: ");
                    DEBUG_PRINT_NUM(distance_cm);
                    DEBUG_PRINTLN(" cm");
                    
                    // Reset sensor for next measurement
                    sensor_reset();
                    sensor_triggered = 0;
                    
                    // Cat detection logic
                    if (distance_cm > 0 && distance_cm < SENSOR_DETECTION_DISTANCE_CM)
                    {
                        // Check if cooldown period has expired
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
                        else
                        {
                            DEBUG_PRINTLN("Cooldown active, ignoring detection");
                        }
                    }
                    else
                    {
                        // No cat detected or out of range
                        if (state == CAT_DETECTED)
                        {
                            DEBUG_PRINTLN("Cat moved away");
                            state = IDLE;
                        }
                    }
                }
                else
                {
                    DEBUG_PRINTLN("Sensor timeout");
                    // Reset sensor for next measurement
                    sensor_reset();
                    sensor_triggered = 0;
                    distance_cm = 0;
                }
            }
        }
#endif // ENABLE_SENSOR_FEED

        // ========== STATE MACHINE ==========
        switch (state)
        {
        case IDLE:
            // Do nothing, wait for triggers
            break;

        case CAT_DETECTED:
#if ENABLE_SENSOR_FEED
            // Wait 1 second to confirm cat presence
            if (get_elapsed_ticks(cat_detected_start) >= 100)  // 100 ticks = 1 second
            {
                DEBUG_PRINTLN("Starting snack dispense...");
                state = DISPENSING_SENSOR;
                dispense_start = system_ticks;
            }
#endif
            break;

        case DISPENSING_SCHEDULED:
            // Start motor if not already running
            if (!motor_is_running()) {
                motor_start_dispensing(SCHEDULED_DISPENSE_SECONDS * 1000);
                DEBUG_PRINTLN("Motor started for scheduled feed");
            }

            // Check if dispensing is complete
            if (get_elapsed_ticks(dispense_start) >= SCHEDULED_DISPENSE_TICKS) {
                motor_stop();
                DEBUG_PRINTLN("Scheduled feed complete!");
                state = IDLE;
#if ENABLE_SENSOR_FEED
                distance_cm = 0;
#endif
            }
            break;

        case DISPENSING_SENSOR:
            // Start motor if not already running
            if (!motor_is_running()) {
                motor_start_dispensing(SENSOR_DISPENSE_SECONDS * 1000);
                DEBUG_PRINTLN("Motor started for snack dispense");
            }

            // Check if dispensing is complete
            if (get_elapsed_ticks(dispense_start) >= SENSOR_DISPENSE_TICKS) {
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
    }
}
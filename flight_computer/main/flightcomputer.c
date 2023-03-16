/* 
    Flight computer for ME 235 project

    Last updated 3/15/2023 (unfinished)
*/
#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

// Define I/O
#define GPIO_LED              2 // Indicator LED
#define GPIO_PWM_N1              18 // Motor channel left
#define GPIO_PWM_N2              19 // Motor channel right
#define GPIO_PWM_N3              23 // Servo channel left
#define GPIO_PWM_N4              5 // Servo channel right

// Define PWM parameters (for engine control)
#define LEDC_TIMER_LEFT              LEDC_TIMER_0 // Timer for Left Motor
#define LEDC_TIMER_RIGHT              LEDC_TIMER_1 // Timer for Right Motor
#define LEDC_MODE              LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO              27 // Test output GPIO (will not be used in actual)
#define LEDC_CHANNEL_LEFT              LEDC_CHANNEL_0
#define LEDC_CHANNEL_RIGHT              LEDC_CHANNEL_1
#define LEDC_DUTY_RES              LEDC_TIMER_16_BIT // Set duty resolution to 16 bits

#define LEDC_FREQUENCY          50 // Frequency in Hertz. Set frequency at 50 Hz (standard for this type of ESC)

#define ENGINE_PWM_MAX              6554 // 10% of 2^16, corresponds to full throttle (2 ms pulse width)
#define ENGINE_PWM_MIN              3277 // 5% of 2^16, corresponds to no throttle (1 ms pulse width)

// Define PWM parameters (for servo control)
#define LEDC_TIMER_SERVO_LEFT              LEDC_TIMER_2
#define LEDC_TIMER_SERVO_RIGHT              LEDC_TIMER_3
#define LEDC_SERVO_FREQUENCY              1000 // 1 kHz for servo PWM frequency, this is because servo PWM uses RMS voltage
#define LEDC_SERVO_CHANNEL_LEFT              LEDC_CHANNEL_2
#define LEDC_SERVO_CHANNEL_RIGHT              LEDC_CHANNEL_3
#define SERVO_PWM_MIN                 0
#define SERVO_PWM_MAX                 8195

// Define I2C parameters
#define SCL_PIN              22 // I2C clock pin
#define SDA_PIN              21 // I2C data pin


// State machine variables
volatile int state = 0; // 0 = idle, 1 = connected to remote, 2 = ready to fly, 3 = auto hover, 4 = ?

// Variables with regards to connection to remote
volatile bool _receivedRemoteHandshake = false; // Have we heard from remote?
volatile long _lastCommunicationWithRemote = 0; // Last time we heard from remote (used to infer connection issues)

//

// Throttle and servo duty cycles
volatile int32_t _throttleLeft = ENGINE_PWM_MIN;
volatile int32_t _throttleRight = ENGINE_PWM_MIN;
volatile int32_t _servoLeft = SERVO_PWM_MIN;
volatile int32_t _servoRight = SERVO_PWM_MIN;

//

/* static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
} */

/// @brief Convert throttle percentage to the PWM amount...
/// @param percentage 
/// @return 
static int ScaleThrottleForce(float percentage)
{
    return (int)(ENGINE_PWM_MIN + percentage/100.0f * ENGINE_PWM_MAX); 
}

/// @brief Connection to remote control (50 Hz)
/// @param pvParam 
static void remote_conn(void* pvParam)
{
    TickType_t lastTaskTime = xTaskGetTickCount();
    const TickType_t taskFreq = 20/portTICK_PERIOD_MS;
    while(true)
    {
        switch (state)
        {
            case 0:
                ESP_LOGI("[RC-INFO]", "Awaiting connection!");
                break;
            case 1:
                ESP_LOGI("[RC-INFO]", "Connected! Awaiting engine start...");
                break;
            case 2:
                // Engine on, now the remote is actually in use
                break;
            case 3:
                break;
            case 4:
                break;
            default:
                // It should never reach this case...
                break;
        }
        //vTaskDelay(20/portTICK_PERIOD_MS);
        vTaskDelayUntil(&lastTaskTime, taskFreq);
    }
    vTaskDelete(NULL);
}

/// @brief Actuation of motors and servos... (75 Hz)
/// [Note:] This method will read from the global variables that were computed by LQI function.
/// @param pvParam 
static void motors(void* pvParam)
{
    float lastTaskTime = xTaskGetTickCount();
    const float taskFreq = 13.33f/portTICK_PERIOD_MS;

    while(true)
    {
        switch (state)
        {
            case 0:
                ESP_LOGI("[MOT-INFO]", "Motors idle - awaiting remote connection!");
                break;
            case 1:
                ESP_LOGI("[MOT-INFO]", "Connected to remote! Awaiting engine start...");
                break;
            case 2:
                break;
            case 3:
                break;
            case 4:
                break;
            default:
                // It should never reach this case...
                break;
        }
        vTaskDelayUntil(&lastTaskTime, taskFreq);
    }
    vTaskDelete(NULL);
}

/// @brief Reading from IMUs (sampling at 100 Hz)
/// @param pvParam 
static void IMU(void* pvParam)
{
    while(true)
    {
        switch (state)
        {
            case 0:
            case 1:
                ESP_LOGI("[IMU-INFO]", "Calibrating!");
                break;
            case 2:
                break;
            case 3:
                break;
            case 4:
                break;
            default:
                // It should never reach this case...
                break;
        }

        vTaskDelay(10/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

/// @brief LQI controller (80 Hz)
/// @param pvParam 
static void LQI(void* pvParam)
{
    while(true)
    {
        switch (state)
        {
            case 0:
            case 1:
                break;
            case 2:
                break;
            case 3:
                break;
            case 4:
                break;
            default:
                // It should never reach this case...
                break;
        }
        vTaskDelay(12.5f/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

/// @brief Create all of the tasks
/// @param  
void app_main(void)
{
    xTaskCreatePinnedToCore(remote_conn, "remote_conn", 4096, (void*) 1, 4, NULL, 0); // Remote control (will listen in the background)
    xTaskCreatePinnedToCore(IMU, "IMU", 4096, (void*) 1, 7, NULL, 0); // Read IMU sensors
    xTaskCreatePinnedToCore(LQI, "LQI", 4096, (void*) 1, 6, NULL, 0); // Control algorithm
    xTaskCreatePinnedToCore(motors, "motors", 4096, (void*) 1, 10, NULL, 1); // Motor actuation is on core 1

   /*  // Set the LEDC peripheral configuration
    example_ledc_init();
    // Set duty to 100%
    ESP_LOGI("A", "MAX");

    _dutyCycle = PWM_MAX;

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, _dutyCycle));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

    vTaskDelay(8000/portTICK_PERIOD_MS);
    ESP_LOGI("A", "MIN");
    _dutyCycle = PWM_MIN;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, _dutyCycle));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

    vTaskDelay(15000/portTICK_PERIOD_MS);
    ESP_LOGI("A", "5500");
    _dutyCycle = 5500;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, _dutyCycle));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL)); */
}

/* 
    Flight computer for ME 235 project

    Last updated 4/4/2023 (unfinished)
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
// Are these actually used?? (Yes, for now, but in the final version no)
#define LEDC_MODE              LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO              27 // Test output GPIO (will not be used in actual)
#define LEDC_DUTY_RES              LEDC_TIMER_16_BIT // Set duty resolution to 16 bits

#define LEDC_FREQUENCY          50 // Frequency in Hertz. Set frequency at 50 Hz (standard for this type of ESC)

// Prepare and then apply the LEDC PWM timer configuration
ledc_timer_config_t ledc_timer_left = {
    .speed_mode       = LEDC_MODE,
    .timer_num        = LEDC_TIMER_0,
    .duty_resolution  = LEDC_DUTY_RES,
    .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
    .clk_cfg          = LEDC_AUTO_CLK
};

// Prepare and then apply the LEDC PWM channel configuration
ledc_channel_config_t ledc_channel_left = {
    .speed_mode     = LEDC_MODE,
    .channel        = LEDC_CHANNEL_0,
    .timer_sel      = LEDC_TIMER_0,
    .intr_type      = LEDC_INTR_DISABLE,
    .gpio_num       = LEDC_OUTPUT_IO,
    .duty           = 0, // Set duty to 0%
    .hpoint         = 0
};

#define ENGINE_PWM_MAX              6554 // 10% of 2^16, corresponds to full throttle (2 ms pulse width)
#define ENGINE_PWM_MIN              3277 // 5% of 2^16, corresponds to no throttle (1 ms pulse width)

// Define PWM parameters (for servo control)
#define LEDC_SERVO_FREQUENCY              1000 // 1 kHz for servo PWM frequency, this is because servo PWM uses RMS voltage
#define SERVO_PWM_MIN                 0
#define SERVO_PWM_MAX                 8195

// Define I2C parameters
#define SCL_PIN              22 // I2C clock pin
#define SDA_PIN              21 // I2C data pin

// State machine variables
volatile int state = 0; // 0 = idle, 1 = connected to remote, 2 = ready to fly, 3 = auto hover, 4 = ?

// Variables with regards to connection to remote
volatile bool _receivedRemoteHandshake = false; // Have we connected to the remote?
volatile long _lastCommunicationWithRemote = 0; // Last time we heard from remote - used to infer connection issues

// The commanded angles and yaw rate will be received from the remote (based on position of remote controller)
volatile float _cmdPitch = 0; // Pitch will be an absolute value (0 = level)
volatile float _cmdRoll = 0; // Roll will be an absolute value (0 = level)
volatile float _cmdYawRate = 0; // Yaw will be only a rate because it cannot be really be stabilized with the onboard sensors
volatile float _cmdThrottlePercentage = 0; // Throttle strength in percent

// The following variables are to be used for the control loop
volatile float _reqForceLeft = 1; // Commanded thrust ratio compared to current throttle input - used for banking
volatile float _reqForceRight = 1; // Commanded thrust ratio compared to current throttle input - used for banking
volatile float _reqServoLeft = 0; // Commanded servo angle - used for PITCH - will convert to PWM
volatile float _reqServoRight = 0; // Commanded servo angle - used for YAW - will convert to PWM

// Throttle and servo duty cycles - The control loop calculates these, which will then be read by motor-running task
volatile int32_t _throttleLeft = ENGINE_PWM_MIN;
volatile int32_t _throttleRight = ENGINE_PWM_MIN;
volatile int32_t _servoLeft = SERVO_PWM_MIN;
volatile int32_t _servoRight = SERVO_PWM_MIN;

// 

static void example_ledc_init(void)
{
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_left));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_left));
}

/// @brief Convert throttle percentage to the PWM amount...
/// @param percentage 
/// @return 
static int ThrottlePercentToPWM(float percentage)
{
    return (int)(ENGINE_PWM_MIN + percentage/100.0f*(ENGINE_PWM_MAX-ENGINE_PWM_MIN)); 
}

/// @brief Connection to remote control (50 Hz)
/// @param pvParam 
static void remote_conn(void* pvParam)
{
    TickType_t lastTaskTime = xTaskGetTickCount();
    const TickType_t delay_time = pdMS_TO_TICKS(20);

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
        vTaskDelayUntil(&lastTaskTime, delay_time);
    }
    vTaskDelete(NULL);
}

/// @brief Actuation of motors and servos... (approx 75 Hz)
/// [Note:] This method will read from the global variables that were computed by LQI function.
/// @param pvParam 
static void motors(void* pvParam)
{
    TickType_t lastTaskTime = xTaskGetTickCount();
    const TickType_t delay_time = pdMS_TO_TICKS(13);

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
        vTaskDelayUntil(&lastTaskTime, delay_time);
    }
    vTaskDelete(NULL);
}

/// @brief Reading from IMUs (sampling at 100 Hz) 
/// @param pvParam 
static void IMU(void* pvParam)
{
    TickType_t lastTaskTime = xTaskGetTickCount();
    const TickType_t delay_time = pdMS_TO_TICKS(10);

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
        vTaskDelayUntil(&lastTaskTime, delay_time);
    }
    vTaskDelete(NULL);
}

/// @brief LQI controller (approx 80 Hz)
/// @param pvParam 
static void LQI(void* pvParam)
{
    TickType_t lastTaskTime = xTaskGetTickCount();
    const TickType_t delay_time = pdMS_TO_TICKS(12);

    while(true)
    {
        switch (state)
        {
            case 0:
            case 1:
                // Do nothing (no control happens while the system is off)
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
        vTaskDelayUntil(&lastTaskTime, delay_time);
    }
    vTaskDelete(NULL);
}

/// @brief Create all of the tasks
/// @param  
void app_main(void)
{
    //xTaskCreatePinnedToCore(IMU, "IMU", 4096, (void*) 1, 7, NULL, 0); // Read IMU sensors
    //xTaskCreatePinnedToCore(remote_conn, "remote_conn", 4096, (void*) 1, 4, NULL, 0); // Remote control (will listen in the background)

    //xTaskCreatePinnedToCore(LQI, "LQI", 4096, (void*) 1, 5, NULL, 1); // Control algorithm
    //xTaskCreatePinnedToCore(motors, "motors", 4096, (void*) 1, 10, NULL, 1); // Motor actuation is on core 1

   // TEST CODE BELOW (WILL BE REMOVED SOON)
    example_ledc_init();
    // Set duty to 100%
    ESP_LOGI("A", "MAX");

    _throttleLeft = ENGINE_PWM_MIN;

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, _throttleLeft));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0));

    vTaskDelay(8000/portTICK_PERIOD_MS);
    ESP_LOGI("A", "MIN");
    _throttleLeft = ENGINE_PWM_MIN;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, _throttleLeft));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0));

    vTaskDelay(15000/portTICK_PERIOD_MS);
    ESP_LOGI("A", "5500");
    _throttleLeft = 5500;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, _throttleLeft));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0));
}

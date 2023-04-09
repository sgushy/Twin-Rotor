/* 
    Remote for twin rotor helicopter

    Last updated 4/8/2023 (unfinished)
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "sdkconfig.h"

#include "ssd1306.h"

#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"

#include "espn.h"

#include "i2cdev.h"


#include "hid_host.h"
#include "hid_usage_keyboard.h"
#include "hid_usage_mouse.h"

// Define I/O
#define SUPERBRIGHTLED           48

// Define I2C parameters - Note: ESP32 can handle 2 I2C ports natively
// but we will need to make sure that it can handle simultaneous broadcast/receive
#define SCL_PIN_2_R              20 // I2C clock pin for data to the display
#define SDA_PIN_2_R              21 // I2C data pin for data to the display

#define ADDR_LCD_SCREEN         0x3c // Address of the LCD screen (may or may not be used in the final product)

#define M_PI                    3.14159265358979 // For some reason PI is needed? IDK

// LCD screen vars
SSD1306_t screen;
int center, top, bottom;
char lineChar[20];

// State machine variable(s)
volatile int state = 0; // 0 = waiting for stick connection, 1 = waiting for flight computer connection
                        //2 = waiting for you to start engine, 3 = controlling the aircraft, 4 = calibration mode of aircraft ESC
volatile bool landed = true; // whether the aviation is on the ground 
                             // starts as true, but will be judged using accelerometer jolt in flight

// Variables with regards to connection to remote
volatile bool _receivedRemoteHandshake = false; // Have we connected to the remote?
volatile long _lastCommunicationWithRemote = 0; // Last time we heard from remote - used to infer connection issues
uint8_t _remoteMACAddr[6]; // MAC address of remote (this MAC address)
uint8_t _aircraftMACAddr[6]; // MAC address of aircraft (will be detected during remote ping/proximity scan)

#define ESPNOW_MAXDELAY 512
static QueueHandle_t remote_conn_queue;

// The commanded angles and yaw rate will be broadcasted to the aircraft
volatile float _cmdPitch = 0; // Pitch will be an absolute value (0 = level)
volatile float _cmdRoll = 0; // Roll will be an absolute value (0 = level)
volatile float _cmdYawRate = 0; // Yaw will be only a rate because it cannot be really be stabilized with the onboard sensors
volatile float _cmdThrottlePercentage = 0; // Throttle strength in percent

struct StickEventData // Flight stick data transmission structure
{
  union {
    uint32_t axes;
    struct {
      uint32_t x : 10;
      uint32_t y : 10;
      uint32_t hat : 4;
      uint32_t twist : 8;      
    };
  };
  uint8_t buttons_a;
  uint8_t slider;
  uint8_t buttons_b;
};

/// @brief Initialize I2C master for transmitting data to display screen
static void i2c_R_init() {
    // Configure the I2C master interface
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER; // (is master for flight computer, slave for datalink computer)
    conf.sda_io_num = (gpio_num_t)SDA_PIN_2_R;
    conf.scl_io_num = (gpio_num_t)SCL_PIN_2_R;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    conf.clk_flags = 0;
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

/* WiFi should start before using ESPNOW */
static void wifi_init()
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    // Initialize wifi
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start());
    //ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
}

static void ESP_NOW_init()
{
    esp_read_mac(_remoteMACAddr, ESP_MAC_WIFI_STA);
}

/// @brief LCD screen
static void LCD_init()
{
    screen._address = ADDR_LCD_SCREEN;
    i2c_init(&screen, 128, 32);
}

static void LCD_disp(char *msg, uint8_t line, bool backlight)
{
    ssd1306_display_text(&screen, line, msg, 20, backlight);
}

static void LCD_clear_line(uint8_t line)
{
    ssd1306_clear_line(&screen, line, true);    
}

/// @brief For updating the LCD screen with relevant information 
/// (I think it is rate-limited because the screen has a slower refresh rate than the other sub-systems)
/// @param param 
static void update_LCD(void* param)
{
    TickType_t lastTaskTime = xTaskGetTickCount();
    const TickType_t delay_time = pdMS_TO_TICKS(150);
    
    char thisMACAddr[16];
    snprintf(thisMACAddr, sizeof(thisMACAddr), "%02X:%02X:%02X:%02X%02X%02X", _remoteMACAddr[0],_remoteMACAddr[1],_remoteMACAddr[2],_remoteMACAddr[3],_remoteMACAddr[4],_remoteMACAddr[5]);

    while(true)
    {
        switch (state) 
        {
            case 0:
                LCD_disp(thisMACAddr,0,false);
                LCD_disp("How will you fly",1,false);
                LCD_disp("a chopper w/o a ",2,false);
                LCD_disp("flight stick??? ",3,false);
                break;
            case 1:
                LCD_disp("                ",0,false);
                LCD_disp("Searching for FC",1,false);
                LCD_disp("A               ",2,false);
                LCD_disp("AWAT-ENG-START  ",3,true); 
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

/// @brief Check connection to remote control (Every 25 ms) 
/// @param pvParam 
static void remote_conn(void* pvParam)
{
    TickType_t lastTaskTime = xTaskGetTickCount();
    const TickType_t delay_time = pdMS_TO_TICKS(25);
    
    while(true)
    {
        switch (state)
        {
            case 0:
                //ESP_LOGI("[RC-INFO]", "Awaiting connection!");
                break;
            case 1:
                //ESP_LOGI("[RC-INFO]", "Connected! Awaiting engine start...");
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

/// @brief Create all of the tasks
/// @param  
extern "C" void app_main(void)
{
    // gpio_reset_pin(SUPERBRIGHTLED);
    // /* Set the GPIO as a push/pull output */
    // gpio_set_direction(SUPERBRIGHTLED, GPIO_MODE_OUTPUT);
    // gpio_set_level(SUPERBRIGHTLED, 0);

    i2c_R_init(); // Start I2C bus to flight computer with error check
    LCD_init();
    
    LCD_disp("LCD_init OK    ",0,false); // Confirm successful I2C start
    vTaskDelay(250/portTICK_PERIOD_MS);
    LCD_disp("I2C_init OK    ",1,false);

    wifi_init();
    vTaskDelay(250/portTICK_PERIOD_MS);
    
    LCD_disp("WIFI_init OK",2,false);

    vTaskDelay(250/portTICK_PERIOD_MS);

    ESP_NOW_init();
    LCD_disp("ESP_NOW_init OK",3,false);

    vTaskDelay(250/portTICK_PERIOD_MS);

    xTaskCreatePinnedToCore(remote_conn, "remote_conn", 4096, (void*) 1, 4, NULL, 0); // Task for remote control (will listen in the background)
    xTaskCreatePinnedToCore(update_LCD, "update_LCD", 4096, (void*) 1, 2, NULL, 0); // Task for screen update
}
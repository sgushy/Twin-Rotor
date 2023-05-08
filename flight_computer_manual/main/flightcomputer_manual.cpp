/* 
    Manual flight mode! Do you feel lucky??
    Last updated 5/2/2023 (unfinished)
*/
#include <stdio.h>
#include <string.h>

#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
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
#include "datalink.h"

// Define I/O
#define GPIO_LED                 2 // Indicator LED
#define GPIO_PWM_N1              26 // Motor channel left
#define GPIO_PWM_N2              18 // Motor channel right
#define GPIO_PWM_N3              19 // Servo channel left
#define GPIO_PWM_N4              23 // Servo channel right

// Define PWM parameters (for engine control)
// Are these actually used?? (Yes, for now, but in the final version no)
#define LEDC_MODE              LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO              27 // Test output GPIO (will not be used in actual)
#define LEDC_DUTY_RES              LEDC_TIMER_16_BIT // Set duty resolution to 16 bits

#define LEDC_FREQUENCY          50 // Frequency in Hertz. Set frequency at 50 Hz (standard for this type of ESC)

#define ENGINE_PWM_MAX              6554 // 10% of 2^16, corresponds to full throttle (2 ms pulse width)
#define ENGINE_PWM_MIN              3277 // 5% of 2^16, corresponds to no throttle (1 ms pulse width)

// Prepare and then apply the LEDC PWM timer configuration
ledc_timer_config_t pwm_timer = {
    .speed_mode       = LEDC_MODE,
    .duty_resolution  = LEDC_DUTY_RES,
    .timer_num        = LEDC_TIMER_0,
    .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
    .clk_cfg          = LEDC_AUTO_CLK
};

// Prepare and then apply the LEDC PWM channel configuration
ledc_channel_config_t motor1_channel = {
    .gpio_num       = GPIO_PWM_N2,
    .speed_mode     = LEDC_MODE,
    .channel        = LEDC_CHANNEL_0,
    .intr_type      = LEDC_INTR_DISABLE,
    .timer_sel      = LEDC_TIMER_0,
    .duty           = 0, // Set duty to 0%
    .hpoint         = 0
};

ledc_channel_config_t motor2_channel = {
    .gpio_num       = GPIO_PWM_N1,
    .speed_mode     = LEDC_MODE,
    .channel        = LEDC_CHANNEL_1,
    .intr_type      = LEDC_INTR_DISABLE,
    .timer_sel      = LEDC_TIMER_0,
    .duty           = 0, // Set duty to 0%
    .hpoint         = 0
};

ledc_channel_config_t servo1_channel = {
    .gpio_num       = GPIO_PWM_N4,
    .speed_mode     = LEDC_MODE,
    .channel        = LEDC_CHANNEL_2,
    .intr_type      = LEDC_INTR_DISABLE,
    .timer_sel      = LEDC_TIMER_0,
    .duty           = 0, // Set duty to 0%
    .hpoint         = 0
};

ledc_channel_config_t servo2_channel = {
    .gpio_num       = GPIO_PWM_N3,
    .speed_mode     = LEDC_MODE,
    .channel        = LEDC_CHANNEL_3,
    .intr_type      = LEDC_INTR_DISABLE,
    .timer_sel      = LEDC_TIMER_0,
    .duty           = 0, // Set duty to 0%
    .hpoint         = 0
};

// Define PWM parameters (for servo control) - May need to change since servo specs are unclear
#define LEDC_SERVO_FREQUENCY          50 
#define SERVO_PWM_MIN                 3277//4527      // 3277 is absolute min
#define SERVO_PWM_MAX                 6554//5304      // 6554 is absolute max  
#define SERVO_PWM_MID                 4916      // Corresponds to approx zero degrees

//#define SERVO_PWM_MIN                 4916 // corresponds to 0 deg servo; this is approximately -33 degrees from horizontal
//#define SERVO_PWM_MAX                 5877 // corresponds to 66 deg servo; approximately +33 degrees from horizontal
//#define SERVO_PWM_MID                 5396 // Corresponds to ~33 deg servo, which is about the horizontal rotor plane

// Define I2C parameters - Note: ESP32 can handle 2 I2C ports natively
// but we will need to make sure that it can handle simultaneous broadcast/receive
#define SCL_PIN_2_FC              22 // I2C clock pin for data to the flight computer
#define SDA_PIN_2_FC              21 // I2C data pin for data to the flight computer
#define SCL_PIN_2_DL              17 // I2C clock pin for data to the datalink computer
#define SDA_PIN_2_DL              16 // I2C data pin for data to the datalink computer

#define ADDR_ESP_SLAV        0x20 // ESP32 memory address for I2C connection shared by datalink/flight controller
                                  // for use in the slave bus (in the flight computer case, this points to the datalink)
                                  // in the datalink case, this points to the flight computer

#define ADDR_IMU_F           0x68 // Primary IMU memory address for I2C connection
#define ADDR_IMU_R           0x69 // Secondary IMU memory address for I2C connection

#define ADDR_LCD_SCREEN      0x3c // Address of the LCD screen

#define LQI_PERIOD_MS        28 // Period to discretize control loop execution time   
#define IMU_PERIOD_MS        12 // Period to discretize IMU read time

#define MAX_THRUST_N         16.778f // Approximate max thrust per propeller in newtons

// LCD screen vars
SSD1306_t screen;
int center, top, bottom;
char lineChar[20];

// LQI gains matrix K (4x9)
const float K[4][9] = {{0.0273, 0.0158, 0, 0, -1.2423, -0.7381, -0.0157, 0, 0.7069},
                        {-0.0273, -0.0158, 0, 0, 1.2423, 0.7381, 0.0157, 0, -0.7069},
                        {-1.2693, -0.7861, -1.3094, -0.8588, -0.0287, -0.0182, 0.7069, 0.7071, 0.0157},
                        {1.2693, 0.7861, -1.3094, -0.8588, 0.0287, 0.0182, -0.7069, 0.7071, -0.0157}};

// Current angle targets (provided by ground station)
float ypr_target[3] = {0, 0, 0};

// Integrator values for state angles
float ypr_accum[3] = {0, 0, 0};

// Derivatives of state angles
float ypr_prime[3] = {0, 0, 0};

// Old values for each angle, for calculating derivatives
float ypr_old[3] = {0, 0, 0};

// Global state machine variable(s) - will we still use?
volatile int _state = 0; // 0 = idle, 1 = connected to remote, 2 = fly, 3 = auto hover, 4 = calibration mode
volatile bool _landed = true; // whether the aviation is on the ground 
                             // starts as true, but will be judged using accelerometer jolt in flight
                             
// Variables with regards to state estimation from first IMU
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   Main yaw/pitch/roll container and gravity vector
uint16_t packetSize = 42;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU

// Variables with regards to state estimation from second IMU
Quaternion q2;           // [w, x, y, z]         quaternion container
VectorFloat gravity2;    // [x, y, z]            gravity vector
float ypr2[3];           // [yaw, pitch, roll]   BACKUP ONLY yaw/pitch/roll container and gravity vector
uint16_t packetSize2 = 42;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount2;     // count of all bytes currently in FIFO
uint8_t fifoBuffer2[64]; // FIFO storage buffer
uint8_t mpuIntStatus2;   // holds actual interrupt status byte from MPU

// Variables with regards to connection to remote
#define ESPNOW_MAXDELAY 512 // Not sure if this is used, but since #define takes no memory... it's fine I guess
#define CONN_TIMEOUT 4096 // If no received packet within this time, we assume lost connection
volatile bool _receivedRemoteHandshake = false; // Have we connected to the remote?
volatile long _lastCommunicationWithRemote = 0; // Last time we heard from remote - used to infer connection issues
uint8_t _aircraftMACAddr[6] = {00,00,00,00,00,00}; // Our Mac Address
uint8_t _remoteMACAddr[6] = {00,00,00,00,00,00}; // That of the remote
static QueueHandle_t remote_conn_queue; // The queue containing messages to send to and receive from the FC
uint16_t last_packetID = 0; 
uint8_t last_button_press = 0; // Joystick button, trigger = 1, BTN 2 = 2, BTN 3 = 4, BTN 4 = 8, BTN 5 = ??, BTN 6 = ??
uint8_t last_hat_switch = 8; // Joystick hat switch, neutral = 8, right = 2, left = 6, up = 0, down = 4

// The commanded angles and yaw rate will be received from the remote (based on position of remote controller)
volatile float _cmdThrottlePercentage = 0; // Throttle strength in percent (should be between 0 - 100)

// The following variables are to be used for the control loop (Are they?)
// these are to be converted from the commanded pitch/roll/yaw/throttle to values that are 
volatile uint8_t _forcePercentageLeft = 0; // Commanded % of left thrust, added to open loop thrust
volatile uint8_t _forcePercentageRight = 0; // Commanded % of left thrust, added to open loop thrust
volatile uint8_t _reqServoLeft = 0; // Commanded servo angle in radians - used for PITCH - will convert to PWM
volatile uint8_t _reqServoRight = 0; // Commanded servo angle in radians - used for YAW - will convert to PWM

// Throttle and servo duty cycles - The control loop calculates these, which will then be read by motor-running task
// to set the motor powers and servo angles - Initialize at zero so that the aircraft does not try to take off for some reason
volatile uint32_t _throttleLeft = 0;
volatile uint32_t _throttleRight = 0;
volatile uint32_t _servoLeft = 0;
volatile uint32_t _servoRight = 0;

static void example_ledc_init(void)
{
    //ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_left));
    //ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_left));
}

/// @brief Initialize I2C master for transmitting data to flight computer
static void i2c_FC_init() {
    // Configure the I2C master interface
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER; // (is master for flight computer, slave for datalink computer)
    conf.sda_io_num = (gpio_num_t)SDA_PIN_2_FC;
    conf.scl_io_num = (gpio_num_t)SCL_PIN_2_FC;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    conf.clk_flags = 0;
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

/// @brief Initialize I2C slave for transmitting data to datalink computer
static void i2c_DL_init() {
    // Configure the I2C slave interface
    i2c_config_t conf2;
    conf2.mode = I2C_MODE_SLAVE; // (is slave for flight computer, master for datalink computer)
    conf2.sda_io_num = (gpio_num_t)SDA_PIN_2_DL;
    conf2.scl_io_num = (gpio_num_t)SCL_PIN_2_DL;
    conf2.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf2.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf2.slave.addr_10bit_en = 0;
    conf2.slave.slave_addr = ADDR_ESP_SLAV;
    conf2.clk_flags = 0;
    conf2.slave.maximum_speed = 400000; // Apparently this is needed for the I2C to work (no explanation given in docs)
    i2c_param_config(I2C_NUM_1, &conf2);
    i2c_driver_install(I2C_NUM_1, conf2.mode, 128, 128, 0); // For some reason SW I2C buffer must be > 100 (no explanation given in docs)
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
    //ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11N)); //|WIFI_PROTOCOL_LR) );
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{ 
    espnow_event_t evt;
    espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL) {
        ESP_LOGE("ESPN", "Send cb arg error");
        return;
    }

    evt.id = ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    // if (xQueueSend(remote_conn_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
    //     ESP_LOGE("ESPN", "Send send queue fail");
    // }
}

static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    espnow_event_t evt;
    espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    ESP_LOGI("ESPN", "Received packet !");

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE("ESPN", "Strange things have happened to packet!");
        return;
    }

    evt.id = ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = (uint8_t *) malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE("ESPN", "Malloc receive data fail");
        return;
    }

    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(remote_conn_queue, &evt, 0) != pdTRUE) {
        ESP_LOGE("ESPN", "Send receive queue fail");
        free(recv_cb->data);
    }
    else
    {
        ESP_LOGI("ESPN","Placed packet in queue!");
    }
}

/* Prepare ESPNOW data to be sent. */
void espnow_data_prepare(espnow_send_param_t *send_param)
{
    ESP_LOGI("ESPN","Spawning the data!");

    espnow_data_t *buf = (espnow_data_t *)send_param->buffer;

    assert(send_param->len >= sizeof(espnow_data_t));

    buf->type = send_param->type; //= IS_BROADCAST_ADDR(send_param->dest_mac) ? ESPNOW_DATA_BROADCAST : ESPNOW_DATA_UNICAST;
    buf->state = _state;
    buf->sender[0] = _aircraftMACAddr[0]; // Tag sender with our MAC address
    buf->sender[1] = _aircraftMACAddr[1];
    buf->sender[2] = _aircraftMACAddr[2];
    buf->sender[3] = _aircraftMACAddr[3];
    buf->sender[4] = _aircraftMACAddr[4];
    buf->sender[5] = _aircraftMACAddr[5];

    if(send_param->broadcast)
    {
        buf->recipient[0] = 0xFF; // No particular intended target of the package
        buf->recipient[1] = 0xFF;
        buf->recipient[2] = 0xFF;
        buf->recipient[3] = 0xFF;
        buf->recipient[4] = 0xFF;
        buf->recipient[5] = 0xFF;
    }
    else
    {
        buf->recipient[0] = _remoteMACAddr[0]; // Targeting the aircraft MAC address
        buf->recipient[1] = _remoteMACAddr[1];
        buf->recipient[2] = _remoteMACAddr[2];
        buf->recipient[3] = _remoteMACAddr[3];
        buf->recipient[4] = _remoteMACAddr[4];
        buf->recipient[5] = _remoteMACAddr[5];
    }

    buf->ypr[0] = ypr[0]; // Here, reply with the yaw, pitch, roll angles
    buf->ypr[1] = ypr[1];
    buf->ypr[2] = ypr[2];
    buf->throttle = _cmdThrottlePercentage/100;

    switch(send_param->type)
    {
        case 0: // Pinging!
        case 1:
            buf->button = 88;
            buf->hat = 69;
        break;
        case 2: // Handshaking!
        case 3:
            buf->button = 69;
            buf->hat = 88;
        break;
        case 4: // Regular operation
        case 5: // Since we are the airplane, there are no buttons on the airplane...
            buf->button = 0;
            buf->hat = 0;
        break;
        default:
        break;
    }
    
    buf->state = _state;
    buf->packet_ID = esp_random(); // Because we send redundant packets to deal with packet loss  
                                   // the recipient should only process one copy of the same packet
}

/// @brief Loadly broadcast the data to all listeners...
/// @param data The data send parameters
/// @return whether it was sent out successfully
static esp_err_t esp_now_blast(espnow_send_param_t *data)
{
    /* Start sending broadcast ESPNOW data. */
    data->dest_mac[0] = 0xFF; // For the broadcasting, send FF:FF:FF:FF:FF:FF
    data->dest_mac[1] = 0xFF;
    data->dest_mac[2] = 0xFF;
    data->dest_mac[3] = 0xFF;
    data->dest_mac[4] = 0xFF;
    data->dest_mac[5] = 0xFF;

    esp_now_peer_info_t peerInfo = {};
    memcpy(&peerInfo.peer_addr, data->dest_mac, 6);
    if (!esp_now_is_peer_exist(data->dest_mac))
    {
        esp_now_add_peer(&peerInfo); // Add peer in case we don't have it for some reason
    }
    
    esp_err_t has_err = pdFALSE;

    ESP_LOGI("ESPN", "START BROADCAST to %02X:%02X:%02X:%02X:%02X:%02X", data->dest_mac[0],data->dest_mac[1],data->dest_mac[2],data->dest_mac[3],data->dest_mac[4],data->dest_mac[5]);

    for (size_t i = 0; i < data->count; i++)
    {
        ESP_LOGD("ESPN", "Packet_Repeated_Send %X", i);
        has_err = esp_now_send(data->dest_mac, data->buffer, data->len);
        vTaskDelay(data->delay);
    }

    ESP_LOGI("ESPN", "FINISHED BROADCAST to %02X:%02X:%02X:%02X:%02X:%02X", data->dest_mac[0],data->dest_mac[1],data->dest_mac[2],data->dest_mac[3],data->dest_mac[4],data->dest_mac[5]);
    return (has_err);
}

/// @brief Loadly broadcast the data to all listeners...
/// @param data The data send parameters
/// @return whether it was sent out successfully
static esp_err_t esp_now_targeted(espnow_send_param_t *data)
{
    data->dest_mac[0] = _remoteMACAddr[0]; // For the targeted send, direct to remote MAC address
    data->dest_mac[1] = _remoteMACAddr[1];
    data->dest_mac[2] = _remoteMACAddr[2];
    data->dest_mac[3] = _remoteMACAddr[3];
    data->dest_mac[4] = _remoteMACAddr[4];
    data->dest_mac[5] = _remoteMACAddr[5];

    esp_now_peer_info_t peerInfo = {};
    memcpy(&peerInfo.peer_addr, data->dest_mac, 6);
    if (!esp_now_is_peer_exist(data->dest_mac))
    {
        esp_now_add_peer(&peerInfo); // Add peer in case we don't have it for some reason
    }

    ESP_LOGI("ESPN", "START TARGET SEND to %02X:%02X:%02X:%02X:%02X:%02X", data->dest_mac[0],data->dest_mac[1],data->dest_mac[2],data->dest_mac[3],data->dest_mac[4],data->dest_mac[5]);
    esp_err_t has_err = pdFALSE;
    
    for (size_t i = 0; i < data->count; i++)
    {
        ESP_LOGD("ESPN", "Packet_Repeated_Send %X", i);
        has_err = esp_now_send(data->dest_mac, data->buffer, data->len);
        vTaskDelay(data->delay);
    }

    ESP_LOGI("ESPN", "FINISHED TARGET SEND to %02X:%02X:%02X:%02X:%02X:%02X", data->dest_mac[0],data->dest_mac[1],data->dest_mac[2],data->dest_mac[3],data->dest_mac[4],data->dest_mac[5]);
    return (has_err);
}

/* Parse received ESPNOW data. Returns [-1] for wrong person, [0] for duplicate, [1] for success received*/
int espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *last_packetID)
{ 
    espnow_data_t *buf = (espnow_data_t *)data;
 
    if (data_len < sizeof(espnow_data_t)) {
        ESP_LOGE("ESPN", "Received ESPNOW data too short, length:%d", data_len);\
        return -1;
    }

    *state = buf->state;
    
    if(buf->type == 4) // Regular remote connection
    {
        for(int ii = 0; ii < 6; ii++)
        {
            if(buf->recipient[ii] != _aircraftMACAddr[ii])
            {
                ESP_LOGE("ESPN", "Packet was not for us!");
                ESP_LOGE("ESPN", "Intended recipient: %X:%X:%X:%X:%X:%X",buf->recipient[0],buf->recipient[1],buf->recipient[2],buf->recipient[3],buf->recipient[4],buf->recipient[5]);
                ESP_LOGE("ESPN", "Us: %X:%X:%X:%X:%X:%X",_aircraftMACAddr[0],_aircraftMACAddr[1],_aircraftMACAddr[2],_aircraftMACAddr[3],_aircraftMACAddr[4],_aircraftMACAddr[5]);
                
                return -1; // Packet was not intended for us if this is true!!!
            }
        }

        ypr_target[0] = buf->ypr[0]* M_PI/180;
        ypr_target[1] = buf->ypr[1]* M_PI/180;
        ypr_target[2] = buf->ypr[2]* M_PI/180;

        last_button_press = buf->button;
        last_hat_switch = buf->hat;
        
        _cmdThrottlePercentage = buf->throttle * 100; // Convert the float of 0-1 to a percentage

        _lastCommunicationWithRemote = 0; // We just heard from the remote!

        /* Initialize sending parameters. */
        espnow_send_param_t *response_packet = (espnow_send_param_t *) malloc(sizeof(espnow_send_param_t));
        if (response_packet == NULL) 
        {
            ESP_LOGE("ESPN", "Malloc send parameter fail");
            vSemaphoreDelete(remote_conn_queue);
            esp_now_deinit();
            //return ESP_FAIL;
            return -1;
        }
        memset(response_packet, 0, sizeof(espnow_send_param_t));
        response_packet->unicast = true;
        response_packet->broadcast = false;
        response_packet->state = 0;
        response_packet->magic = 999999;
        response_packet->count = 1;
        response_packet->delay = 5/portTICK_PERIOD_MS;
        response_packet->len = sizeof(espnow_data_t);
        response_packet->buffer = (uint8_t*) malloc(sizeof(espnow_data_t));
        response_packet->type = 5;

        if (response_packet->buffer == NULL) {
            ESP_LOGE("ESPN", "Malloc send buffer fail");
            free(response_packet);
            vSemaphoreDelete(remote_conn_queue);
            esp_now_deinit();
            //return ESP_FAIL;
            return -1;
        }
        
        memcpy(response_packet->dest_mac, _remoteMACAddr, ESP_NOW_ETH_ALEN);
        espnow_data_prepare(response_packet);

        ESP_LOGI("PING", "PACKET READY!");

        if(esp_now_targeted(response_packet) != ESP_OK)
        {
            ESP_LOGE("PING", "Could not ping!!");
        }
        else
        {
            ESP_LOGI("PING", "PINGING!!!!!");
        }

        free(response_packet);

        return 1;
    }

    if(buf->packet_ID == *last_packetID)
    {
        ESP_LOGI("ESPN", "Packet was redundant!");
        return 0; // Redundant packet, ignore!
    }
    else if(buf->type == 0 && buf->button == 88) // This must be a ping from the remote!
    {
        for(int ii = 0; ii < 6; ii++)
        {
            _remoteMACAddr[ii] = buf->sender[ii];
        }

        ESP_LOGI("Ping","From remote: MACADDR %02X:%02X:%02X:%02X:%02X:%02X", buf->sender[0], buf->sender[1], buf->sender[2], buf->sender[3], buf->sender[4], buf->sender[5]);
        //ESP_LOGI("REM","We will record this aircraft MAC address in the QUEUE");

        //xQueueSend(potential_connections, buf->recipient, pdMS_TO_TICKS(69)); // Place this into the potential connections queue

        *last_packetID = buf->packet_ID; // Update the packet ID, we have received this data!

        /* Reply to handshake. */
        espnow_send_param_t *response_packet = (espnow_send_param_t *) malloc(sizeof(espnow_send_param_t));
        if (response_packet == NULL) 
        {
            ESP_LOGE("ESPN", "Malloc send parameter fail");
            vSemaphoreDelete(remote_conn_queue);
            esp_now_deinit();
            //return ESP_FAIL;
        }

        _lastCommunicationWithRemote = 0; // We just heard from the remote!

        memset(response_packet, 0, sizeof(espnow_send_param_t));
        response_packet->unicast = false;
        response_packet->broadcast = true;
        response_packet->state = 0;
        response_packet->magic = 999999;
        response_packet->count = 2;
        response_packet->delay = 5/portTICK_PERIOD_MS;
        response_packet->len = sizeof(espnow_data_t);
        response_packet->buffer = (uint8_t*) malloc(sizeof(espnow_data_t));
        response_packet->type = 1;

        if (response_packet->buffer == NULL) {
            ESP_LOGE("PING", "Malloc send buffer fail");
            free(response_packet);
            vSemaphoreDelete(remote_conn_queue);
            esp_now_deinit();
            //return ESP_FAIL;
            return -1;
        }
        
        memcpy(response_packet->dest_mac, buf->recipient, ESP_NOW_ETH_ALEN);
        espnow_data_prepare(response_packet);

        ESP_LOGI("PING", "PACKET READY!");

        if(esp_now_blast(response_packet) != ESP_OK)
        {
            ESP_LOGE("PING", "Could not ping!!");
        }
        else
        {
            ESP_LOGI("PING", "PINGING!!!!!");
        }

        free(response_packet);
    }

    else if(buf->type == 2 && buf->hat == 88) // This must be a connection request!
    {
        for(int ii = 0; ii < 6; ii++)
        {
            _remoteMACAddr[ii] = buf->sender[ii];
        }

        ESP_LOGI("Ping","CONNECTED TO REMOTE - %02X:%02X:%02X:%02X:%02X:%02X", _remoteMACAddr[0], _remoteMACAddr[1], _remoteMACAddr[2], _remoteMACAddr[3], _remoteMACAddr[4], _remoteMACAddr[5]);
        
        _state = 1; // We are now connected to the remote!
        *last_packetID = buf->packet_ID; // Update the packet ID, we have received this data!

        /* Reply to sender */
        espnow_send_param_t *response_packet = (espnow_send_param_t *) malloc(sizeof(espnow_send_param_t));
        if (response_packet == NULL) 
        {
            //ESP_LOGE(TAG, "Malloc send parameter fail");
            vSemaphoreDelete(remote_conn_queue);
            esp_now_deinit();
            //return ESP_FAIL;
        }
        memset(response_packet, 0, sizeof(espnow_send_param_t));
        response_packet->unicast = true;
        response_packet->broadcast = false;
        response_packet->state = 0;
        response_packet->count = 2;
        response_packet->delay = 5/portTICK_PERIOD_MS;
        response_packet->len = sizeof(espnow_data_t);
        response_packet->buffer = (uint8_t*) malloc(sizeof(espnow_data_t));
        response_packet->type = 3;

        if (response_packet->buffer == NULL) {
            //ESP_LOGE(TAG, "Malloc send buffer fail");
            free(response_packet);
            vSemaphoreDelete(remote_conn_queue);
            esp_now_deinit();
            //return ESP_FAIL;
        }

        _lastCommunicationWithRemote = 0; // We just heard from the remote!
        
        memcpy(response_packet->dest_mac, _remoteMACAddr, ESP_NOW_ETH_ALEN);
        espnow_data_prepare(response_packet);

        ESP_LOGI("PING", "PACKET READY!");

        if(esp_now_targeted(response_packet) != ESP_OK)
        {
            ESP_LOGE("PING", "Could not ping!!");
        }
        else
        {
            ESP_LOGI("PING", "PINGING!!!!!");
        }

        free(response_packet);
    }

    return buf->type;
}

static esp_err_t ESP_NOW_init()
{
    esp_read_mac(_aircraftMACAddr, ESP_MAC_WIFI_STA); // For use in display of own MAC address
    ESP_LOGI("ESPN", "I know my MAC address");
    //espnow_send_param_t *send_param;

    remote_conn_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (remote_conn_queue == NULL) {
        //ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }
    ESP_LOGI("ESPN", "I have created a queue...");
    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    //ESP_ERROR_CHECK( esp_now_register_send_cb(espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(espnow_recv_cb) );
#if CONFIG_ESP_WIFI_STA_DISCONNECTED_PM_ENABLE
    ESP_ERROR_CHECK( esp_now_set_wake_window(65535) );
#endif
    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)"pmk1234567890123") );
    //ESP_LOGI("ESPN", "I have initialized one part.");
    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = (esp_now_peer_info_t *) malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE("ESPN", "Malloc peer information fail");
        vSemaphoreDelete(remote_conn_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = 1;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, _remoteMACAddr, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);
    //ESP_LOGI("ESPN", "I have initialized 2 part");
     /* Initialize sending parameters. */
    espnow_send_param_t *pingpacket = (espnow_send_param_t *) malloc(sizeof(espnow_send_param_t));
    if (pingpacket == NULL) {
        ESP_LOGE("ESPN", "Malloc send parameter fail");
        vSemaphoreDelete(remote_conn_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    
    //ESP_LOGI("ESPN", "I have initialized 3 part");
        memset(pingpacket, 0, sizeof(espnow_send_param_t));
        pingpacket->unicast = false;
        pingpacket->broadcast = true;
        pingpacket->state = _state; // We use this to highlight the state of the remote 
        pingpacket->magic = 0; // NOTE: Magic is unused
        pingpacket->count = 25; // Number of times to send
        pingpacket->delay = 10/portTICK_PERIOD_MS; // Delay between consecutive packet sends
        pingpacket->len = sizeof(espnow_data_t); // Length of packet
        pingpacket->buffer = (uint8_t*) malloc(sizeof(espnow_data_t)); // Pointer to the packet data

        pingpacket->type = 0;

    //ESP_LOGI("ESPN", "I have initialized 4 part");
        if (pingpacket->buffer == NULL) {
            ESP_LOGE("ESPN", "Malloc send buffer fail");
            free(pingpacket);
            vSemaphoreDelete(remote_conn_queue);
            esp_now_deinit();
            return ESP_FAIL;
        }
        
        memcpy(pingpacket->dest_mac, &_aircraftMACAddr, ESP_NOW_ETH_ALEN);
        espnow_data_prepare(pingpacket);

        ESP_LOGI("PING", "PACKET READY!");
        free(pingpacket);
        ESP_LOGI("PING", "DELETING PACKET!");
    // //xTaskCreate(espnow_task, "espnow_task", 2048, send_param, 4, NULL);
        
        ESP_LOGI("PING", "PACKET TEST COMPLETED!");
    return ESP_OK;
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

/// @brief Convert throttle percentage to the PWM amount...
/// @param percentage 
/// @return 
static int ThrottlePercentToPWM(float percentage)
{
    return (int)(ENGINE_PWM_MIN + percentage/100.0f*(ENGINE_PWM_MAX-ENGINE_PWM_MIN)); 
}

/// @brief For updating the LCD screen with relevant information 
/// (I think it is rate-limited because the screen has a slower refresh rate than the other sub-systems)
/// @param param 
static void update_LCD(void* param)
{
    TickType_t lastTaskTime = xTaskGetTickCount();
    const TickType_t delay_time = pdMS_TO_TICKS(150);

    char YPRstr[16];
    LCD_disp("STATE 0          ",0,false);
    LCD_disp("NO-REMOTE        ",1,false);
    
    while(true)
    {
        switch (_state)
        {
            case 0:
                if(_remoteMACAddr[0] != 0 && _remoteMACAddr[1] != 0 && _remoteMACAddr[2] != 0)
                {
                    snprintf(YPRstr, sizeof(YPRstr), "%02X:%02X:%02X:%02X%02X%02X", _remoteMACAddr[0],_remoteMACAddr[1],_remoteMACAddr[2],_remoteMACAddr[3],_remoteMACAddr[4],_remoteMACAddr[5]);
                    LCD_disp(YPRstr,0,true);
                    LCD_disp("[TRIGGER]CONFIRM",1,false);    
                }
                else
                {
                    LCD_disp("STATE 0          ",0,false);
                    LCD_disp("NO-REMOTE        ",1,false);      
                }
                snprintf(YPRstr, sizeof(YPRstr), "Y%.1fP%.1fR%.1f", ypr[0]* 180/M_PI, ypr[1]* 180/M_PI, ypr[2]* 180/M_PI);
                LCD_disp(YPRstr,2,false);
                snprintf(YPRstr, sizeof(YPRstr), "Y%.1fP%.1fR%.1f", ypr2[0]* 180/M_PI, ypr2[1]* 180/M_PI, ypr2[2]* 180/M_PI);
                LCD_disp(YPRstr,3,false);
                break;
            case 1:
                //snprintf(YPRstr, sizeof(YPRstr), "%02X:%02X:%02X:%02X%02X%02X", _remoteMACAddr[0],_remoteMACAddr[1],_remoteMACAddr[2],_remoteMACAddr[3],_remoteMACAddr[4],_remoteMACAddr[5]);
                //LCD_disp(YPRstr,0,true);
                //snprintf(YPRstr, sizeof(YPRstr), "Y%.1fP%.1fR%.1f", ypr[0]* 180/M_PI, ypr[1]* 180/M_PI, ypr[2]* 180/M_PI);
                LCD_disp("STATE 1          ",1,false);
                LCD_disp("START ENGINES < >",2,true);
                LCD_disp("BUTTON [3] TO FLY",3,true);
                break;
            case 2:
                snprintf(YPRstr, sizeof(YPRstr), "%02X:%02X:%02X:%02X%02X%02X", _remoteMACAddr[0],_remoteMACAddr[1],_remoteMACAddr[2],_remoteMACAddr[3],_remoteMACAddr[4],_remoteMACAddr[5]);
                LCD_disp(YPRstr,0,true);
                snprintf(YPRstr, sizeof(YPRstr), "Y%.1fP%.1fR%.1f", ypr[0]* 180/M_PI, ypr[1]* 180/M_PI, ypr[2]* 180/M_PI);
                LCD_disp(YPRstr,1,false);
                snprintf(YPRstr, sizeof(YPRstr), "Y%.1fP%.1fR%.1f", ypr_target[0]* 180/M_PI, ypr_target[1]* 180/M_PI, ypr_target[2]* 180/M_PI);
                LCD_disp(YPRstr,2,true); 
                snprintf(YPRstr, sizeof(YPRstr), "T%.1fL%liB%d", _cmdThrottlePercentage, _lastCommunicationWithRemote, last_button_press);
                LCD_disp(YPRstr,3,false); 
                break;
            case 3: // As of now, unused case
                LCD_disp("REMOTE DISCONNECT",0,true);
                LCD_disp("TRY TO AUTO LAND!",1,false);
                LCD_disp("                 ",2,false);
                snprintf(YPRstr, sizeof(YPRstr), "T%.1fL%liS%d", _cmdThrottlePercentage, _lastCommunicationWithRemote, _state);
                LCD_disp(YPRstr,3,true); 
                break;
            case 4: // As of now, unused case
                LCD_disp("STATE 4          ",0,false);
                LCD_disp("CAL              ",1,false);
                LCD_disp("A                ",2,false);
                LCD_disp("ENG-OFF          ",3,true); 
                break;
            default:
                // It should never reach this case...
                break;
        }

        vTaskDelayUntil(&lastTaskTime, delay_time);
    }

    vTaskDelete(NULL);
}

/// @brief Check connection to remote control and process remote output
/// The longer delay should be ok as it is still much faster than a human reaction time
/// What really matters is that the control loop is fast enough, which is handled in other sections of the code... 
/// @param pvParam 
static void remote_conn(void* pvParam)
{
    TickType_t lastTaskTime = xTaskGetTickCount();
    const TickType_t delay_time = pdMS_TO_TICKS(44);

    espnow_event_t evt; // We use this in this task to check if connection exists...
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    int recv_magic = 0;
    bool is_broadcast = false;
    int ret;

    //vTaskDelay(pdMS_TO_TICKS(6000));

    while(true)
    {
        while(xQueueReceive(remote_conn_queue, &evt, 0))
        {
            if(evt.id == ESPNOW_RECV_CB) // Make sure the queue data is a reception
            {
                espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
                ESP_LOGI("Ping","Examining received message!");
                espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &last_packetID);
            }
            else if(evt.id == ESPNOW_SEND_CB)
            {
                ESP_LOGI("Ping", "Ping message was previously sent.");
            }
            else
            {
                ESP_LOGE("Ping", "Something strange happened!");
            }
        }

        _lastCommunicationWithRemote += pdTICKS_TO_MS(delay_time); // Increment counter
        if(_lastCommunicationWithRemote > CONN_TIMEOUT && _state == 2)
        {
            // Timed out
            _state = 3;
        }

        vTaskDelayUntil(&lastTaskTime, delay_time);
    }
    vTaskDelete(NULL);
}

/// @brief Actuation of motors and servos...
/// [Note:] This method will read from the global variables that were computed by LQI function.
/// @param pvParam 
static void motors(void* pvParam)
{
    TickType_t lastTaskTime = xTaskGetTickCount();
    const TickType_t delay_time = pdMS_TO_TICKS(21);

    while(true)
    {
        switch (_state)
        {
            case 0:
                //ESP_LOGI("[MOT-INFO]", "Motors idle - awaiting remote connection!");
                // Send no signal at all
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0);

                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
                break;
            case 1:
                //ESP_LOGI("[MOT-INFO]", "Connected to remote! Awaiting engine start...");
                //ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, _servoRight);
                //ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, _servoLeft);
                //ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
                //ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);

                // Open loop motor control testing fo today
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, _throttleRight);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, _throttleLeft);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, _servoRight);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, _servoLeft);

                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
                break;
            case 2: case 4:
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, _throttleRight);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, _throttleLeft);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, _servoRight);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, _servoLeft);

                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
                break;
            case 3: // Automatic landing
            {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, _throttleRight); // Gradual decrease throttle (handled in control loop)
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, _throttleLeft);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, SERVO_PWM_MID); // Force center of servos
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, SERVO_PWM_MID);

                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
            }
            default:
                // It should never reach this case...
                break;
        }
        vTaskDelayUntil(&lastTaskTime, delay_time);
    }
    vTaskDelete(NULL);
}

/// @brief Reading from IMUs
/// @param pvParam 
static void IMU(void* pvParam)
{
    TickType_t lastTaskTime = xTaskGetTickCount();
    const TickType_t delay_time = pdMS_TO_TICKS(IMU_PERIOD_MS);

    // Start IMU and initialize the DMP
    MPU6050 mpu = MPU6050(ADDR_IMU_F);
	mpu.initialize();
	mpu.dmpInitialize();

    // Calibrate IMU to get rid of offsets
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);

    // Enable digital motion processor so that we don't need to write our own quaternion filter
	mpu.setDMPEnabled(true);

    // Start 2nd IMU and initialize the DMP
    MPU6050 mpu2 = MPU6050(ADDR_IMU_R);
	mpu2.initialize();
	mpu2.dmpInitialize();

    // Calibrate IMU to get rid of offsets
    mpu2.CalibrateAccel(6);
    mpu2.CalibrateGyro(6);

    // Enable digital motion processor so that we don't need to write our own quaternion filter
	mpu2.setDMPEnabled(true);

    while(true)
    {
        mpuIntStatus = mpu.getIntStatus();
		// get current FIFO count
		fifoCount = mpu.getFIFOCount();

	    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
	        // reset so we can continue cleanly
	        mpu.resetFIFO();

	    // otherwise, check for DMP data ready interrupt frequently)
	    } else if (mpuIntStatus & 0x02) {
	        // wait for correct available data length, should be a VERY short wait
	        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

	        // read a packet from FIFO (Not sure how this works?? I didn't write it)
	        mpu.getFIFOBytes(fifoBuffer, packetSize);
	 		mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
            for (int i = 0; i < 3; i++) {ypr_old[i] = ypr[i];}
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            for (int i = 0; i < 3; i++) {ypr_prime[i] = (ypr[i] - ypr_old[i])/((float)IMU_PERIOD_MS/1000.0f);}
	    }

	    //Best result is to match with DMP refresh rate
	    // Its last value in components/MPU6050/MPU6050_6Axis_MotionApps20.h file line 310
	    // Now its 0x13, which means DMP is refreshed with 10Hz rate
		// vTaskDelay(5/portTICK_PERIOD_MS);

        mpuIntStatus2 = mpu2.getIntStatus();
		// get current FIFO count
		fifoCount2 = mpu2.getFIFOCount();

	    if ((mpuIntStatus2 & 0x10) || fifoCount2 == 1024) {
	        // reset so we can continue cleanly
	        mpu2.resetFIFO();

	    // otherwise, check for DMP data ready interrupt frequently)
	    } else if (mpuIntStatus2 & 0x02) {
	        // wait for correct available data length, should be a VERY short wait
	        while (fifoCount2 < packetSize2) fifoCount2 = mpu2.getFIFOCount();

	        // read a packet from FIFO (Not sure how this works?? I didn't write it)
	        mpu2.getFIFOBytes(fifoBuffer2, packetSize2);
	 		mpu2.dmpGetQuaternion(&q2, fifoBuffer2);
			mpu2.dmpGetGravity(&gravity2, &q2);
            
			mpu2.dmpGetYawPitchRoll(ypr2, &q2, &gravity2);
	    }

        vTaskDelayUntil(&lastTaskTime, delay_time);
	}

    vTaskDelete(NULL);
}

/// @brief Update the I2C buffer (for datalink computer to draw from)
/// @param pvParam 
static void I2CBufferUpdate(void* pvParam)
{
    TickType_t lastTaskTime = xTaskGetTickCount();
    const TickType_t delay_time = pdMS_TO_TICKS(96);

    DL_debug_info_t *send_data = (DL_debug_info_t *) malloc(sizeof(DL_debug_info_t));
    size_t d_size = 0;

    while(true)
    {
        switch(_state)
        {
            case 0:
            case 1:
            case 2:
            case 3:
            case 4:
                send_data->state = _state;

                send_data->ypr[0] = ypr[0];
                send_data->ypr[1] = ypr[1];
                send_data->ypr[2] = ypr[2];

                send_data->throttle = _cmdThrottlePercentage;
                send_data->buttonPress = last_button_press;
                send_data->hatSwitch = last_hat_switch;

                send_data->ypr_cmd[0] = ypr_target[0];
                send_data->ypr_cmd[1] = ypr_target[1];
                send_data->ypr_cmd[2] = ypr_target[2];

                send_data->dypr[0] = ypr_prime[0];
                send_data->dypr[1] = ypr_prime[1];                
                send_data->dypr[2] = ypr_prime[2];

                send_data->integral[0] = ypr_accum[0];
                send_data->integral[1] = ypr_accum[1];
                send_data->integral[2] = ypr_accum[2]; 
                
                send_data->motor_PWM[0] = _throttleLeft;
                send_data->motor_PWM[1] = _throttleRight;

                send_data->servo_PWM[0] = _servoLeft;
                send_data->servo_PWM[1] = _servoRight;

                d_size = i2c_slave_write_buffer(I2C_NUM_1, (uint8_t *)send_data, sizeof(DL_debug_info_t), pdMS_TO_TICKS(25));
                if (d_size == 0) {
                    ESP_LOGW("I2C", "i2c slave tx buffer full");
                }
                break;
            default: // Should never reach this!!!!
                break;
        }

        vTaskDelayUntil(&lastTaskTime, delay_time);
	}
    free(send_data);
    vTaskDelete(NULL);
}

/// @brief Control loop (not actually LQI tho)
/// @param pvParam 
static void LQI(void* pvParam)
{
    TickType_t lastTaskTime = xTaskGetTickCount();
    const TickType_t delay_time = pdMS_TO_TICKS(LQI_PERIOD_MS);
    float u[4] = {0, 0, 0, 0};
    float x[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

    bool engine1On = false; bool engine2On = false; 
    bool servo1On = false; bool servo2On = false;

    _throttleLeft = 0;
    _throttleRight = 0;

    while(true)
    {
        switch (_state)
        {
            case 0:
                // Do nothing (no control happens while the system is off)
                break;
            case 1: // Await all engine ignition!
                servo1On = true; _servoLeft = SERVO_PWM_MID;
                servo2On = true; _servoRight = SERVO_PWM_MID;

                switch(last_hat_switch)
                {
                    case 6:
                       engine1On = true; _throttleLeft = ENGINE_PWM_MIN;
                    break;
                    case 2:
                       engine2On = true; _throttleRight = ENGINE_PWM_MIN;
                    break;
                }
                switch(last_button_press)
                {
                    case 4:
                        if(_cmdThrottlePercentage < 10) // Safety check, make sure throttle is low!!
                            _state = 2;
                        else
                            LCD_disp("ALERT: THROTTLE DOWN!", 0, true);
                    break;
                }

                break;
            case 2: 
                // Manual control!! Skill needed... NOTE: Yaw has large values to compensate for asymmetry of actuators (use with caution!)
                _servoRight = -200 * ypr_target[0] * (SERVO_PWM_MAX - SERVO_PWM_MIN) / 2 - 120 * ypr_target[1] * (SERVO_PWM_MAX - SERVO_PWM_MIN) / 2 + SERVO_PWM_MID;
                _servoLeft = -200 * ypr_target[0] * (SERVO_PWM_MAX - SERVO_PWM_MIN) / 2 + 120 * ypr_target[1] * (SERVO_PWM_MAX - SERVO_PWM_MIN) / 2 + SERVO_PWM_MID;

                // Manual control!! Skill needed...
                _throttleRight = _cmdThrottlePercentage/100 * (ENGINE_PWM_MAX - ENGINE_PWM_MIN) * 4 / 5 - 25 * ypr_target[2] * (ENGINE_PWM_MAX - ENGINE_PWM_MIN) / 5 + ENGINE_PWM_MIN;
                _throttleLeft = _cmdThrottlePercentage/100 * (ENGINE_PWM_MAX - ENGINE_PWM_MIN) * 4 / 5 + 25 * ypr_target[2] * (ENGINE_PWM_MAX - ENGINE_PWM_MIN) / 5 + ENGINE_PWM_MIN; 
                break;
            case 3:
                if(_lastCommunicationWithRemote < CONN_TIMEOUT)
                {
                    _state = 2;
                    break;
                }
                _servoRight = SERVO_PWM_MID; _servoLeft = SERVO_PWM_MID;
                if(_cmdThrottlePercentage > 25)
                    _cmdThrottlePercentage = _cmdThrottlePercentage - 10.0f * (float)LQI_PERIOD_MS/(float)(1000);
                else
                {
                    _cmdThrottlePercentage = 0;
                    _state = 0;
                }
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
extern "C" void app_main(void)
{
    i2c_DL_init(); // Start I2C bus to data link with error check
    i2c_FC_init(); // Start I2C bus to flight computer with error check

    LCD_init();
    
    LCD_disp("LCD_init OK    ",0,false); // Confirm successful I2C start
    vTaskDelay(500/portTICK_PERIOD_MS);
    LCD_disp("I2C_init OK    ",1,false);

    LCD_disp("ESP_NOW_init OK",2,false);

    vTaskDelay(500/portTICK_PERIOD_MS);
    LCD_disp("START TASKS    ",3,false);
   
    vTaskDelay(1000/portTICK_PERIOD_MS);

    // setup motor and servo PWMs
    if(ledc_timer_config(&pwm_timer) == ESP_OK)
	{
		ESP_LOGI("[MOT-INFO]", "PWM timer inited successfully");
	}
	else
	{
		ESP_LOGE("[MOT-ERR]", "PWM timer failed to init");
	}

    if(ledc_channel_config(&motor1_channel) == ESP_OK)
	{
		ESP_LOGI("[MOT-INFO]", "Motor 1 PWM channel OK");
	} else {
        ESP_LOGE("[MOT-ERR]", "Motor 1 PWM channel FAILED");
    }

    if(ledc_channel_config(&motor2_channel) == ESP_OK)
	{
		ESP_LOGI("[MOT-INFO]", "Motor 2 PWM channel OK");
	} else {
        ESP_LOGE("[MOT-ERR]", "Motor 2 PWM channel FAILED");
    }

    if(ledc_channel_config(&servo1_channel) == ESP_OK)
	{
		ESP_LOGI("[MOT-INFO]", "Servo 1 PWM channel OK");
	} else {
        ESP_LOGE("[MOT-ERR]", "Servo 1 PWM channel FAILED");
    }

    if(ledc_channel_config(&servo2_channel) == ESP_OK)
	{
		ESP_LOGI("[MOT-INFO]", "Servo 2 PWM channel OK");
	} else {
        ESP_LOGE("[MOT-ERR]", "Servo 2 PWM channel FAILED");
    }

    xTaskCreatePinnedToCore(IMU, "IMU", 4096, (void*) 1, 8, NULL, 0); // Task for reading IMU sensors and actually other I2C tasks!
    
    vTaskDelay(pdMS_TO_TICKS(1500));
    
    wifi_init();
    ESP_NOW_init();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    xTaskCreatePinnedToCore(remote_conn, "remote_conn", 4096, (void*) 1, 8, NULL, 0); // Task for remote control (will listen in the background)
    xTaskCreatePinnedToCore(I2CBufferUpdate, "I2CBufferUpdate", 4096, (void*) 1, 6, NULL, 0); // Task for updating I2C slave buffer
    xTaskCreatePinnedToCore(update_LCD, "update_LCD", 4096, (void*) 1, 2, NULL, 0); // Task for screen update

    xTaskCreatePinnedToCore(motors, "motors", 4096, (void*) 1, 12, NULL, 1); // Motor actuation (core 1) - High priority
                                                                             // can interrupt controls since it is slower
    xTaskCreatePinnedToCore(LQI, "LQI", 4096, (void*) 1, 8, NULL, 1); // Control algorithm (core 1) 
}
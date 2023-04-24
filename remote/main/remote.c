/* 
    Remote for twin rotor helicopter
    Last updated 4/14/2023 - Now uses the USB joystick "Logitech Extreme 3D Pro"
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
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

#include "usb/usb_host.h"
#include "fs_hid_host.h"
#include "fs_hid_usage_keyboard.h"
#include "fs_hid_usage_mouse.h"
#include "fs_hid_usage_logi_flightstick.h"

#include "espn.h"

//#include "i2cdev.h"
#include "driver/i2c.h"

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
uint8_t _remoteMACAddr[6] = {00,00,00,00,00,00}; // MAC address of remote (this MAC address)
uint8_t _aircraftMACAddr[6] = {00,00,00,00,00,00}; // MAC address of aircraft (will be detected during remote ping/proximity scan)

static uint16_t s_espnow_seq[ESPNOW_DATA_MAX] = { 0, 0 };

#define ESPNOW_MAXDELAY 512
static QueueHandle_t remote_conn_queue; // The queue containing messages to send to and receive from the FC
static QueueHandle_t potential_connections; // The queue containing potential remote addresses

// The commanded angles and yaw rate will be broadcasted to the aircraft
volatile float _cmdPitch = 0; // Pitch will be an absolute value (0 = level)
volatile float _cmdRoll = 0; // Roll will be an absolute value (0 = level)
volatile float _cmdYawRate = 0; // Yaw will be only a rate because it cannot be really be stabilized with the onboard sensors
volatile float _cmdThrottlePercentage = 0; // Throttle strength in percent

// Variables with regards to USB flight stick...
#define APP_QUIT_PIN                GPIO_NUM_0
#define APP_QUIT_PIN_POLL_MS        500

#define READY_TO_UNINSTALL          (HOST_NO_CLIENT | HOST_ALL_FREE)

volatile uint8_t lastButtonPress = 0; // Joystick button, trigger = 1, BTN 2 = 2, BTN 3 = 4, BTN 4 = 8, BTN 5 = 10, BTN 6 = 20
volatile uint8_t lastHatSwitchPress = 8; // Joystick hat switch, neutral = 8, right = 2, left = 6, up = 0, down = 4
volatile uint8_t throttlePos = 0xFF; // Throttle position, for some reason is reversed, 0xFF is zero pos, 0 is max
volatile uint16_t lastStickR = 0x1FF; // Joystick roll axis, 0x0 is left, 0x1FF is center, 0x3FF is right
volatile uint16_t lastStickP = 0x1FF; // Joystick pitch axis, 0x0 is forward, 0x1FF is center, 0x3FF is back
volatile uint16_t lastStickY = 0x80; // Joystick yaw axis, 0x0 is left, 0x80 is center, 0xFF is right

volatile hid_stick_input_report_boot_t last_stick_report; // Previous message... (initialize with all 0)

/**
 * @brief Application Event from USB Host driver
 *
 */
typedef enum {
    HOST_NO_CLIENT = 0x1,
    HOST_ALL_FREE = 0x2,
    DEVICE_CONNECTED = 0x4,
    DEVICE_DISCONNECTED = 0x8,
    DEVICE_ADDRESS_MASK = 0xFF0,
} app_event_t;

#define USB_EVENTS_TO_WAIT      (DEVICE_CONNECTED | DEVICE_ADDRESS_MASK | DEVICE_DISCONNECTED)

//static const char *TAG = "example";
static EventGroupHandle_t usb_flags;
static bool is_hid_device_connected = false;
static hid_host_interface_handle_t mouse_handle = NULL; 

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
    
    //i2c_set_timeout(I2C_NUM_0,40000);
}

/// @brief Suspect that USB initialization interrupts I2C and leaves I2C slave in control of line
/// then may require this to reset...
static void i2c_refresh(){
    //i2c_master_clear_bus(I2C_NUM_0); 
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

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    espnow_event_t evt;
    espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL) {
        //ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(remote_conn_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        //ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    espnow_event_t evt;
    espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        //ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    evt.id = ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = (uint8_t *) malloc(len);
    if (recv_cb->data == NULL) {
        //ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(remote_conn_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        //ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

/* Parse received ESPNOW data. */
int espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, int *magic)
{
    espnow_data_t *buf = (espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(espnow_data_t)) {
        //ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *state = buf->state;
    *seq = buf->seq_num;
    *magic = buf->magic;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc) {
        return buf->type;
    }

    return -1;
}

/* Prepare ESPNOW data to be sent. */
void espnow_data_prepare(espnow_send_param_t *send_param)
{
    espnow_data_t *buf = (espnow_data_t *)send_param->buffer;

    assert(send_param->len >= sizeof(espnow_data_t));

    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? ESPNOW_DATA_BROADCAST : ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->magic = send_param->magic;
    /* Fill all remaining bytes after the data with random values */
    esp_fill_random(buf->payload, send_param->len - sizeof(espnow_data_t));
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}

static esp_err_t ESP_NOW_init()
{
    esp_read_mac(_remoteMACAddr, ESP_MAC_WIFI_STA); // For use in display of own MAC address

    espnow_send_param_t *send_param;

    remote_conn_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (remote_conn_queue == NULL) {
        //ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(espnow_recv_cb) );
#if CONFIG_ESP_WIFI_STA_DISCONNECTED_PM_ENABLE
    ESP_ERROR_CHECK( esp_now_set_wake_window(65535) );
#endif
    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = (esp_now_peer_info_t *) malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        //ESP_LOGE(TAG, "Malloc peer information fail");
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

    // /* Initialize sending parameters. */
    // send_param = (espnow_send_param_t *) malloc(sizeof(espnow_send_param_t));
    // if (send_param == NULL) {
    //     //ESP_LOGE(TAG, "Malloc send parameter fail");
    //     vSemaphoreDelete(remote_conn_queue);
    //     esp_now_deinit();
    //     return ESP_FAIL;
    // }
    // memset(send_param, 0, sizeof(espnow_send_param_t));
    // send_param->unicast = false;
    // send_param->broadcast = true;
    // send_param->state = 0;
    // send_param->magic = esp_random();
    // send_param->count = CONFIG_ESPNOW_SEND_COUNT;
    // send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
    // send_param->len = CONFIG_ESPNOW_SEND_LEN;
    // send_param->buffer = (uint8_t*) malloc(CONFIG_ESPNOW_SEND_LEN);
    // if (send_param->buffer == NULL) {
    //     //ESP_LOGE(TAG, "Malloc send buffer fail");
    //     free(send_param);
    //     vSemaphoreDelete(remote_conn_queue);
    //     esp_now_deinit();
    //     return ESP_FAIL;
    // }
    // memcpy(send_param->dest_mac, _remoteMACAddr, ESP_NOW_ETH_ALEN);
    // espnow_data_prepare(send_param);

    // //xTaskCreate(espnow_task, "espnow_task", 2048, send_param, 4, NULL);

    return ESP_OK;
}

/**
 * @brief Makes new line depending on report output protocol type
 *
 * @param[in] proto Current protocol to output
 */
static void hid_print_new_device_report_header(hid_protocol_t proto)
{
    static hid_protocol_t prev_proto_output = HID_PROTOCOL_NONE;
    if (prev_proto_output != proto) {
        prev_proto_output = proto;
        printf("\r\n");
        printf(":%02X",proto);
        fflush(stdout);
    }
    ESP_LOGI("HID_NEW_DEVICE_REPORT","AAA");
}

/**
 * @brief USB HID Host Mouse Interface report callback handler
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_mouse_report_callback(const uint8_t *const data, const int length)
{ 
    hid_mouse_input_report_boot_t *mouse_report = (hid_mouse_input_report_boot_t *)data;

    if (length < sizeof(hid_mouse_input_report_boot_t)) {
        return;
    }

    static int x_pos = 0;
    static int y_pos = 0;

    // Calculate absolute position from displacement
    x_pos += mouse_report->x_displacement;
    y_pos += mouse_report->y_displacement;

    hid_print_new_device_report_header(HID_PROTOCOL_MOUSE);

    printf("X: %06d\tY: %06d\t|%c|%c|\r",
           x_pos, y_pos,
           (mouse_report->buttons.button1 ? 'o' : ' '),
           (mouse_report->buttons.button2 ? 'o' : ' '));
    fflush(stdout);
}

/**
 * @brief The joystick callback handler...
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
*/
static void hid_flight_stick_cb(const uint8_t *const dat, const int length)
{
    hid_stick_input_report_boot_t *stick_rep = (hid_stick_input_report_boot_t *)dat;

    if(length < sizeof(hid_stick_input_report_boot_t))
    {
        ESP_LOGE("FS-CB", "Wrong size!");
         return;
    }

    last_stick_report = *stick_rep;
    //printf("P %X, R %X, Y %X, T %X, H %X, BA %X, BB %X", stick_rep->x, stick_rep->y, stick_rep->twist, stick_rep->slider, stick_rep->hat, stick_rep->buttons_a, stick_rep->buttons_b);
    //fflush(stdout);
}

/**
 * @brief USB HID Host event callback. Handle such event as device connection and removing
 *
 * @param[in] event  HID device event
 * @param[in] arg    Pointer to arguments, does not used
 */
void hid_host_event_callback(const hid_host_event_t *event, void *arg)
{
    if (event->event == HID_DEVICE_CONNECTED) {
        // Obtained USB device address is placed after application events
        ESP_LOGI("USB-DETEC","DEVADDRESS: %X", event->device.address);
        xEventGroupSetBits(usb_flags, DEVICE_CONNECTED | (event->device.address << 4));
    } else if (event->event == HID_DEVICE_DISCONNECTED) {
        ESP_LOGI("USB-DISCONN","DEVADDRESS: %X", event->device.address);
        xEventGroupSetBits(usb_flags, DEVICE_DISCONNECTED);
    }
}

/**
 * @brief USB HID Host interface callback
 *
 * @param[in] event  HID interface event
 * @param[in] arg    Pointer to arguments, does not used
 */
void hid_host_interface_event_callback(const hid_host_interface_event_t *event2, void *arg)
{
    ESP_LOGI("HID_HOST_EVENT","Proto: %X", event2->interface.proto);

    switch (event2->event) {
    case HID_DEVICE_INTERFACE_INIT:
        if (event2->interface.proto == HID_PROTOCOL_MOUSE) {
            const hid_host_interface_config_t hid_mouse_config = {
                .proto = HID_PROTOCOL_MOUSE,
                .callback = hid_host_mouse_report_callback,
            };

            hid_host_claim_interface(&hid_mouse_config, &mouse_handle);
        }
        else // Force the system to select flight stick configuration, we know what it is
        {
            ESP_LOGI("HID_HOST_EVENT","Claim interface attempt");
            const hid_host_interface_config_t hid_stick_config = {
                .proto = event2->interface.proto, // Aviation stick
                .callback = hid_flight_stick_cb,
            };

            hid_host_claim_interface(&hid_stick_config, &mouse_handle);
        }

        break;
    case HID_DEVICE_INTERFACE_TRANSFER_ERROR:
        ESP_LOGD("PPS", "Interface number %d, transfer error",
                 event2->interface.num);
        break;

    case HID_DEVICE_INTERFACE_CLAIM:
    case HID_DEVICE_INTERFACE_RELEASE:
        ESP_LOGI("PPS", "Claim/Release");
        break;

    default:
        ESP_LOGI("PPS", "%s Unhandled event %X, Interface number %d",
                 __FUNCTION__,
                 event2->event,
                 event2->interface.num);
        break;
    }
}

/**
 * @brief Handle common USB host library events
 *
 * @param[in] args  Pointer to arguments, does not used
 */
void handle_usb_events(void *args)
{
    uint32_t event_flags;

    while (1) {
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

        // Release devices once all clients has deregistered
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
            xEventGroupSetBits(usb_flags, HOST_NO_CLIENT);
        }
        // Give ready_to_uninstall_usb semaphore to indicate that USB Host library
        // can be deinitialized, and terminate this task.
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            xEventGroupSetBits(usb_flags, HOST_ALL_FREE);
        }

        //vTaskDelay(25);
    }

    vTaskDelete(NULL);
}

bool wait_for_event(EventBits_t event, TickType_t timeout)
{
    return xEventGroupWaitBits(usb_flags, event, pdTRUE, pdTRUE, timeout) & event;
}

static void LCD_disp(char *msg, uint8_t line, bool backlight)
{   
    //i2c_refresh();
    ssd1306_display_text(&screen, line, msg, 20, backlight);
}

static void LCD_clear_line(uint8_t line)
{
    //i2c_refresh();
    ssd1306_clear_line(&screen, line, false);    
}
/// @brief LCD screen
static void LCD_init()
{
    screen._address = ADDR_LCD_SCREEN;
    i2c_init(&screen, 128, 32);

    LCD_clear_line(0);
    LCD_clear_line(1);
    LCD_clear_line(2);
    LCD_clear_line(3);
}

/// @brief Searching for the flight computer by sending out pings every 2.5 seconds
/// @param param 
static void esp_now_ping(void* param)
{
    TickType_t lastTaskTime = xTaskGetTickCount();
    const TickType_t delay_time = pdMS_TO_TICKS(2500);
    
    potential_connections = xQueueCreate(8,sizeof(_remoteMACAddr));

    char seensofar[16];

    uint8_t num_potential_conn = 0;

    //LCD_clear_line(1);

    while(true)
    {
        if(state == 2 || state == 1)
        {
            // Broadcast message and wait for response...
            ESP_LOGI("Ping","Pong");
            num_potential_conn = uxQueueMessagesWaiting(potential_connections); // Number of devices currently found
            
            if(num_potential_conn > 0)
            {
                ESP_LOGI("ESPN","Found %02X device", num_potential_conn);
                state = 2;
            }
            else
            {
                //LCD_disp("NONE FOUND YET",1,false);
                ESP_LOGI("ESPN","None found yet!");
            }

            ESP_LOGI("ESPN","Preparing to broadcast payload for discovery!");

                
    /* Initialize sending parameters. */
    espnow_send_param_t *pingpacket = (espnow_send_param_t *) malloc(sizeof(espnow_send_param_t));
    if (pingpacket == NULL) {
        //ESP_LOGE(TAG, "Malloc send parameter fail");
        vSemaphoreDelete(remote_conn_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
        memset(pingpacket, 0, sizeof(espnow_send_param_t));
        pingpacket->unicast = false;
        pingpacket->broadcast = true;
        pingpacket->state = 0;
        pingpacket->magic = esp_random();
        pingpacket->count = 25;
        pingpacket->delay = 10/portTICK_PERIOD_MS;
        pingpacket->len = CONFIG_ESPNOW_SEND_LEN;
        pingpacket->buffer = (uint8_t*) malloc(CONFIG_ESPNOW_SEND_LEN);

        if (pingpacket->buffer == NULL) {
            //ESP_LOGE(TAG, "Malloc send buffer fail");
            free(pingpacket);
            vSemaphoreDelete(remote_conn_queue);
            esp_now_deinit();
            return ESP_FAIL;
        }
        
        memcpy(pingpacket->dest_mac, _remoteMACAddr, ESP_NOW_ETH_ALEN);
        espnow_data_prepare(pingpacket);

        // TODO: Send the ping

        vTaskDelayUntil(&lastTaskTime, delay_time);
    }
    
    vTaskDelete(NULL);
}

/// @brief Check if we should transmit to flight computer every 50 ms (if there is something to send in queue)
/// @param param 
static void esp_now_transmit(void* param)
{
    TickType_t lastTaskTime = xTaskGetTickCount();
    const TickType_t delay_time = pdMS_TO_TICKS(50);
    
    //potential_connections = xQueueCreate(8,sizeof(_remoteMACAddr));

    char seensofar[16];

    uint8_t num_potential_conn = 00;

    //LCD_clear_line(1);

    while(true)
    {
        if(state == 3) // Transmit mode is state 3 - we will broadcast and also take pingbacks as acknowledgements
        {
            // Broadcast message and wait for response...
            ESP_LOGI("Ping","Pong");
            //num_potential_conn = uxQueueMessagesWaiting(potential_connections); // Number of devices currently found
            
            // if(num_potential_conn > 0)
            // {
            //     snprintf(seensofar, sizeof(seensofar), "Found %02X device", num_potential_conn);
            //     state = 2;
            // }
            // else
            // {
            //     snprintf(seensofar, sizeof(seensofar), "None found yet!");
            // }

            //LCD_disp(seensofar,1,false);
            //vTaskDelay(delay_time);
        }
        vTaskDelayUntil(&lastTaskTime, delay_time);
    }
    
    vTaskDelete(NULL);
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

    char otherMACAddr[16];
    snprintf(otherMACAddr, sizeof(otherMACAddr), "%02X:%02X:%02X:%02X%02X%02X", _aircraftMACAddr[0],_aircraftMACAddr[1],_aircraftMACAddr[2],_aircraftMACAddr[3],_aircraftMACAddr[4],_aircraftMACAddr[5]);

    // Hardware compatibility issue with I2C and USB host means that we can only show static message...
    LCD_disp("That MAC ADDRESS",1,false);
    LCD_disp("should appear on",2,false);
    LCD_disp("aircraft display",3,false);
    LCD_disp(thisMACAddr,0,true);

    vTaskDelay(1000/portTICK_PERIOD_MS);
    vTaskDelete(NULL);
}

void parseInput() // The input parsing test function
{
    //hid_print_new_device_report_header(HID_PROTOCOL_NONE);
    if(lastStickP != last_stick_report.y)
    {
        _cmdPitch = (float)last_stick_report.y - 0x1FF;
        _cmdPitch /= 0x3FF;
        _cmdPitch *= 150/M_PI;
        ESP_LOGI("CMDPITCH","%.02f",_cmdPitch);
    }
    if(lastStickR != last_stick_report.x)
    {
        _cmdRoll = (float)last_stick_report.x - 0x1FF;
        _cmdRoll /= 0x3FF;
        _cmdRoll *= 150/M_PI;
        ESP_LOGI("CMDROLL","%.02f",_cmdRoll);
    }
    if(lastStickY != last_stick_report.twist)
    {
        _cmdYawRate = (float)last_stick_report.twist - 0x80;
        _cmdYawRate /= 0xFF;
        _cmdYawRate *= 150/M_PI;
        ESP_LOGI("CMDYAWRATE","%.02f",_cmdYawRate);
    }
    if(throttlePos != last_stick_report.slider)
    {
        _cmdThrottlePercentage = 0xFF - (float)last_stick_report.slider;
        _cmdThrottlePercentage = _cmdThrottlePercentage/255;
        ESP_LOGI("CMDTHRTL","%.02f",_cmdThrottlePercentage);
    }
    if(lastHatSwitchPress != last_stick_report.hat)
    {
        switch (last_stick_report.hat)
        {
        case 0:
            ESP_LOGI("HAT","UP");
            break;
        
        case 2:
            ESP_LOGI("HAT","RIGHT");
            break;
        
        case 4:
            ESP_LOGI("HAT","DOWN");
            break;
        
        case 6:
            ESP_LOGI("HAT","LEFT");
            break;
        
        case 8:
            ESP_LOGI("HAT","CENTER");
            break;
        
        default:
            break;
        }
    }
    if(lastButtonPress != last_stick_report.buttons_a)
    {
        switch (last_stick_report.buttons_a)
        {
        case 0:
            ESP_LOGI("BTN","NONE");
            break;
        
        case 1:
            ESP_LOGI("BTN","TRIGGER");
            break;
        
        case 2:
            ESP_LOGI("BTN","[2]");
            break;
        
        case 4:
            ESP_LOGI("BTN","[3]");
            break;
        
        case 8:
            ESP_LOGI("BTN","[4]");
            break;

        case 10:
            ESP_LOGI("BTN","[5]");
            break;

        case 20:
            ESP_LOGI("BTN","[6]");
            break;
        default:
            break;
        }
    }

    lastStickR = last_stick_report.x;
    lastStickP = last_stick_report.y;
    lastStickY = last_stick_report.twist;
    throttlePos = last_stick_report.slider;
    lastHatSwitchPress = last_stick_report.hat;
    lastButtonPress = last_stick_report.buttons_a;
}

/// @brief Check connection to remote control (Every 25 ms) 
/// @param pvParam 
static void remote_conn(void* pvParam)
{
    TickType_t lastTaskTime = xTaskGetTickCount();
    const TickType_t delay_time = pdMS_TO_TICKS(25);

    TaskHandle_t search_for_remote_task=NULL; // The esp_now_ping task
    //uint32_t ram_remaining = xPortGetFreeHeapSize();
    
    while(true)
    {
        //ESP_LOGI("RAM USAGE","N");
        switch (state)
        {
            case 0:
               // ESP_LOGI("[RC-INFO]", "Awaiting connection!");
              
                if(is_hid_device_connected)
                {
                    state = 1; // Move to state 1 once an USB connection is identified...
                }
                break;
            case 1:
                //parseInput(); 
                break;
            case 2:
                // Collected multiple potential in queue, now we want to select the right one to connect to
                break;
            case 3:
                // In this case, we are connected to the FC and all we need is to run pre-flight routine
                // (engine is off)
                break;
            case 4:
                // In this case, we are now actively controlling flight...
                break;
            default:
                // It should never reach this case...
                break;
        }
        vTaskDelayUntil(&lastTaskTime, delay_time);
        //vTaskDelay(delay_time);
    }
    vTaskDelete(NULL);
}

void usb_init()
{

}

static void usb_core_task(void* p)
{
    TaskHandle_t usb_events_task_handle;
    hid_host_device_handle_t hid_device;

    BaseType_t task_created;

    ESP_LOGI("AAAAAA", "HID HOST example");

    usb_flags = xEventGroupCreate();
    assert(usb_flags);
    ESP_LOGI("AAAAAA", "HID HOST 2 example");

    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1
    };

    ESP_LOGI("AAAAAA", "HID HOST 3example");

    ESP_ERROR_CHECK( usb_host_install(&host_config) );
    vTaskDelay(pdMS_TO_TICKS(105));
    task_created = xTaskCreatePinnedToCore(handle_usb_events, "usb_events", 4096, NULL, 8, &usb_events_task_handle, 1);
    assert(task_created);
    
    ESP_LOGI("AAAAAA", "HID HOST 4 example");

    // hid host driver config
    const hid_host_driver_config_t hid_host_config = {
        .create_background_task = true,
        .task_priority = 8,
        .stack_size = 4096,
        .core_id = 1,
        .callback = hid_host_event_callback,
        .callback_arg = NULL
    };

    ESP_LOGI("AAAAAA", "HID HOST 5example");
    vTaskDelay(pdMS_TO_TICKS(105));
    
    ESP_ERROR_CHECK( hid_host_install(&hid_host_config) );
    vTaskDelay(pdMS_TO_TICKS(105));
    
    ESP_LOGI("AAAAAA", "HID HOST 6example");

    EventBits_t event;// = xEventGroupWaitBits(usb_flags, USB_EVENTS_TO_WAIT, pdTRUE, pdFALSE, pdMS_TO_TICKS(725));

    do {
        event = xEventGroupWaitBits(usb_flags, USB_EVENTS_TO_WAIT, pdTRUE, pdFALSE, pdMS_TO_TICKS(725));

        //ESP_LOGI("AAAAAA", "HID HOST example");

        if (event & DEVICE_CONNECTED) {
            ESP_LOGI("PPPPPP", "USB device connected...!");
            xEventGroupClearBits(usb_flags, DEVICE_CONNECTED);
            is_hid_device_connected = true;
            ESP_LOGI("PPPPPP", "USB device connected...!");
        }

        if (event & DEVICE_ADDRESS_MASK) {
            
            ESP_LOGI("PPPPP2P", "USB device address %X", (uint8_t)(event & (DEVICE_ADDRESS_MASK >> 4)));
            xEventGroupClearBits(usb_flags, DEVICE_ADDRESS_MASK);
            const hid_host_device_config_t hid_host_device_config = {
                // Don't know why bit shifting gives a 14, should be 1 for device address, so it will be hardcoded
                .dev_addr = 0x1, // (uint8_t)(event & (DEVICE_ADDRESS_MASK >> 4),
                .iface_event_cb = hid_host_interface_event_callback,
                .iface_event_arg = NULL,
            };

            ESP_ERROR_CHECK( hid_host_install_device(&hid_host_device_config, &hid_device) );

            const usb_intf_desc_t ifd = {
                .bLength = 0x09,
                .bDescriptorType = 0x04,
                .bInterfaceNumber = 0x00,
                .bAlternateSetting = 0x00,
                .bNumEndpoints = 0x01,
                .bInterfaceClass = 0x03,
                .bInterfaceSubClass = 0x01,
                .bInterfaceProtocol = 0x00,
                .iInterface = 0x00
            };

            const hid_descriptor_t hidd = {
                .bLength = 0x09,
                .bDescriptorType = 0x21,
                .bcdHID = 0x0111,
                .bCountryCode = 0x00,
                .bNumDescriptors = 0x01,
                .bReportDescriptorType = 0x22,
                .wReportDescriptorLength = 0x007A
            };
         
            const usb_ep_desc_t espdesc = {
                .bLength = 0x07,
                .bDescriptorType = 0x05,
                .bEndpointAddress = 0x81,
                .bmAttributes = 0x03,
                .wMaxPacketSize = 0x0007,
                .bInterval = 0x01
            };

            ESP_ERROR_CHECK( create_interface_new(&hid_host_device_config, &hid_device, &ifd, &hidd, &espdesc));
            // TODO: ADD INTERFACE 
            ESP_LOGI("PPPPP2P", "USB device address %X", hid_host_device_config.dev_addr);
        }

        if (event & DEVICE_DISCONNECTED) {
            
            ESP_LOGI("PPPPP3P", "Lost USB connection...!");
            xEventGroupClearBits(usb_flags, DEVICE_DISCONNECTED);

            hid_host_release_interface(mouse_handle);

            ESP_ERROR_CHECK( hid_host_uninstall_device(hid_device) );

            is_hid_device_connected = false;
            
            ESP_LOGI("PPPPP3P", "Unloaded USB device...!");
        }

        vTaskDelay(pdMS_TO_TICKS(22));
    } while (true);

    if (is_hid_device_connected) {
        ESP_LOGI("PPS", "Uninitializing HID Device");
        hid_host_release_interface(mouse_handle);
        ESP_ERROR_CHECK( hid_host_uninstall_device(hid_device) );
        is_hid_device_connected = false;
    }

    ESP_LOGI("PPS", "Uninitializing USB");
    ESP_ERROR_CHECK( hid_host_uninstall() );
    wait_for_event(READY_TO_UNINSTALL, portMAX_DELAY);
    ESP_ERROR_CHECK( usb_host_uninstall() );
    vTaskDelete(usb_events_task_handle); // ???
    vEventGroupDelete(usb_flags);
    ESP_LOGI("PPS", "Done");
    vTaskDelete(NULL);
}

/// @brief Create all of the tasks
/// @param  
/*extern "C" */void app_main(void)
{
    mouse_handle = NULL;

    i2c_R_init(); // Start I2C bus to flight computer with error check
    
    vTaskDelay(500/portTICK_PERIOD_MS);
    LCD_init(); // Start LCD display
    
    LCD_disp("LCD_init OK    ",0,false); // Confirm successful I2C start
    
    vTaskDelay(500/portTICK_PERIOD_MS);
    LCD_disp("I2C_init OK    ",1,false);

    wifi_init(); // Start WIFI ANTENNA
    vTaskDelay(500/portTICK_PERIOD_MS);
    LCD_disp("WIFI_init OK   ",2,false);

    vTaskDelay(500/portTICK_PERIOD_MS);
    ESP_NOW_init(); // Initialize ESP-NOW 
    LCD_disp("ESP_NOW_init OK",3,false);

    LCD_disp("USB_init...   ",0,false);
    LCD_clear_line(1); LCD_clear_line(2); LCD_clear_line(3);

    vTaskDelay(500/portTICK_PERIOD_MS);

    xTaskCreatePinnedToCore(update_LCD, "update_LCD", 4096, (void*) 1, 8, NULL, 0); // Task for screen update
    vTaskDelay(250/portTICK_PERIOD_MS);

    xTaskCreatePinnedToCore(remote_conn, "remote_conn", 4096, (void*) 1, 7, NULL, 0); // Task for remote control

    xTaskCreatePinnedToCore(esp_now_ping, "esp_now_ping", 4096, (void*) 1, 9, NULL, 1); //Scanner task, used only to connect to remote, after which it shuts off
    vTaskDelay(500/portTICK_PERIOD_MS);
    xTaskCreatePinnedToCore(usb_core_task, "usb_init", 4096, (void*) 1, 8, NULL, 1); // Task for USB stuff
    //usb_init((void*) 1);
}
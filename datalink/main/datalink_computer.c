/*
    Datalink to ground 
*/
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_mac.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "lwip/sockets.h"

#include "driver/ledc.h"
#include "sdkconfig.h"

#include "driver/i2c.h"

#include "datalink.h"

#define BLINK_GPIO 2
#define M_PI                   3.14159265358979 // Tired of manually writing out PI yet??

// TCP/WiFi AP parameters for streaming the data
#define PORT_NUMBER            8888 // The TCP port we will open our server on...
#define SERVER_IP_ADDR         "192.168.4.1" // The default fixed IP address of our access point (unused in current implementation)
#define WFSSID                 "RDA-Debug" // The SSID of our access point
#define WFPASSWORD             "88888888" // The password (8x 8's - very lucky!)
#define WFCHANNEL              1 // Default channel
#define WFMAXCONN              1 // Only have up to 1 connection - refuse all others - reduce lag!

volatile uint32_t _probableTargetIPAddr = 0; // The IP address of connected device
                                             // this will likely correspond to the debug computer running that LABVIEW thing 
                                             // generally this value should be the equivalent of 192.168.4.2, but
                                             // it is good to make sure regardless...


esp_err_t connect_success; // Whether or not we have successfully connected to a TCP server at the 
                                              // proper port number and IP address...

// More definitions...
#define WIFI_SUCCESS 1 << 0
#define WIFI_FAILURE 1 << 1
#define TCP_SUCCESS 1 << 0
#define TCP_FAILURE 1 << 1

// Define I2C parameters - Note: ESP32 can handle 2 I2C ports natively
// but we will need to make sure that it can handle simultaneous broadcast/receive (Edit: Why?)
#define SCL_PIN_2_FC              22 // I2C clock pin for data to the flight computer
#define SDA_PIN_2_FC              21 // I2C data pin for data to the flight computer
#define SCL_PIN_2_DL              17 // I2C clock pin for data to the datalink computer
#define SDA_PIN_2_DL              16 // I2C data pin for data to the datalink computer

#define ADDR_ESP_SLAV        0x20 // ESP32 memory address for I2C connection shared by datalink/flight controller
                                  // for use in the slave bus (in the flight computer case, this points to the datalink)
                                  // in the datalink case, this points to the flight computer

static const char *TAG = "example";

// LED and state machine variables... 
static uint8_t s_led_state = 0;
volatile uint8_t _state = 0; // [0] Awaiting I2C data transfer, [1] Awaiting TCP connection, [2] Connected to both

// Storage for variables from I2C connection read
uint8_t acftState = 0;                          // Aircraft state
float ypr[3] = {0,0,0};                         // Current yaw, pitch, roll
float throttle = 0;                       // Current throttle setting
uint8_t buttonPress = 0;                  // Last button pressed
uint8_t hatSwitch = 8;                    // Last hat switch
float ypr_cmd[3] = {0,0,0};                     // Last commanded yaw, pitch, roll
float dypr[3] = {0,0,0};                        // LQI derivative term
float integral[3] = {0,0,0};                    // LQI integral term
uint32_t motor_PWM[2] = {0,0};                 // Motor PWM duty cycle
uint32_t servo_PWM[2] = {0,0};                 // Servo PWM duty cycle

/// @brief Initialize I2C master for transmitting data to datalink computer
static void i2c_DL_init() {
    // Configure the I2C master interface
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER; // (is master for flight computer, slave for datalink computer)
    conf.sda_io_num = (gpio_num_t)SDA_PIN_2_DL;
    conf.scl_io_num = (gpio_num_t)SCL_PIN_2_DL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    conf.clk_flags = 0;
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

/// @brief Initialize I2C slave for transmitting data to datalink computer
static void i2c_FC_init() {
    // Configure the I2C slave interface
    i2c_config_t conf2;
    conf2.mode = I2C_MODE_SLAVE; // (is slave for flight computer, master for datalink computer)
    conf2.sda_io_num = (gpio_num_t)SDA_PIN_2_FC;
    conf2.scl_io_num = (gpio_num_t)SCL_PIN_2_FC;
    conf2.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf2.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf2.slave.addr_10bit_en = 0;
    conf2.slave.slave_addr = ADDR_ESP_SLAV;
    conf2.clk_flags = 0; // Apparently this is needed for the I2C to work (no explanation given in docs)
    conf2.slave.maximum_speed = 400000; // Apparently this is needed for the I2C to work (no explanation given in docs)
    i2c_param_config(I2C_NUM_1, &conf2);
    i2c_driver_install(I2C_NUM_1, conf2.mode, 128, 128, 0); // For some reason SW I2C buffer must be > 100 (no explanation given in docs)
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } 
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
        if(_state!=0)
            _state = 1; // We lost connection for sure if this happens!
        connect_success = TCP_FAILURE;
    }
    else if (event_id == IP_EVENT_AP_STAIPASSIGNED) //[b]added this to get IP but this event is not being triggered [/b]
    {
        ip_event_ap_staipassigned_t *event = (ip_event_ap_staipassigned_t *)event_data;
        ip4_addr_t staAddr;
        staAddr.addr = event->ip.addr;
        _probableTargetIPAddr = staAddr.addr; // Save this IP address - it will come in handy when we check if a TCP server exists there!
        ESP_LOGI(TAG, "STA ip address maybe %s\n", ip4addr_ntoa(&staAddr));
    }
}

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_AP_STAIPASSIGNED,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WFSSID,
            .ssid_len = strlen(WFSSID),
            .channel = WFCHANNEL,
            .password = WFPASSWORD,
            .max_connection = WFMAXCONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    if (strlen(WFPASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             WFSSID, WFPASSWORD, WFCHANNEL);
}

/// @brief Attempt to read from I2C connection 
/// @param Nothing... 
/// @return Whether we succeeded or not (assumed that failure is caused by lack of connection)...
static esp_err_t FCI2CReadAttempt()
{
    uint8_t *readbuffer = malloc(sizeof(DL_debug_info_t));
    //ESP_LOGI("I2C","Allocated %i size to readbuffer...should be %i", sizeof(readbuffer), sizeof(DL_debug_info_t));
    esp_err_t success = i2c_master_read_from_device(I2C_NUM_0, ADDR_ESP_SLAV, readbuffer, sizeof(DL_debug_info_t), 25);
    //return ESP_FAIL;
    if(success != ESP_OK)
    {
        ESP_LOGE("I2C", "Read failed!");
        return ESP_FAIL;   
    }

    // if(sizeof(readbuffer) != sizeof(DL_debug_info_t))
    // {
    //     ESP_LOGE("I2C", "Incorrect size of data! Size: %i, expected: %i",sizeof(readbuffer),sizeof(DL_debug_info_t));
    //     free(readbuffer);
    //     return ESP_FAIL;
    // }

    DL_debug_info_t *received = (DL_debug_info_t *) malloc(sizeof(DL_debug_info_t));
    memcpy(received, readbuffer, sizeof(DL_debug_info_t));

    acftState = received->state;

    ypr[0] = received->ypr[0]; 
    ypr[1] = received->ypr[1];
    ypr[2] = received->ypr[2];

    throttle = received->throttle;
    buttonPress = received->buttonPress;
    hatSwitch = received->hatSwitch;

    ypr_cmd[0] = received->ypr_cmd[0];
    ypr_cmd[1] = received->ypr_cmd[1];
    ypr_cmd[2] = received->ypr_cmd[2];

    dypr[0] = received->dypr[0];
    dypr[1] = received->dypr[1];
    dypr[2] = received->dypr[2];

    integral[0] = received->integral[0];
    integral[1] = received->integral[1];
    integral[2] = received->integral[2];

    motor_PWM[0] = received->motor_PWM[0];
    motor_PWM[1] = received->motor_PWM[1];

    servo_PWM[0] = received->servo_PWM[0];
    servo_PWM[1] = received->servo_PWM[1];

    ESP_LOGI("I2C","Process data: s:%i,y:%0.2f,p:%0.2f,r:%0.2f,t:%0.2f,...",acftState,ypr[0],ypr[1],ypr[2],throttle);

    free(readbuffer);
    free(received);
    return success;
}

/// @brief Connect to the server and return the result
/// @param Target IP address and a handle to a socket number that we will use to send data later...
/// @return Will return success if connected, failed if not...
esp_err_t connect_tcp_server(uint32_t targetIPAddr, int *socketN)
{
    struct sockaddr_in serverInfo = {0};
    
    serverInfo.sin_family = AF_INET;
    serverInfo.sin_addr.s_addr = targetIPAddr;//0x2600000a; // use IP address to hex convertor
    serverInfo.sin_port = htons(PORT_NUMBER);

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    {
        *socketN = -1; 
        ESP_LOGE(TAG, "Failed to create a socket..?");
        return TCP_FAILURE;
    }

    if (connect(sock, (struct sockaddr *)&serverInfo, sizeof(serverInfo)) != 0)
    {
        *socketN = -1;
        ESP_LOGE(TAG, "Failed to connect to %s!", inet_ntoa(serverInfo.sin_addr.s_addr));
        close(sock);
        return TCP_FAILURE;
    }

    *socketN = sock;
    ESP_LOGI(TAG, "Connected to TCP server.");

    return TCP_SUCCESS;
}

/// @brief Stream the data to TCP server
/// @param The socket we send to
/// @return Will always return success, in this program we will infer disconnection using WIFI_DISCONNECT interrupt instead
esp_err_t tcp_stream_data(int sock)
{
    char stateStr[8] = {0};
    sprintf(stateStr, "%i", acftState); // Convert aircraft state to a string... note that this is always a 1 digit number!

    char yawStr[8] = {0};
    sprintf(yawStr, "%i", (int8_t)(ypr[0] * 180.0f/M_PI)); // Convert yaw to degrees and place in a string...
    char pitchStr[8] = {0};
    sprintf(pitchStr, "%i", (int8_t)(ypr[1] * 180.0f/M_PI)); // Convert pitch to degrees and place in a string...
    char rollStr[8] = {0};
    sprintf(rollStr, "%i", (int8_t)(ypr[2] * 180.0f/M_PI));// Convert roll to degrees and place in a string...

    char cmdYawStr[8] = {0};
    sprintf(cmdYawStr, "%i", (int8_t)(ypr[0] * 180.0f/M_PI)); // Some kind of a normalized joystick YAW motion (between -100 to 100)
    char cmdPitchStr[8] = {0};
    sprintf(cmdPitchStr, "%i", (int8_t)(ypr[1] * 180.0f/M_PI)); // Some kind of a normalized joystick PITCH motion (between -100 to 100)
    char cmdRollStr[8] = {0};
    sprintf(cmdRollStr, "%i", (int8_t)(ypr[2] * 180.0f/M_PI)); // Some kind of a normalized joystick ROLL motion (between -100 to 100)

    char cmdThrottleStr[8] = {0};
    sprintf(cmdThrottleStr, "%i", (uint8_t)(throttle)); // Throttle: Bounded between 0 - 100
    char leftMotorPWMStr[8] = {0};
    sprintf(leftMotorPWMStr, "%li", motor_PWM[0]); // Left motor PWM signal, raw value between 3277 and 6554, if 0 indicates shut down engine
    char rightMotorPWMStr[8] = {0};
    sprintf(rightMotorPWMStr, "%li", motor_PWM[1]); // Right motor PWM signal, raw value between 3277 and 6554, if 0 indicates shut down engine
    char leftServoPWMStr[8] = {0};
    sprintf(leftServoPWMStr, "%li", servo_PWM[0]); // Left servo PWM signal, raw value between 3277 and 6554, if 0 indicates shut down servo
    char rightServoPWMStr[8] = {0};
    sprintf(rightServoPWMStr, "%li", servo_PWM[1]); // Right servo PWM signal, raw value between 3277 and 6554, if 0 indicates shut down servo

    //char data_to_send[1024] = {0};
    //sprintf(data_to_send, "%d%d%d%d", strlen(potentiometer_value_string), potentiometer_value+i, strlen(potentiometer_value_string2), potentiometer_value2+i);
    //send(sock, data_to_send, strlen(data_to_send), 0);
    //ESP_LOGI(TAG, "%s", data_to_send);

    /* OPTION 1: ALL OR NOTHING */
    // Sends in order, one string, with the format of a number indicating length of subsequent, followed by the data
    // Send order is [state,yaw,pitch,roll,cmdYaw,cmdPitch,cmdRoll,cmdThrottle,leftMotorPWM,rightMotorPWM,leftServoPWM,rightServoPWM]
    char send_string[512] = {0};
    sprintf(send_string,"%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s",strlen(stateStr),stateStr,strlen(yawStr),yawStr,
        strlen(pitchStr),pitchStr,strlen(rollStr),rollStr,
        strlen(cmdYawStr),cmdYawStr,strlen(cmdPitchStr),cmdPitchStr,
        strlen(cmdRollStr),cmdRollStr,strlen(cmdThrottleStr),cmdThrottleStr,
        strlen(leftMotorPWMStr),leftMotorPWMStr,strlen(rightMotorPWMStr),rightMotorPWMStr,
        strlen(leftServoPWMStr),leftServoPWMStr,strlen(rightServoPWMStr),rightServoPWMStr);

    send(sock, &send_string, strlen(send_string), 0); // Send the data to TCP server
    ESP_LOGI("TCP", "%s", send_string);

    // /* OPTION 2: SEND LABELED PACKETS */
    // // This gives a "type" identifier (1 character) followed by the value
    // // The advantage of this format is that send order doesn't really matter, disadvantage is that lost packets may be notable
    // char send_string[24] = {0};
    // sprintf(send_string, "%s%s", "S",stateStr);
    // send(sock, &send_string, strlen(send_string),0);
    // sprintf(send_string, "%s%s", "y",yawStr); // In this case, use small y to indicate measured yaw
    // send(sock, &send_string, strlen(send_string),0);
    // sprintf(send_string, "%s%s", "p",pitchStr); // Same, but for pitch
    // send(sock, &send_string, strlen(send_string),0);
    // sprintf(send_string, "%s%s", "r",rollStr); // Same, but for roll
    // send(sock, &send_string, strlen(send_string),0);
    // sprintf(send_string, "%s%s", "Y",cmdYawStr); // Large character for command values of Yaw, Pitch, Roll 
    // send(sock, &send_string, strlen(send_string),0);
    // sprintf(send_string, "%s%s", "P",cmdPitchStr); // Large character for command values of Yaw, Pitch, Roll 
    // send(sock, &send_string, strlen(send_string),0);
    // sprintf(send_string, "%s%s", "R",cmdRollStr); // Large character for command values of Yaw, Pitch, Roll 
    // send(sock, &send_string, strlen(send_string),0);
    // sprintf(send_string, "%s%s", "L",leftMotorPWMStr); // Large character L for PWM of left motor
    // send(sock, &send_string, strlen(send_string),0);
    // sprintf(send_string, "%s%s", "R",rightMotorPWMStr); // Large character R for PWM of right motor
    // send(sock, &send_string, strlen(send_string),0);
    // sprintf(send_string, "%s%s", "l",leftServoPWMStr); // Small character L for PWM of left servo
    // send(sock, &send_string, strlen(send_string),0);
    // sprintf(send_string, "%s%s", "r",rightServoPWMStr); // Small character R for PWM of right servo
    // send(sock, &send_string, strlen(send_string),0);

    return TCP_SUCCESS;
}

/// @brief Test for multitasking, this task does nothing but just blinks the LED
/// @param pvParameters 
static void LEDTask(void *pvParameters)
{
    ESP_LOGI("LED", "GPIO LED will visually show us the state!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    uint16_t blinkPeriod = 500; // Default blinking time...

    while(1)
    {
        switch (_state)
        {
            case 0: // Awaiting I2C connection, we blink at an average speed
                blinkPeriod = 500;
                break;

            case 1: // Connected to I2C, we blink much slower
                blinkPeriod = 1000;
                break;
                
            case 2: // I2C and TCP connected, we blink much faster, in fact it should match TCP transfer rate
                blinkPeriod = 150;
                break;
        
            default:
                break;
        }

        ESP_LOGI("LED", "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        /* Toggle the LED state */
        s_led_state = !s_led_state;
          /* Set the GPIO level according to the state (LOW or HIGH)*/
        gpio_set_level(BLINK_GPIO, s_led_state);

        vTaskDelay((TickType_t) blinkPeriod/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

/// @brief I2C data collection (every 100 milliseconds or so...)
/// @param pvParameters 
static void DataCollectionTask(void *pvParameters)
{
    i2c_FC_init();
    i2c_DL_init();

    const TickType_t delay = pdMS_TO_TICKS(125);
    esp_err_t hasEstablishedConnection;
    while(1)
    {
        switch (_state)
        {
            case 0: // Awaiting I2C connection
                hasEstablishedConnection = FCI2CReadAttempt();
                if(hasEstablishedConnection == ESP_OK)
                {
                    _state = 1; // Move to next state when I2C communication succeeded...
                }
                else
                {
                    ESP_LOGE("I2C", "Did not find device!");
                }
                break;

            case 1: // Connected to I2C, awaiting TCP connection
                hasEstablishedConnection = FCI2CReadAttempt();
                if(hasEstablishedConnection == ESP_OK)
                {
                    //_state = 1; // Move to next state when I2C communication succeeded...
                }
                else
                {
                    ESP_LOGE("I2C", "Did not find device!");
                    _state = 0; // Seems like an issue with the I2C stream!
                }
                break;
                
            case 2: // I2C and TCP connected, data will stream
                break;
        
            default:
                break;
        }

        vTaskDelay(delay);
    }

    vTaskDelete(NULL);
}

/// @brief I2C data collection (every 100 milliseconds or so...)
/// @param pvParameters 
static void TCPTask(void *pvParameters)
{
    nvs_flash_init();
    wifi_init_softap();

    const TickType_t delay = pdMS_TO_TICKS(150);

    int socketNum = -1;

    while(1)
    {
        switch (_state)
        {
            case 0: // Awaiting I2C connection
                // We do nothing as we are not connected to I2C for some reason, meaning that this would
                // be pointless to try sending any data. Sad!
                break;

            case 1: // Connected to I2C, awaiting TCP connection
                if(connect_success != TCP_SUCCESS && _probableTargetIPAddr != 0) // Try connection if we are not already connected...
                {
                    connect_success = connect_tcp_server(_probableTargetIPAddr, &socketNum);
                }
                if(connect_success == TCP_SUCCESS)
                {
                    _state = 2; // Connected !!!
                }
                else
                {
                    ESP_LOGI("TCP","SCANNING, NONE FOUND!");
                }
                break;
                
            case 2: // I2C and TCP connected, data will stream
                connect_success = tcp_stream_data(socketNum); // Try to stream TCP data
                if(connect_success != TCP_SUCCESS)
                {
                    _state = 1; // This implies we were disconnected!
                }
                break;
        
            default:
                break;
        }

        vTaskDelay(delay);
    }
}

void app_main(void)
{
    xTaskCreatePinnedToCore(LEDTask, "LEDTask", 4096, (void*) 1, 8, NULL, 0); // LED blinking as an indicator
    xTaskCreatePinnedToCore(DataCollectionTask, "DataCollectionTask", 4096, (void*) 1, 10, NULL, 0); // I2C data collection from flight computer
    xTaskCreatePinnedToCore(TCPTask, "TCPTask", 4096, (void*) 1, 8, NULL, 1); // Combined WiFi and TCP client task
}

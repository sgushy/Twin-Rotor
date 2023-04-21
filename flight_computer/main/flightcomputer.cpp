/* 
    Flight computer for twin rotor helicopter
    Last updated 4/6/2023 (unfinished)
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

#include "esp_wifi.h"
#include "esp_now.h"


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

// Define PWM parameters (for servo control)
#define LEDC_SERVO_FREQUENCY          50 
#define SERVO_PWM_MIN                 3277
#define SERVO_PWM_MAX                 6554
#define SERVO_PWM_MID                 4916

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
//#define ADDR_IMU_R           0x69 // Secondary IMU memory address for I2C connection (will no longer be used, only 1 IMU is decided)
#define ADDR_LCD_SCREEN      0x3c // Address of the LCD screen (may or may not be used in the final product)

#define LQI_PERIOD_MS        50 // Period to discretize LQI execution time by   
#define IMU_PERIOD_MS        41

#define MAX_THRUST_N         16.778f // max thrust per propeller in newtons

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
                             
// Variables with regards to state estimation
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint16_t packetSize = 42;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU

// Variables with regards to connection to remote
volatile bool _receivedRemoteHandshake = false; // Have we connected to the remote?
volatile long _lastCommunicationWithRemote = 0; // Last time we heard from remote - used to infer connection issues
volatile uint8_t _remoteMACAddr = 0; // MAC address of remote (will be detected during remote ping/proximity scan)

// The commanded angles and yaw rate will be received from the remote (based on position of remote controller)
volatile float _cmdThrottlePercentage = 0; // Throttle strength in percent

// The following variables are to be used for the control loop
// these are to be converted from the commanded pitch/roll/yaw/throttle to values that are 
volatile uint8_t _forcePercentageLeft = 0; // Commanded % of left thrust, added to open loop thrust
volatile uint8_t _forcePercentageRight = 0; // Commanded % of left thrust, added to open loop thrust
volatile uint8_t _reqServoLeft = 0; // Commanded servo angle in radians - used for PITCH - will convert to PWM
volatile uint8_t _reqServoRight = 0; // Commanded servo angle in radians - used for YAW - will convert to PWM

// Throttle and servo duty cycles - The control loop calculates these, which will then be read by motor-running task
// to set the motor powers and servo angles - Initialize at zero so that the aircraft does not try to take off for some reason
volatile uint32_t _throttleLeft = ENGINE_PWM_MIN;
volatile uint32_t _throttleRight = ENGINE_PWM_MIN;
volatile uint32_t _servoLeft = SERVO_PWM_MIN;
volatile uint32_t _servoRight = SERVO_PWM_MIN;

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
/*     // Configure the I2C slave interface
    i2c_config_t conf;
    conf.mode = I2C_MODE_SLAVE; // (is slave for flight computer, master for datalink computer)
    conf.sda_io_num = (gpio_num_t)SDA_PIN_2_DL;
    conf.scl_io_num = (gpio_num_t)SCL_PIN_2_DL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.slave.slave_addr = ADDR_ESP_SLAV,
    i2c_param_config(I2C_NUM_1, &conf);
    i2c_driver_install(I2C_NUM_1, conf.mode, 128, 128, 0); */
}

/// @brief Turn on the connection that will be used to contact the remote
static void ESP_NOW_init()
{

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

    while(true)
    {
        switch (_state)
        {
            case 0:
                LCD_disp("STATE 0          ",0,false);
                LCD_disp("NO-REMOTE        ",1,false);
                snprintf(YPRstr, sizeof(YPRstr), "Y%.1fP%.1fR%.1f", ypr[0]* 180/M_PI, ypr[1]* 180/M_PI, ypr[2]* 180/M_PI);
                LCD_disp(YPRstr,2,false);
                LCD_disp("MOT-CNTRL-OFF    ",3,false);
                break;
            case 1:
                LCD_disp("STATE 1          ",0,false);
                LCD_disp("RMT-CONNECT      ",1,false);
                LCD_disp("A                ",2,false);
                LCD_disp("AWAT-ENG-START   ",3,true); 
                break;
            case 2:
                LCD_disp("STATE 2          ",0,false);
                LCD_disp("ENG-ON           ",1,false);
                LCD_disp("A                ",2,false);
                LCD_disp("                 ",3,true); 
                break;
            case 3:
                LCD_disp("STATE 3          ",0,false);
                LCD_disp("AUTO-HVR         ",1,false);
                LCD_disp("A                ",2,false);
                LCD_disp("                 ",3,true); 
                break;
            case 4:
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

/// @brief Check connection to remote control and process remote output (Every 25 ms)
/// The longer delay should be ok as it is still much faster than a human reaction time
/// What really matters is that the control loop is fast enough, which is handled in other sections of the code... 
/// @param pvParam 
static void remote_conn(void* pvParam)
{
    TickType_t lastTaskTime = xTaskGetTickCount();
    const TickType_t delay_time = pdMS_TO_TICKS(25);
    
    while(true)
    {
        switch (_state)
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

/// @brief Actuation of motors and servos... (approx 200 Hz)
/// [Note:] This method will read from the global variables that were computed by LQI function.
/// @param pvParam 
static void motors(void* pvParam)
{
    TickType_t lastTaskTime = xTaskGetTickCount();
    const TickType_t delay_time = pdMS_TO_TICKS(15);

    while(true)
    {
        switch (_state)
        {
            case 0:
                //ESP_LOGI("[MOT-INFO]", "Motors idle - awaiting remote connection!");
                break;
            case 1:
                //ESP_LOGI("[MOT-INFO]", "Connected to remote! Awaiting engine start...");
                break;
            case 2: case 3: case 4:
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, _throttleRight);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, _throttleLeft);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, _servoRight);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, _servoLeft);

                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
                break;
            default:
                // It should never reach this case...
                break;
        }
        vTaskDelayUntil(&lastTaskTime, delay_time);
    }
    vTaskDelete(NULL);
}

/// @brief Reading from IMUs (sampling at 1 kHz) 
/// @param pvParam 
static void IMU(void* pvParam)
{
    TickType_t lastTaskTime = xTaskGetTickCount();
    const TickType_t delay_time = pdMS_TO_TICKS(IMU_PERIOD_MS);
    
    /* // Following are for the state measurement (Not anymore!!)
    mpu6050_handle_t mpu6050_dev = NULL;
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    complimentary_angle_t complimentary_angle;
    // Initialize MPU-6050 sensor
    mpu6050_dev = mpu6050_create(I2C_NUM_0, ADDR_IMU_F);
    mpu6050_config(mpu6050_dev, ACCE_FS_4G, GYRO_FS_500DPS);
    mpu6050_wake_up(mpu6050_dev); */

    // Start IMU and initialize the DMP
    MPU6050 mpu = MPU6050(ADDR_IMU_F);
	mpu.initialize();
	mpu.dmpInitialize();

    // Calibrate IMU to get rid of offsets
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);

    // Enable digital motion processor so that we don't need to write our own quaternion filter
	mpu.setDMPEnabled(true);

    while(true)
    {
        // Read sensor data and print results
       /*  mpu6050_get_acce(mpu6050_dev, &acce);
        mpu6050_get_gyro(mpu6050_dev, &gyro);
        mpu6050_complimentory_filter(mpu6050_dev, &acce, &gyro, &complimentary_angle);
        ESP_LOGI("IMU", "Acceleration: (%f, %f, %f) g", acce.acce_x, acce.acce_y, acce.acce_z);
        ESP_LOGI("IMU", "Rotation: (%f, %f, %f) deg/s", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);  
        ESP_LOGI("IMU", "EST_ANGLE: (%f, %f) deg", complimentary_angle.roll, complimentary_angle.pitch);
         */
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
            for (int i = 0; i < 3; i++) {ypr_prime[i] = (ypr[i] - ypr_old[i])/(IMU_PERIOD_MS/1000);}

			/* printf("YAW: %3.1f, ", ypr[0] * 180/M_PI); 
			printf("PITCH: %3.1f, ", ypr[1] * 180/M_PI);
			printf("ROLL: %3.1f \n", ypr[2] * 180/M_PI); */
 
            //LCD_disp("Y: %3.1f, " + ypr[0],2,false);
	    }

	    //Best result is to match with DMP refresh rate
	    // Its last value in components/MPU6050/MPU6050_6Axis_MotionApps20.h file line 310
	    // Now its 0x13, which means DMP is refreshed with 10Hz rate
		// vTaskDelay(5/portTICK_PERIOD_MS);

        vTaskDelayUntil(&lastTaskTime, delay_time);
	}

    vTaskDelete(NULL);
}

/// @brief LQI controller (approx 330 Hz)
/// @param pvParam 
static void LQI(void* pvParam)
{
    TickType_t lastTaskTime = xTaskGetTickCount();
    const TickType_t delay_time = pdMS_TO_TICKS(LQI_PERIOD_MS);
    float u[4] = {0, 0, 0, 0};
    float x[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    
    while(true)
    {
        switch (_state)
        {
            case 0:
                // Do nothing (no control happens while the system is off)
                break;
            case 1:
                // Should put here a test or something (or maybe in the motors section)
                // At least for the servos range of motion
                break;
            case 2: case 3: case 4:
                // Fly state
                // update accumulator values
                for (int i = 0; i < 3; i++) {
                    ypr_accum[i] += (ypr_target[i] - ypr[i]) * LQI_PERIOD_MS / 1000;
                }

                // first zero u
                for (int i = 0; i < 4; i++) {
                    u[i] = 0;
                }
                x[0] = ypr[0];
                x[1] = ypr_prime[0];
                x[2] = ypr[1];
                x[3] = ypr_prime[1];
                x[4] = ypr[2];
                x[5] = ypr_prime[2];
                x[6] = ypr_accum[0];
                x[7] = ypr_accum[1];
                x[8] = ypr_accum[2];
                // multiply by K matrix to obtain u

                for (int i = 0; i < 4; i++) {
                    for (int j = 0; j < 9; j++) {
                        u[i] += K[i][j] * x[j] * -1;
                    }
                }

                // operate on u to get thrusts and angles
                _throttleRight = _state == 4 ? 0 : (u[0] + _forcePercentageRight / 100) 
                    / MAX_THRUST_N * (ENGINE_PWM_MAX - ENGINE_PWM_MIN)
                    + ENGINE_PWM_MIN;
                _throttleLeft = _state == 4 ? 0 : (u[1] + _forcePercentageLeft / 100)
                    / MAX_THRUST_N * (ENGINE_PWM_MAX - ENGINE_PWM_MIN)
                    + ENGINE_PWM_MIN; 
                _servoRight = u[2] / (u[0] * 1.570796f) * (SERVO_PWM_MAX - SERVO_PWM_MIN) 
                    + SERVO_PWM_MID;
                _servoLeft = u[3] / (u[1] * 1.570796f) * (SERVO_PWM_MAX - SERVO_PWM_MIN)
                    + SERVO_PWM_MID;
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
extern "C" void app_main(void)
{
    i2c_DL_init(); // Start I2C bus to data link with error check
    i2c_FC_init(); // Start I2C bus to flight computer with error check

    LCD_init();
    
    LCD_disp("LCD_init OK    ",0,false); // Confirm successful I2C start
    vTaskDelay(500/portTICK_PERIOD_MS);
    LCD_disp("I2C_init OK    ",1,false);

    ESP_NOW_init();

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

    xTaskCreatePinnedToCore(IMU, "IMU", 4096, (void*) 1, 7, NULL, 0); // Task for reading IMU sensors
    xTaskCreatePinnedToCore(remote_conn, "remote_conn", 4096, (void*) 1, 4, NULL, 0); // Task for remote control (will listen in the background)
    
    xTaskCreatePinnedToCore(update_LCD, "update_LCD", 4096, (void*) 1, 2, NULL, 1); // Task for screen update

    xTaskCreatePinnedToCore(LQI, "LQI", 4096, (void*) 1, 5, NULL, 1); // Control algorithm
    xTaskCreatePinnedToCore(motors, "motors", 4096, (void*) 1, 10, NULL, 1); // Motor actuation is on core 1
}

/// @brief Test/Calibration code
static void Test_Calibration()
{
    example_ledc_init();
    // Set duty to 100%
    ESP_LOGI("A", "MIN");

    _throttleLeft = ENGINE_PWM_MIN;

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, _throttleLeft));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0));

/*     vTaskDelay(8000/portTICK_PERIOD_MS);
    ESP_LOGI("A", "MAX");
    _throttleLeft = ENGINE_PWM_MIN;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, _throttleLeft));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0)); */

    vTaskDelay(8000/portTICK_PERIOD_MS);
    ESP_LOGI("A", "5500");
    _throttleLeft = 4000;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, _throttleLeft));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0));
}
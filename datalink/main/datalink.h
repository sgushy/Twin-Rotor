#ifndef DATALINK_H
#define DATALINK_H

typedef struct 
{
    uint8_t state;                         // Current state of the aircraft state machine
    float ypr[3];                          // Current yaw, pitch, roll
    float throttle;                        // Current throttle setting
    uint8_t buttonPress;                   // Last button pressed
    uint8_t hatSwitch;                     // Last hat switch
    float ypr_cmd[3];                      // Last yaw, pitch, roll command
    float dypr[3];                         // LQI derivative term
    float integral[3];                     // LQI integral term
    uint32_t motor_PWM[2];                 // Motor PWM duty cycle
    uint32_t servo_PWM[2];                 // Servo PWM duty cycle
} __attribute__((packed)) DL_debug_info_t; // This is info that will be sent to datalink computer for purpose of showing on instrument panel

#endif
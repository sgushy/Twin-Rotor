#ifndef DATALINK_H
#define DATALINK_H

typedef struct 
{
    float ypr[3];                         // Current yaw, pitch, roll
    float throttle;                       // Current throttle setting
    uint8_t buttonPress;                  // Last button pressed
    uint8_t hatSwitch;                    // Last hat switch
    float ypr_cmd[3];                     // Last commanded yaw, pitch, roll
    float dypr[3];                        // LQI derivative term
    float integral[3];                    // LQI integral term

} __attribute__((packed)) DL_debug_info_t;

#endif
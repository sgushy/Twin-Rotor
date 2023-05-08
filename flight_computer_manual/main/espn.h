#ifndef ESPN_H
#define ESPN_H

/* ESPNOW can work in both station and softap mode. It is configured in menuconfig. */
#if CONFIG_ESPNOW_WIFI_MODE_STATION
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   WIFI_IF_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   WIFI_IF_AP
#endif

#define ESPNOW_QUEUE_SIZE           32 // Enlarged queue size to reduce chance of that one weird hanging crash

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, _remoteMACAddr, ESP_NOW_ETH_ALEN) == 0)

typedef enum {
    ESPNOW_SEND_CB,
    ESPNOW_RECV_CB,
} espnow_event_id_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} espnow_event_send_cb_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
} espnow_event_recv_cb_t;

typedef union {
    espnow_event_send_cb_t send_cb;
    espnow_event_recv_cb_t recv_cb;
} espnow_event_info_t;

/* When ESPNOW sending or receiving callback function is called, post event to ESPNOW task. */
typedef struct {
    espnow_event_id_t id;
    espnow_event_info_t info;
} espnow_event_t;

enum {
    ESPNOW_DATA_BROADCAST,
    ESPNOW_DATA_UNICAST,
    ESPNOW_DATA_MAX,
};

/* User defined field of ESPNOW data in this example. */
typedef struct {
    uint8_t type;                         // MSG_TYPE [0 - Ping],[1 - Ping response],[2 - Handshake req],[3 - Handshake response],[4 - Remote update],[5 - Remote update response]
    uint8_t sender[6];                    // MAC address of sender
    uint8_t recipient[6];                 // MAC address of intended recipient
    uint8_t state;                        // The state of the sender's state machine
    uint16_t packet_ID;                   // Packet ID so that receiver can ignore redundant
    float ypr[3];                         // Desired yaw [0], pitch [1], roll [2] (all floating point).
    float throttle;                       // Throttle 
    uint8_t button;                       // Number of button pressed (if value is 88, this is actually a ping)
    uint8_t hat;                          // Hat switch position (if value is 88, this is actually a pairing request)
} __attribute__((packed)) espnow_data_t;

/* Parameters of sending ESPNOW data. */
typedef struct {
    bool unicast;                         //Send unicast ESPNOW data.
    bool broadcast;                       //Send broadcast ESPNOW data.
    uint8_t state;                        // The state of the sender's state machine
    uint32_t magic;                       // [UNUSED] Magic number which is used to determine which device to send unicast ESPNOW data.
    uint16_t count;                       //Total count of unicast ESPNOW data to be sent.
    uint16_t delay;                       //Delay between sending two ESPNOW data, unit: ms.
    int len;                              //Length of ESPNOW data to be sent, unit: byte.
    uint8_t type;                         //Type of message being sent: [0 - Ping],[1 - Ping response],[2 - Handshake req],[3 - Handshake response],[4 - Remote update],[5 - Remote update response]
    uint8_t *buffer;                      //Buffer pointing to ESPNOW data.
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];   //MAC address of destination device.
} espnow_send_param_t;

#endif
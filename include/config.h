#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* Circular Buffer Configuration
* Freematics ONE+ Model B (no PSRAM)
**************************************/
#define BUFFER_SLOTS 32 /* max number of buffer slots */
#define BUFFER_LENGTH 256 /* bytes per slot */
#define SERIALIZE_BUFFER_SIZE 1024 /* bytes */

/**************************************
* Configuration Definitions
**************************************/
#define STORAGE_NONE 0
#define STORAGE_SPIFFS 1
#define STORAGE_SD 2

#define GNSS_NONE 0
#define GNSS_STANDALONE 1
#define GNSS_CELLULAR 2

// Not TCP or HTTP option available, removed from code base for simplicity.
#define PROTOCOL_UDP 1

/**************************************
* OBD-II configurations
**************************************/
#define ENABLE_OBD 1
#define MAX_OBD_ERRORS 3

/**************************************
* Networking configurations
* Cellular only - WiFi disabled
**************************************/
#define ENABLE_WIFI 0

// cellular network settings
// #define CELL_APN "iot.1nce.net"
#define CELL_APN "hologram"
#define APN_USERNAME NULL
#define APN_PASSWORD NULL
#define SIM_CARD_PIN ""

// Telemetry server settings (Traccar)
#define SERVER_HOST "traccar.example.com"
#define SERVER_PROTOCOL PROTOCOL_UDP
#define SERVER_PORT 5170

// maximum consecutive communication errors before resetting network
#define MAX_CONN_ERRORS_RECONNECT 5
// maximum allowed connecting time
#define MAX_CONN_TIME 10000 /* ms */
// data receiving timeout
#define DATA_RECEIVING_TIMEOUT 5000 /* ms */
// expected maximum server sync signal interval
#define SERVER_SYNC_INTERVAL 120 /* seconds, 0 to disable */
// data interval settings
#define STATIONARY_TIME_TABLE {10, 60, 180} /* seconds */
#define DATA_INTERVAL_TABLE {1000, 2000, 5000} /* ms */
#define PING_BACK_INTERVAL 900 /* seconds */
#define SIGNAL_CHECK_INTERVAL 10 /* seconds */

/**************************************
* Data storage configurations
**************************************/
#define STORAGE STORAGE_SD

/**************************************
* MEMS sensors
**************************************/
#define ENABLE_MEMS 1

/**************************************
* GPS
* Change to GNSS_STANDALONE to use the built-in u-blox M10
**************************************/
#define GNSS GNSS_STANDALONE
#define GNSS_ALWAYS_ON 0
#define GNSS_RESET_TIMEOUT 300 /* seconds */

/**************************************
* Standby/wakeup
**************************************/
#define MOTION_THRESHOLD 0.4f /* vehicle motion threshold in G */
#define JUMPSTART_VOLTAGE 14 /* V */
#define RESET_AFTER_WAKEUP 1

/**************************************
* Additional features
**************************************/
#define PIN_SENSOR1 34
#define PIN_SENSOR2 26
#define COOLING_DOWN_TEMP 75 /* celsius degrees */

#define ENABLE_OLED 0
#define ENABLE_HTTPD 0
#define ENABLE_BLE 0
#define ENABLE_ORIENTATION 0

#endif // CONFIG_H_INCLUDED

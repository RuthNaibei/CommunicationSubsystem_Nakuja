#ifndef DEFS_H
#define DEFS_H

/* debug parameters for use during testing - disable before launch */
#define DEBUG 1

#if DEBUG

#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#define debugf(x, y) Serial.printf(x, y)

#else

#define debug(x)
#define debugln(x)
#define debugf(x, y)

#endif

#endif

// #define NSS 5
// #define RST 4
// #define DI0 2
// #define NSS1 15
// #define RST1 32
// #define DI3 33
// #define led 34

#define VSPI_MISO   19
#define VSPI_MOSI   23
#define VSPI_SCLK   18
#define VSPI_SS     5
#define VSPI_RST    4
#define VSPI_DI0    2

#define HSPI_MISO   12
#define HSPI_MOSI   13
#define HSPI_SCLK   14
#define HSPI_SS     15
#define HSPI_RST    32
#define HSPI_DI0    33

#define MQTT_SERVER "192.168.0.104"
#define MQTT_PORT 1883

// /* WIFI credentials */
// const char* SSID = "";
// const char* PASSWORD = "";



const char* topic = "sensor_data_topic";
const int Push_Button = 14;
int Button_State = 0;

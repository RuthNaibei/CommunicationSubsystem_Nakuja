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

/* end of debug parameters */

/* timing constant */
#define SETUP_DELAY 300
#define TASK_DELAY 10

/* flight constants */
#define EJECTION_HEIGHT 1000 // eject at 1000m AGL
#define SEA_LEVEL_PRESSURE 101325 // Assume the sea level pressure is 101325 Pascals - this can change with weather
#define ALTITUDE_OFFSET 1415 /* this value is the altitude at rocket launch site */

/* tasks constants */
#define STACK_SIZE 2048
#define ALTIMETER_QUEUE_LENGTH 10 // todo: change to 2 items
#define GYROSCOPE_QUEUE_LENGTH 10
#define GPS_QUEUE_LENGTH 24
#define ALL_TELEMETRY_DATA_QUEUE_LENGTH  10
#define FILTERED_DATA_QUEUE_LENGTH 10
#define FLIGHT_STATES_QUEUE_LENGTH 1


/* ROCKET FLIGHT STATES */
#define PRE_FLIGHT          0
#define POWERED_FLIGHT      1
#define COASTING            2
#define APOGEE              3
#define BALLISTIC_DESCENT   4
#define PARACHUTE_DESCENT   5
#define POST_FLIGHT         6

/* LEDs for testing - remove on production */
#define PRE_FLIGHT_LED 4
int counter = 0;


#endif
#define TX 17
#define RX 16
// #define NSS 5
// #define RST 4
// #define DI0 2
// #define NSS1 15
// #define RST1 32
// #define DI3 33
// #define led 34
// #define CLK1 14
// #define CS_PIN_1 5  // Chip Select pin for LoRa 1
// #define CS_PIN_2 15  // Chip Select pin for LoRa 2

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


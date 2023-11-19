#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>
#include <TinyGPS++.h>
#include <LoRa.h>
#include "state_machine.h"
#include "defs.h"
#include "kalman.h"
#include "SPIFFS.h"


File file;


double displacement_array[10] = {0}; 

SPIClass vspi; // Define VSPI object
SPIClass hspi; // Define HSPI object
// SPISettings spiSettings(500000, MSBFIRST, SPI_MODE0);
// SPIClass HSPI2(HSPI);
// SPIClass VSPI2(VSPI);
int state;
State_machine fsm;
String LoRaData;
byte destination = 0xF1;

/* create lora classes */
LoRaClass lora1; // transmitter
LoRaClass lora2; // receiver

Adafruit_MPU6050 gyroscope;

/* create altimeter objects */
Adafruit_BMP085 altimeter;

/* GPS Setup*/
HardwareSerial hard(2);
TinyGPSPlus gps;

/* position integration variables */
unsigned long current_time = 0;
unsigned long previous_time = 0;

/* velocity integration variables */
float y_velocity = 0.0;
float y_displacement = 0.0;
int consecutive_count = 0;
float new_y_displacement = 0.0;
float old_y_displacement = new_y_displacement;


double new_y_velocity = 0.0;
double old_y_velocity = new_y_velocity;
// double total_y_displacement = 0.0;

double fallBackLat = -1.0953775626377544;
double fallBackLong = 37.01223403257954;

/* functions to initialize sensors */
void initialize_gyroscope()
{
  /* attempt to initialize MPU6050 */
  if (!gyroscope.begin(0x68) && !gyroscope.begin(0x69))
  {
    debugln("[-]Gyroscope allocation failed!");
    // loop forever until found
    while (true)
    {
    }
  }

  debugln("[+]Gyroscope Initialized");
  gyroscope.setAccelerometerRange(MPU6050_RANGE_8_G);
  gyroscope.setGyroRange(MPU6050_RANGE_500_DEG);
  gyroscope.setFilterBandwidth(MPU6050_BAND_5_HZ);

  delay(SETUP_DELAY);
}

void initialize_altimeter()
{
  if (!altimeter.begin())
  {
    debugln("[-]Could not find a valid altimeter sensor");
    while (1)
    {
    }
  }

  debugln("[+]Altimeter initialized");
}

struct Gyro_Data
{
  double ax;
  double ay;
  double az;
  double gx;
  double gy;
  double gz;
};

struct GPS_Data
{
  double latitude;
  double longitude;
  uint time;
};

struct Altimeter_Data
{
  double pressure;
  double altitude;
  double velocity;
  double AGL; /* altitude above ground level */
};

struct Altimeter_Data altimeter_data;
struct Gyro_Data gyro_data;
char telemetry_data[700]; // buffer to hold telemetry data

// struct Telemetry_Data
// {
//   float ax;
//   float ay;
//   float az;
//   float gx;
//   float gy;
//   float gz;
//   float pressure;
//   float altitude;
//   float velocity;
//   float AGL; /* altitude above ground level */
//   double latitude;
//   double longitude;
// };

// initializing lora modules
void lora1params(){
  lora1.setTxPower(18);
  lora1.setFrequency(433E6);       // Set a specific frequency for lora1 (e.g., 433 MHz)
  lora1.setSyncWord(0xF1);         // Set a unique sync word for lora1
  lora1.setSpreadingFactor(8);     // Set SF for lora1 (adjust as needed)
  lora1.setSignalBandwidth(125E3); // Set bandwidth for lora1 (adjust as needed)
  lora1.setCodingRate4(5);         // Set coding rate for lora1 (adjust as needed)
}
void reconnectLora1()
{
  while (!lora1.begin(433E6))
  {
    Serial.println("Error initializing LoRa transmitter. Retrying...");
    delay(500); // Wait for 5 seconds before retrying
    lora1params();
  }
  Serial.println("LoRa transmitter initialized successfully.");
}

void initializeLora1()
{
  // hspi = SPIClass(HSPI);
  // hspi.begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS);
  // lora1.setPins(HSPI_SS,HSPI_RST,HSPI_DI0);
  // lora1.setSPIFrequency(10000000);
  // lora1.setSPI(hspi);

  vspi = SPIClass(VSPI);
  // vspi.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS);
  lora1.setPins(VSPI_SS, VSPI_RST, VSPI_DI0);
  // lora2.setSPI(vspi);
  lora1.setSPIFrequency(10000000);

  if (!lora1.begin(433E6))
  {
    Serial.println("Error initializing LoRa transmitter.");
    reconnectLora1();
  }
  lora1params();
  // lora1.enableCrc();
  // debugln("LoRa transmitters successfully initialized!");
}
void lora2params(){
  lora2.setFrequency(433E6);       // Set a different frequency for lora2 (within the same band)
  lora2.setSyncWord(0xF4);         // Set a different sync word for lora2
  lora2.setSpreadingFactor(9);     // Set a different SF for lora2 (adjust as needed)
  lora2.setSignalBandwidth(250E3); // Set a different bandwidth for lora2 (adjust as needed)
  lora2.setCodingRate4(7);         // Set a different coding rate for lora2 (adjust as needed)
}
void reconnectLora2()
{
  while (!lora2.begin(433E6))
  {
    Serial.println("Error initializing LoRa receiver. Retrying...");
    delay(500); // Wait for 5 seconds before retrying
    lora2params();
  }
  Serial.println("LoRa receiver initialized successfully.");
}

void initializeLora2()
{
  lora2.setPins(HSPI_SS, HSPI_RST, HSPI_DI0);
  lora2.setSPIFrequency(10000000);
  if (!lora2.begin(433E6))
  {
    Serial.println("Error initializing LoRa receiver.");
    reconnectLora2();
  }
  debugln("lora receiver set");
  // Set a common sync word for communication between the modules
  lora2params();
}

void readAltimeter()
{

    /* Read pressure
   * This is the pressure from the sea level
   * */
  altimeter_data.pressure = altimeter.readSealevelPressure();

  /* Read altitude
   * This is the altitude from the sea level
   * */
  altimeter_data.altitude = altimeter.readAltitude(SEA_LEVEL_PRESSURE);

  /* approximate velocity from acceleration by integration for apogee detection */
  current_time = millis();
  unsigned long deltaTime = (current_time -previous_time)/1000;
  /* differentiate displacement to get velocity */
  if (deltaTime > 0){

    new_y_displacement = altimeter_data.altitude - ALTITUDE_OFFSET;

    y_velocity = (new_y_displacement - old_y_displacement) / static_cast<float>(deltaTime);
    
     /* update integration variables */
    previous_time = current_time;
    // double displacement_diff = new_y_displacement - old_y_displacement;
   

    displacement_array[consecutive_count] = new_y_displacement;
    debugln(displacement_array[consecutive_count]);
            printf("Displacement Array: [");
                    for (int j = 0; j < consecutive_count; j++) {
                        printf("%lf", displacement_array[j]);
                        if (j < consecutive_count - 1) {
                            printf(", ");
                        }
                    }
                    printf("]\n");
     consecutive_count++;
     if (consecutive_count==10){
        double displacement_diff = displacement_array[9]-displacement_array[0];
    
        if(displacement_diff<0){
          debug("apogee detected displacement");
        }
        else{
          debugln("apogee nope");
        }
        consecutive_count=0;
    }
    old_y_displacement = new_y_displacement;
      
    /* subtract current altitude to get the maximum height reached */

    float rocket_height = altimeter_data.altitude - ALTITUDE_OFFSET;

    /* update altimeter data */
    altimeter_data.velocity = y_velocity;
    altimeter_data.AGL = rocket_height;

  
  }


  /* ------------------------ END OF APOGEE DETECTION ALGORITHM ------------------------ */
    new_y_velocity = y_velocity;
    double velocity_diff = new_y_velocity - old_y_velocity;
    
    if(fabs(velocity_diff)<=1){
      debug("apogee detected velocity");
      debugln(fabs(velocity_diff));
    }
    else{
      debug("not there yet");
    }
    old_y_velocity = new_y_velocity;

}

void readGyroscope()
{

  sensors_event_t a, g, temp;
  gyroscope.getEvent(&a, &g, &temp);

  /*
   * Read accelerations on all axes
   */
  gyro_data.ax = a.acceleration.x;
  gyro_data.ay = a.acceleration.y;
  gyro_data.az = a.acceleration.z;
  gyro_data.gx = g.gyro.x;
  gyro_data.gy = g.gyro.y;
  gyro_data.gz = g.gyro.z;

  // FILTER THIS READINGS
}

struct GPS_Data gps_data;
void readGPS()
{
  /* This function reads GPS data and sends it to the ground station */

  while (hard.available() > 0)
  {
    gps.encode(hard.read());
  }
  if (gps.location.isUpdated())
  {
    gps_data.latitude = gps.location.lat();
    gps_data.longitude = gps.location.lng();
    gps_data.time = gps.time.value();
    fallBackLat = gps_data.latitude;
    fallBackLong = gps_data.longitude;
    debugln("[!!] GPS Data Received [!!]");
    // delay(TASK_DELAY);
  }
  else
  {
    gps_data.latitude = fallBackLat;
    gps_data.longitude = fallBackLong;
    gps_data.time = 20230601;
  }
}

void displayData()
{
  debugln("Reached displayData()");
  // struct Acceleration_Data gyroscope_data;
  // struct Altimeter_Data altimeter_data;
  // struct GPS_Data gps_data;
  // struct Acceleration_Data gyroscope_buffer;
  // struct Altimeter_Data altimeter_buffer;
  // struct GPS_Data gps_buffer;

  debugln("------------------------------");
  debug("x: ");
  debug(gyro_data.ax);
  debugln();
  debug("y: ");
  debug(gyro_data.ay);
  debugln();
  debug("z: ");
  debug(gyro_data.az);
  debugln();
  debug("roll: ");
  debug(gyro_data.gx);
  debugln();
  debug("pitch: ");
  debug(gyro_data.gy);
  debugln();
  debug("yaw: ");
  debug(gyro_data.gz);
  debugln();

  debug("Pressure: ");
  debug(altimeter_data.pressure);
  debugln();
  debug("Altitude: ");
  debug(altimeter_data.altitude);
  debugln();
  debug("Velocity: ");
  debug(altimeter_data.velocity);
  debugln();
  debug("AGL: ");
  debug(altimeter_data.AGL);
  debugln();

  debug("Latitude: ");
  debug(gps_data.latitude);
  debugln();
  debug("Longitude: ");
  debug(gps_data.longitude);
  debugln();
  debug("Time: ");
  debug(gps_data.time);
  debugln();

  // Add a delay to control the display rate (equivalent to the delay(10) in your original code)
  delay(10);
}
void transmitTelemetry()
{


  /* This function sends data to the ground station */

  // struct Acceleration_Data gyroscope_data_receive;
  // struct Altimeter_Data altimeter_data_receive;
  // struct GPS_Data gps_data_receive;
  // int32_t flight_state_receive;
  int id = 0;

  float ax = gyro_data.ax;
  float ay = gyro_data.ay;
  float az = gyro_data.az;
  float gx = gyro_data.gx;
  float gy = gyro_data.gy;
  float gz = gyro_data.gz;
  float AGL = altimeter_data.AGL;
  float altitude = altimeter_data.altitude;
  float velocity = altimeter_data.velocity;
  float pressure = altimeter_data.pressure;
  float latitude = gps_data.latitude;
  float longitude = gps_data.longitude;

  sprintf(telemetry_data,
          "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.2f,%.2f,%.2f,%.2f\n",
          ax,        // 1  -0.70
          ay,        // 2  0.24
          az,        // 3  10.59
          gx,        // 4  -0.07
          gy,        // 5   0.03
          gz,        // 6   0.04
          AGL,       // 7   46.97
          altitude,  // 8   1461.97
          velocity,  // 9   -0.00
          pressure,  // 10   84951.00
          altitude,  // 11    1461.97
          latitude,  // 12   -1.10
          longitude  // 13    37.01
  );
  file = SPIFFS.open("/log.csv", FILE_APPEND);
        if(!file) debugln("[-] Failed to open file for appending");
        else debugln("[+] File opened for appending");
  Filtered_Data result = filterData(gyro_data.ax, altimeter_data.AGL, altimeter_data.velocity);
  char formattedData[100];
  sprintf(formattedData, " %.2f, %.2f, %.2f",
          result.filtered_AGL,
          result.filtered_velocity,
          result.filtered_acceleration);
  // filtered_data transmit
  char combinedData[200]; // Adjust the size as needed
  sprintf(combinedData, "%s%s", telemetry_data, formattedData);
  if (file.print(telemetry_data))
  {
    debugln("[+] Message appended");
  }
  else
  {
    debugln("[-] Append failed");
  }
  file.close();
  id += 1;
  byte filteredBytes[strlen(formattedData)];
  strcpy((char *)filteredBytes, formattedData);
  int filteredLength = strlen(formattedData);

  //telemetry_data from character array to byte array
  byte telemetryBytes[strlen(telemetry_data)];
  strcpy((char *)telemetryBytes, telemetry_data);
  int telemetryLength = strlen(telemetry_data);
  lora1.beginPacket();
  lora1.write(destination);
  lora1.write(telemetryBytes, telemetryLength);
  lora1.write(filteredBytes, filteredLength);
  //return lora1.endPacket();

  lora1.endPacket();

  debug(telemetry_data);
  debugln(formattedData);
}
void setup()
{
  /* Initialize serial */
  Serial.begin(115200);
  SPI.begin();
  /* Setup GPS */
  // if (!SPIFFS.begin(true)) debugln("[-] An error occurred while mounting SPIFFS");
  // else debugln("[+] SPIFFS mounted successfully");
  pinMode(LED_PIN, OUTPUT);
  // hard.begin(9600, SERIAL_8N1, RX, TX);
  // SPI.setDataMode(SPI_MODE0);
  pinMode(HSPI_SS, OUTPUT);
  pinMode(VSPI_SS, OUTPUT);
  pinMode(VSPI_MISO, INPUT);

  /* Initialize sensors */
  initialize_gyroscope();
  initialize_altimeter();
  // read sensors
  debugln("reading sensors");
  // readAltimeter();
  // readGPS();
  // readGyroscope();
  // initializeLora2(); // receiver
}

void loop()
{

  readAltimeter();
  // readGPS();
  readGyroscope();
  // displayData();
  digitalWrite(VSPI_SS, LOW);
  delay(100);
  if (digitalRead(VSPI_SS) == LOW)
  {
    initializeLora1();
    debugln("lora 1 set");
    transmitTelemetry();
  }
  else
  {
    debugln("lora 1 not");
  }

  digitalWrite(VSPI_SS, HIGH);
  delay(100);

  byte receivedData[300]; // Adjust the buffer size as needed
  int bytesRead = 0;

  int packetSize = lora2.parsePacket();

  if (packetSize == 0)
    return; // No data received, exit
  else
  {
    digitalWrite(LED_PIN, HIGH); // Assuming LED_PIN is the pin connected to the LED
    Serial.println("LED turned on");
  }
  while (lora2.available())
  {
    receivedData[bytesRead] = lora2.read();
    bytesRead++;
  }

  // Process the received data here (e.g., print it)
  Serial.print("Received packet: ");
  for (int i = 0; i < bytesRead; i++)
  {
    Serial.print((char)receivedData[i]);
  }
  Serial.println();
  //   delay(100);
}

#include <SPI.h>
#include <LoRa.h>
#include "defs.h"
#include <PubSubClient.h>
#include <WiFi.h>
LoRaClass lora1; // Receiver
LoRaClass lora2; // Transmitter

WiFiClient wifi_client;


/* create MQTT publish-subscribe client */
PubSubClient client(wifi_client);
SPISettings spiSettings(500000, MSBFIRST, SPI_MODE0);
SPIClass vspi1; // Define VSPI object
SPIClass hspi;  // Define HSPI object
byte destination = 0xF4;
void setup_wifi()
{
    debugln();
    debug("Connecting to:");
    debugln(SSID);
    Serial.begin(115200);
    WiFi.begin(SSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED)
        ;

    {
        delay(500);
        debug(".");
    }
    randomSeed(micros());
    debugln("");
    debugln("WiFi connected");
    debugln("IP address: ");
    debugln(WiFi.localIP());
}

void reconnect()
{
    while (!client.connected())
    {
        Serial.begin(115200);
        debug("Attempting MQTT connection...");
        String clientId = "GClient-";
        clientId += String(random(0xffff), HEX);
        if (client.connect(clientId.c_str()))
        {
            debugln("Connected");
            client.subscribe("Control/Ejection");
        }
        else
        {
            debug("failed,rc=");
            debug(client.state());
            debugln("try again in 5 seconds");
            delay(5000);
        }
    }
}

void reconnectLora1() {
  while (!lora1.begin(433E6)) {
    debugln("Error initializing LoRa receiver. Retrying...");
    delay(500); // Wait for 5 seconds before retrying
  }
  Serial.println("LoRa receiver initialized successfully.");
}
void initializeLora1()
{

  lora1.setPins(VSPI_SS, VSPI_RST, VSPI_DI0); // RECEIVER
  if (!lora1.begin(433E6))
  {
    Serial.println("Error initializing LoRa receiver.");
    reconnectLora1(); 
  }
  lora1.setSyncWord(0xF1);
  lora1.setSpreadingFactor(8);     // Adjust SF as needed
  lora1.setSignalBandwidth(125E3); // Adjust bandwidth as needed
  lora1.setCodingRate4(5);         // Adjust coding rate as needed
  Serial.println("LoRa Receiver Initializing Successful!");

  // debugln("LoRa transmitters successfully initialized!");
}
void reconnectLora2() {
  while (!lora2.begin(433E6)) {
    Serial.println("Error initializing LoRa receiver. Retrying...");
    delay(500); // Wait for 5 seconds before retrying
  }
  Serial.println("LoRa receiver initialized successfully.");
}
void initializeLora2()
{

  hspi = SPIClass(VSPI);
  lora2.setPins(HSPI_SS, HSPI_RST, HSPI_DI0);
  lora2.setSPIFrequency(10000000);

  if (!lora2.begin(433E6))
  {
    Serial.println("Error initializing LoRa transmitter.");
    reconnectLora2(); // Halt the program if initialization fails
  }
  else
  {
    debugln("ok");
  }

  // Set a common sync word for communication between the modules
  lora2.setFrequency(433E6); // Set a different frequency for lora2 (within the same band)
  lora2.setSyncWord(0xF4); // Set a different sync word for lora2
  lora2.setSpreadingFactor(9); // Set a different SF for lora2 (adjust as needed)
  lora2.setSignalBandwidth(250E3); // Set a different bandwidth for lora2 (adjust as needed)
  lora2.setCodingRate4(7);         // Set a different coding rate for lora2 (adjust as needed)
  lora2.setTxPower(18);
  // debugln("LoRa transmitters successfully initialized!");
}
void onReceive(int packetSize)
{

  packetSize = lora1.parsePacket();
  if (packetSize == 0)
    return; // No data received, exit

  // Read the received data into a buffer
  byte receivedData[255]; // Adjust the buffer size as needed
  int bytesRead = 0;

  while (lora1.available())
  {
    receivedData[bytesRead] = lora1.read();
    bytesRead++;
  }

  // Process the received data here (e.g., print it)
  Serial.print("Received packet: ");
  for (int i = 0; i < bytesRead; i++)
  {
    Serial.print((char)receivedData[i]);
  }
  Serial.println();
}
void mqttCallback(char *topic, byte *message, unsigned int length)
{
    debug("Message arrived on topic:");
    debug(topic);
    debug("Message");
    String messageTemp;

    for (int i = 0; i < length; i++)
    {
        debug((char)message[i]);
    }
    debugln();
    
}
void setup()
{
  Serial.begin(115200);
  SPI.begin();
  pinMode(Push_Button, INPUT); 
  initializeLora1(); // receiver
   // Connect to Wi-Fi
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Configure the MQTT broker
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(mqttCallback);
  if(!client.connected()){
    reconnect();
  } 
}

void loop()
{
  // delay(100);
  int packetSize = lora1.parsePacket();

  if (packetSize == 0)
    return; // No data received, exit

  // Read the received data into a buffer
  byte receivedData[255]; // Adjust the buffer size as needed
  int bytesRead = 0;
  while (lora1.available())
  {
    receivedData[bytesRead] = lora1.read();
    bytesRead++;
  }

  // Process the received data here (e.g., print it)
  Serial.print("Received packet: ");
  for (int i = 0; i < bytesRead; i++)
  {
    Serial.print((char)receivedData[i]);
  }
  Serial.println();
  //  if(!client.connected()){
  //   reconnect();
  // } 
  char message[bytesRead + 1];
  memcpy(message,receivedData,bytesRead);
  message[bytesRead]= '\0';

  client.publish(topic,message);
  delay(100);
  // // }
  initializeLora2();
  digitalWrite(HSPI_SS, LOW);
  if (digitalRead(HSPI_SS) == LOW)
  {

    debugln("lora2");

    // const char *telemetry_data = "1234"; // Define and initialize telemetry_data
    // byte telemetryBytes[strlen(telemetry_data)];
    // strcpy((char *)telemetryBytes, telemetry_data);
    // int telemetryLength = strlen(telemetry_data);
    // lora2.beginPacket();
    // lora2.write(destination);
    // lora2.write(telemetryBytes, telemetryLength);
    // lora2.endPacket();
    // Serial.println("Acknowledgment sent");
    
    Button_State = digitalRead(Push_Button); 
    Serial.println(Button_State);
    const char *manual_override = "123";
    byte manualBytes[strlen(manual_override)];
    strcpy((char *)manualBytes, manual_override);
    if (Button_State == HIGH)
    {                              /*if condition to check button status*/
      int manualLength = strlen(manual_override);
      lora2.beginPacket();
      lora2.write(destination);
      lora2.write(manualBytes,manualLength);
      lora2.endPacket();
      debugln("manual sent");
    }
  }
  digitalWrite(HSPI_SS, HIGH);
 
 
}


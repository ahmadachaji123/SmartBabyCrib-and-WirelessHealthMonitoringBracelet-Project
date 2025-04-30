#include <esp_now.h>
#include <WiFi.h>
#include <HardwareSerial.h>

#define TXD2 17
#define RXD2 16
HardwareSerial SerialToArduino(2);

// Use the same structure that the sender uses
typedef struct __attribute__((packed)) {
  uint16_t bodyTemp;     // Already stored as integer (value * 100)
  uint16_t ambientTemp;  // Already stored as integer (value * 100)
  uint8_t heartRate;
  uint8_t oxygenLevel;
  uint8_t batteryLevel;
  uint8_t batteryVolt;   // Already stored as integer (value * 10)
} SensorData;

// Create a structure to hold the received data
SensorData receivedData;

// ESP-NOW callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  
  // Print received data for debugging (converting back to floating point for display)
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Body Temperature: ");
  Serial.println(receivedData.bodyTemp / 100.0, 2);
  Serial.print("Ambient Temperature: ");
  Serial.println(receivedData.ambientTemp / 100.0, 2);
  Serial.print("Heart Rate: ");
  Serial.println(receivedData.heartRate);
  Serial.print("Oxygen Level: ");
  Serial.println(receivedData.oxygenLevel);
  Serial.print("Battery Level: ");
  Serial.println(receivedData.batteryLevel);
  Serial.print("Battery Voltage: ");
  Serial.println(receivedData.batteryVolt / 10.0, 1);
  Serial.println();
  
  // Forward data to Arduino directly
  sendDataToArduino();
}

// Function to send data to Arduino
void sendDataToArduino() {
  // Send sensor data with tag
  SerialToArduino.write(0xAA); // Start byte
  SerialToArduino.write(0x01); // Tag for sensor data
  SerialToArduino.write((uint8_t*)&receivedData, sizeof(receivedData));
  Serial.println("ESP32: Sent sensor data to Arduino");
}

void setup() {
  // Initialize Serial Monitor for debugging
  Serial.begin(115200);
  
  // Initialize Serial connection to Arduino
  SerialToArduino.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callback function
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  
  Serial.println("ESP32 initialized and ready to receive ESP-NOW data");
}

void loop() {
  // Optional: Send periodic commands to Arduino
  static unsigned long lastCommandTime = 0;
  if (millis() - lastCommandTime > 5000) {
    SerialToArduino.write(0xAA); // Start byte
    SerialToArduino.write(0x02); // Tag for command
    SerialToArduino.print("LED_ON\n");
    Serial.println("ESP32: Sent command to Arduino");
    
    lastCommandTime = millis();
  }
}
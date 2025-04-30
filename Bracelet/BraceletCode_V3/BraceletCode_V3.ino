#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MLX90614.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <esp_now.h>
#include <WiFi.h>

MAX30105 particleSensor;

#define Battery_adcPin 3
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

struct SensorData {
  float ambientTempC;
  float objectTempC;
  float coreTempC;  // Calibrated core (axillary) temperature in Celsius
};
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ADC characteristics

#define VoltageBattery_Calibration 0.71  // the lowest the higher the voltage 
const int Battery_adcMaxValue = 4096;  // Arduino (1023) - Esp (4096)
const float referenceVoltage = 3.3;    // Arduino (5V) - Esp (3.3 v)
//const int numReadings_Battery = 10;    // Number of readings for averaging
#define numReadings_Battery 100  // Number of stored readings
// Voltage divider ratio
const float Battery_voltageDividerRatio = 0.25;
// LiPo battery characteristics
const float minBatteryVoltage = 3.0;  // Minimum voltage (0% charge)
const float maxBatteryVoltage = 4.2;  // Maximum voltage (100% charge)

float voltageReadings[numReadings_Battery] = {0};  // FIFO buffer for voltage values
int currentIndex = 0;  // Track the current position in the buffer
int sampleCount = 0;   // Counts the number of readings taken
float sumVoltage = 0;  // Running sum of stored values

//////insde code variables///////////////////////////////////////////////////////////////////////////////////////////////////

int HeartRate = 0;
int32_t O2 = 0;
int batteryLevel = 0;
float batteryVoltage_FV =0 ; 
SensorData IR_data;
bool isConnected = false;

int Minimum_accepted_o2 = 90;
int Minimum_accepted_HR = 40;

unsigned long previousMillis = 0;  // Store the last time the function was called

/////////////////////////////////////////////////////////////////////////////////////////////////
long irValue = 0;
//presence sensing ////////////////////////////////////////////////////////////////////////////////////
bool presenceState = 0;
//Heart rate///////////////////////////////////////////////////////////////////////////////////////////
bool Startup1 = 0;
const byte RATE_SIZE = 16;  // Increased window for averaging
byte rates[RATE_SIZE];      // Array to store heart rate samples
byte rateSpot = 0;
long lastBeat = 0;  // Time at which the last beat occurred
float beatsPerMinute = 0;
int beatAvg = 0;
// Variables for exponential moving average (low-pass filter)
float filteredBPM = 0;
const float alpha = 0.9;  // Smoothing factor (closer to 1 gives smoother, slower response)
//////o2////////////////////////////////////////////////////////////////////////////////////////////////////
bool Startup2 = 0;
int count = 0;
bool skip = 0;
int false_reading_count =0;
#define MAX_BRIGHTNESS 255
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100];   //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100];   //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif
int32_t bufferLength;   //data length
int32_t spo2;           //SPO2 value
int8_t validSPO2;       //indicator to show if the SPO2 calculation is valid
int32_t heartRate;      //heart rate value
int8_t validHeartRate;  //indicator to show if the heart rate calculation is valid
////////////////////////////////////////////////////////////////////////////////

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = { 0xe8, 0x6b, 0xea, 0xd4, 0x7b, 0x10 };

// Optimized structure to send data via ESP-NOW that's already compatible with Arduino
typedef struct __attribute__((packed)) {
  uint16_t bodyTemp;     // Store as integer (value * 100) for 2 decimal places
  uint16_t ambientTemp;  // Store as integer (value * 100) for 2 decimal places
  uint8_t heartRate;     // BPM as uint8_t (0-255)
  uint8_t oxygenLevel;   // SpO2 as uint8_t (0-100%)
  uint8_t batteryLevel;  // Battery percentage as uint8_t (0-100%)
  uint8_t batteryVolt;   // Battery voltage * 10 for 1 decimal place
} struct_message;

// Create a struct_message called myData
struct_message myData;
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  
  Wire.begin();  // for esp32 Wire.begin(SDA,SCL) (9,10)
  Serial.begin(11520);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED initialization failed");
    while (1)
      ;
  }
  ///mlx.writeEmissivity(0.98); // Set emissivity to 0.98 for skin
  if (!mlx.begin()) {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    while (1)
      ;
  }
  Serial.println("MAX30102 Presence Sensing Example");
  if (particleSensor.begin(Wire,100000) == false)  //Use default I2C port, 100kHz speed
  {
    Serial.println("MAX30102 was not found. Please check wiring/power. ");
    while (1)
      ;
  }

  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Transmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0F);  // Increased from 0x0A
  particleSensor.setPulseAmplitudeGreen(0);   // Turn off green LED

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(32, 32);
  display.print("Hello");
  display.display();
}
void loop() {
  irValue = particleSensor.getIR();
  presenceState = presenceSensing();
  if (presenceState) {
    IR_data = readSensorData();
    batteryVoltage_FV =batteryVoltage_2();
    batteryLevel = batteryPercentage_L();
    HeartRate = heartRateSensing();
    O2 = O2_reading_with_delay(2000);
   // Serial.print("IR=");
    //Serial.print(irValue);
    //Serial.print("  Avg BPM=");
    //Serial.print(HeartRate);
    //Serial.print("  O2=");
    //Serial.println(O2);

    //Serial.print("Ambient T=");
    //Serial.print(IR_data.ambientTempC);
    //Serial.print("  Core T=");
    //Serial.print(IR_data.coreTempC);
    //Serial.println(" *C");

    //Serial.print("V | Battery Percentage: ");
    //Serial.print(batteryLevel, 1);
    ///Serial.println("%");
    Display_and_send_with_Delay(3000);  //sending and displayin data every 3 seconds
  }

  else {
    display.clearDisplay();
    display.setTextSize(1.5);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 32);
    display.print("No Object detected");
    display.display();
  }
}

void Display_and_send_with_Delay(const unsigned long interval) {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // Update the last execution time
    drawDisplay();
    EspNow_sending();
  }
}
void drawDisplay() {

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Bat:");
  display.print(batteryLevel);
  display.print("%");

  display.setCursor(90, 0);
  display.print(batteryVoltage_FV);
  display.print("V");
 // display.print(isConnected ? "Connected" : "Disconnected");

  display.setCursor(30, 20);
  display.print("Temp:");
  display.print(IR_data.coreTempC);
  display.print(" C");

  display.setCursor(36, 35);
  display.print("HR:");
  display.print(HeartRate);
  display.print(" bpm");

  display.setCursor(37, 50);
  display.print("SpO2:");
  display.print(O2);
  display.print("%");

  display.display();
}

SensorData readSensorData() {  //temperature data
  SensorData data;
  data.ambientTempC = mlx.readAmbientTempC();
  data.objectTempC = mlx.readObjectTempC();
  data.coreTempC = calculateCoreTemp(data.objectTempC, data.ambientTempC);

  //printIRSensorData(data);
  return data;
}
float calculateCoreTemp(float foreheadTemp, float ambientTemp) {
  return 0.296 * foreheadTemp - 0.018 * ambientTemp + 28.335;
}

void printIRSensorData(const SensorData &data) {
  Serial.print("Ambient = ");
  Serial.print(data.ambientTempC);
  Serial.print(" *C\tObject (Forehead) = ");
  Serial.print(data.objectTempC);
  Serial.println(" *C");

  // Print the calibrated core temperature in Celsius
  Serial.print("Calibrated Core (Axillary) Temperature = ");
  Serial.print(data.coreTempC);
  Serial.println(" *C");
  Serial.println();
}

float batteryVoltage_2() {
  // Read the ADC value
  int adcValue = analogRead(Battery_adcPin);

  // Convert to voltage
  float measuredVoltage = (adcValue / (float)Battery_adcMaxValue) * referenceVoltage;
  float batteryVoltage = measuredVoltage / Battery_voltageDividerRatio;
  batteryVoltage /= VoltageBattery_Calibration;  // Apply calibration

  // Add new value and update the sum
  sumVoltage -= voltageReadings[currentIndex];  // Remove the oldest value from the sum
  voltageReadings[currentIndex] = batteryVoltage;  // Store new reading in FIFO buffer
  sumVoltage += batteryVoltage;  // Add the new value to the sum

  // Move to the next position in a circular manner
  currentIndex = (currentIndex + 1) % numReadings_Battery;

  // Increase sample counter
  sampleCount++;

  // Compute and return the average only when enough samples are collected
  if (sampleCount >= numReadings_Battery) {
    float averageVoltage = sumVoltage / numReadings_Battery;

    // Debugging output
    //Serial.print("ADC: ");
    //Serial.print(adcValue);
    //Serial.print(" | Raw Voltage: ");
    //Serial.print(measuredVoltage, 2);
    //Serial.print("V | Smoothed Battery Voltage: ");
    //Serial.print(averageVoltage, 2);
    //Serial.println("V");

    return averageVoltage;
  }

  return batteryVoltage_FV;  // Return -1 to indicate average is not yet ready
}
float batteryVoltage_1() {
  int totalAdcValue = 0;
  for (int i = 0; i < numReadings_Battery; i++) {
    totalAdcValue += analogRead(Battery_adcPin) ;
   // totalAdcValue += (analogRead(Battery_adcPin) * 0.9);
  }
  int adcValue = totalAdcValue / numReadings_Battery;  //average
  float measuredVoltage = (adcValue / (float)Battery_adcMaxValue) * referenceVoltage;
  float batteryVoltage = measuredVoltage / Battery_voltageDividerRatio;

  batteryVoltage = batteryVoltage / VoltageBattery_Calibration ;

  Serial.print("ADC Value: ");
  Serial.print(adcValue);
  Serial.print(" | Measured Voltage: ");
  Serial.print(measuredVoltage, 2);
  Serial.print("V | Battery Voltage: ");
  Serial.print(batteryVoltage, 2);

  return batteryVoltage;
}

int batteryPercentage_NL() {
  float batteryVoltage = batteryVoltage_1();
  int batteryPercentage = 0;

  if (batteryVoltage >= 4.2) batteryPercentage = 100.0;
  else if (batteryVoltage >= 4.15) batteryPercentage = 95 + (batteryVoltage - 4.15) * 5 / 0.05;
  else if (batteryVoltage >= 4.1) batteryPercentage = 90 + (batteryVoltage - 4.1) * 5 / 0.05;
  else if (batteryVoltage >= 4.0) batteryPercentage = 80 + (batteryVoltage - 4.0) * 10 / 0.1;
  else if (batteryVoltage >= 3.9) batteryPercentage = 60 + (batteryVoltage - 3.9) * 20 / 0.1;
  else if (batteryVoltage >= 3.8) batteryPercentage = 40 + (batteryVoltage - 3.8) * 20 / 0.1;
  else if (batteryVoltage >= 3.7) batteryPercentage = 20 + (batteryVoltage - 3.7) * 20 / 0.1;
  else if (batteryVoltage >= 3.5) batteryPercentage = 10 + (batteryVoltage - 3.5) * 10 / 0.2;
  else if (batteryVoltage >= 3.3) batteryPercentage = (batteryVoltage - 3.3) * 10 / 0.2;
  else batteryPercentage = 0;

  //to make the readings more stable
  if ((abs(batteryLevel - batteryPercentage) <= 20))
    batteryPercentage = batteryLevel;

  Serial.print("V | Battery Percentage: ");
  Serial.print(batteryPercentage, 1);
  Serial.println("%");

  return batteryPercentage;
}
int batteryPercentage_L() {
  float batteryVoltage = batteryVoltage_2() ; // Get measured voltage
  int batteryPercentage = 0;

  // Ensure voltage is within the expected range
  if (batteryVoltage < 3.3) batteryVoltage = 3.3;
  if (batteryVoltage > 4.2) batteryVoltage = 4.2;

  // Linear mapping from 3.3V - 4.2V to 0% - 100%
  batteryPercentage = ((batteryVoltage - 3.3) / (4.2 - 3.3)) * 100;

  // Stability filter (optional)
  if (abs(batteryLevel - batteryPercentage) <= 5) {
    batteryPercentage = batteryLevel;
  }

  //Serial.print("V | Battery Percentage: ");
  //Serial.print(batteryPercentage);
  //Serial.println("%");

  return batteryPercentage;
}

bool presenceSensing() {
  bool pressenceState_readings = false;
  if (irValue > 40000) {
    //Serial.println("Object detected");
    pressenceState_readings = 1;
  } else {
    pressenceState_readings = 0;
    Serial.println("Put your fingure please");
  }
  return pressenceState_readings;
}

int heartRateSensing() {

  if (Startup1 == 0) {
    // Initialize filtered BPM.
    filteredBPM = 0;
    Startup1 = 1;
  }

  int HeartRate_readings_filtered = 0;
  //long irValue = particleSensor.getIR();
  // Check if a beat is detected.

  if (checkForBeat(irValue) == true) {
    // We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();
    // Calculate BPM from the time between beats.
    beatsPerMinute = 60 / (delta / 1000.0);
    // Validate BPM range.
    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute;  // Store this reading.
      rateSpot %= RATE_SIZE;                     // Wrap the array index.

      // Compute average BPM over the stored samples.
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++) {
        beatAvg += rates[x];
      }
      beatAvg /= RATE_SIZE;

      // Apply an exponential moving average filter for smoother output.
      if (filteredBPM == 0) {
        filteredBPM = beatsPerMinute;  // Initialize on the first valid reading.
      } else {
        filteredBPM = alpha * filteredBPM + (1 - alpha) * beatsPerMinute;
      }
    }
  }
  // Serial.print("IR=");
  // Serial.print(irValue);
  // Serial.print(", BPM=");
  // Serial.print(beatsPerMinute, 2);
  // Serial.print(", Filtered BPM=");
  // Serial.print(filteredBPM, 2);
  // Serial.print(", Avg BPM=");
  // Serial.print(beatAvg);
  // Serial.println();

  if (beatAvg >= Minimum_accepted_HR) {  //filtering values that are less then 'Minimum_accepted_HR'
    HeartRate_readings_filtered = beatAvg;
  }
  return HeartRate_readings_filtered;
}

int32_t O2_reading_with_delay(int Delay_iteration_nb) {  // the aim of this function is to simply take a acceptable value of the SPO2 and after that skip the reading of it for specific amount of loops
  int32_t O2_radings_filtered = O2;
  if (!skip) {
    int32_t O2_readings = O2value();
    if (O2_readings >= Minimum_accepted_o2) {
      O2_radings_filtered = O2_readings;
      skip = true;
      false_reading_count =0; 
      Serial.println( "O2 value has captures;    Skip on");
    }
    else false_reading_count ++ ;
  } 
  else {
    count++;
  }
   Serial.print("Count false readings: ");
   Serial.println( false_reading_count);
   Serial.print("Count value: ");
   Serial.println( count);

  if (false_reading_count >= (20) ){ 
    false_reading_count=0;
     skip = true; 
     Serial.println("Skiping by force");
     }

  if (count >= Delay_iteration_nb) {
    Serial.println("reset count");
    count = 0;
    skip = false;
  }
  return O2_radings_filtered;
}
int32_t O2value() {

  if (Startup2 == 0) {
    bufferLength = 50;  //buffer length of 100 stores 4 seconds of samples running at 25sps

    //read the first 100 samples, and determine the signal range
    for (byte i = 0; i < bufferLength; i++) {
      while (particleSensor.available() == false)  //do we have new data?
        particleSensor.check();                    //Check the sensor for new data

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample();  //We're finished with this sample so move to next sample

      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.println(irBuffer[i], DEC);
    }

    //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
    Startup2 = 1;
  }

  //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
  for (byte i = 25; i < 100; i++) {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25] = irBuffer[i];
  }

  //take 25 sets of samples before calculating the heart rate.
  for (byte i = 75; i < 100; i++) {
    while (particleSensor.available() == false)  //do we have new data?
      particleSensor.check();                    //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();  //We're finished with this sample so move to next sample

    //Serial.print(F(", SPO2="));
    //Serial.println(spo2, DEC);
  }

  //After gathering 25 new samples recalculate HR and SP02
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  return spo2;
}
void EspNow_sending() {
  // Convert floating point values to fixed-point integers
  myData.bodyTemp = (uint16_t)(IR_data.coreTempC * 100);     // e.g., 37.25°C becomes 3725
  myData.ambientTemp = (uint16_t)(IR_data.ambientTempC * 100); // e.g., 22.50°C becomes 2250
  
  // Convert integers with bounds checking
  myData.heartRate = (HeartRate > 255) ? 255 : (uint8_t)HeartRate;
  myData.oxygenLevel = (O2 > 100) ? 100 : (uint8_t)O2;
  myData.batteryLevel = (batteryLevel > 100) ? 100 : (uint8_t)batteryLevel;
  
  // Convert voltage with single decimal place
  myData.batteryVolt = (uint8_t)(batteryVoltage_FV * 10); // e.g., 3.7V becomes 37
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  
  if (result == ESP_OK) {
    Serial.println("Sent with success");
    isConnected = true;
  } else {
    Serial.println("Error sending the data");
    isConnected = false;
  }
}
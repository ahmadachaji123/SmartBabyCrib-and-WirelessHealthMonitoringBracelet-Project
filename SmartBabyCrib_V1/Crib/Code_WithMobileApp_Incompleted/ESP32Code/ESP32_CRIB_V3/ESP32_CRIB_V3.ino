#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <Preferences.h>
#include <time.h>
#include <HardwareSerial.h>
#include <esp_now.h>


#define TXD2 17
#define RXD2 16
HardwareSerial SerialToArduino(2);


// New structure to receive data

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
}


// ===== CONFIGURATION =====
// WiFi and Firebase credentials
const char* ssid = "Ahmadac";        //TP-Link_09F8        Ahmadac    .    hawAh
const char* password = "AhmaAd123";  // 18230888    AhmaAd123     ahmadzouhair
#define API_KEY "AIzaSyCa8Zl1aMVWQYy5v_6K0TBR4bgWNRXe9DM"
#define DATABASE_URL "https://fir-iot-9d8f7-default-rtdb.firebaseio.com/"

// NTP configuration
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 0;

// Application settings
const long dataInterval = 10000;  // Send data every 10 seconds
bool debugMode = true;

// Connection retry settings
const int maxWiFiAttempts = 20;
const int maxNtpRetries = 3;
const int maxFirebaseAttempts = 20;

// ===== GLOBAL OBJECTS =====
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
Preferences preferences;

// ===== STATE VARIABLES =====
// Data point tracking
int heartRateDataPoint;
int oxygenDataPoint;
int temperatureDataPoint;
int humidityDataPoint;

// Timer
unsigned long previousMillis = 0;


// Firebase paths
String CMD_manualPath = "FirebaseIOT/CMD/manualMode/";
String CMD_autoPath = "FirebaseIOT/CMD/autoMode/";
//////////////////////////////Flags
// Manual Mode Flags (as Strings)
String autoModeFlag;
String heatingCoolingFlag;
String lightFlag;
String musicFlag;
String swingingFlag;
String swingingSpeedFlag;
String temperatureSetPointFlag;
// Auto Mode Flags (as Strings)
String autoSwingingFlag;
String autoLightFlag;
String autoMusicFlag;
String autoSwingingSpeedFlag;
//////////////////////////////////////




// ===== CONNECTIVITY FUNCTIONS =====
bool setupWiFi() {
  debug("Connecting to WiFi...");

  WiFi.mode(WIFI_STA);   // Fix: ensure station mode
  WiFi.setSleep(false);  // Fix: disable power save mode
  delay(500);            // Fix: small delay before starting connection
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < maxWiFiAttempts) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    debug("\n❌ WiFi connection failed after " + String(attempts) + " attempts");
    return false;
  }

  debug("\n✅ WiFi connected! IP: " + WiFi.localIP().toString());
  return true;
}
bool setupNTP() {
  debug("Configuring NTP time sync...");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // Try to sync with NTP
  int ntpAttempts = 0;
  struct tm timeinfo;
  bool timeSuccess = false;

  while (!timeSuccess && ntpAttempts < 3) {
    ntpAttempts++;
    debug("NTP sync attempt " + String(ntpAttempts) + "...");

    // Wait for time sync
    delay(500);

    if (getLocalTime(&timeinfo)) {
      timeSuccess = true;
      char timeStringBuff[30];
      strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%d %H:%M:%S", &timeinfo);
      debug("✅ Time synchronized with NTP: " + String(timeStringBuff));
    } else {
      debug("⚠️ NTP time sync failed, retrying...");
    }
  }
  if (!timeSuccess) {
    debug("❌ NTP time sync failed after " + String(ntpAttempts) + " attempts");
    return false;
  }

  return true;
}
bool setupFirebase() {
  debug("Setting up Firebase connection...");

  // Configure Firebase
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  // Anonymous sign-in
  if (!Firebase.signUp(&config, &auth, "", "")) {
    debug("❌ Firebase authentication failed: " + fbdo.errorReason());
    return false;
  }

  debug("✅ Firebase authentication successful");

  // Initialize connection
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  // Wait for connection
  int attempts = 0;
  debug("Waiting for Firebase connection");
  while (!Firebase.ready() && attempts < maxFirebaseAttempts) {
    Serial.print(".");
    delay(500);
    attempts++;
  }

  if (!Firebase.ready()) {
    debug("\n❌ Firebase connection failed after " + String(attempts) + " attempts");
    return false;
  }

  debug("\n✅ Firebase ready!");

  // Test write to verify connection
  String testPath = "FirebaseIOT/test";
  if (Firebase.RTDB.setString(&fbdo, testPath, "ESP32 connected at " + getISOTimestamp())) {
    debug("✅ Firebase test write successful");
    return true;
  } else {
    debug("❌ Firebase test write failed: " + fbdo.errorReason());
    return false;
  }
}
// ===== UTILITY FUNCTIONS =====
// Print debug messages if debug mode is on
void debug(String message) {
  if (debugMode) {
    Serial.println(message);
  }
}

// Get ISO 8601 timestamp for Firebase data
String getISOTimestamp() {
  time_t now = time(nullptr);
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  char timeStringBuff[30];
  strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(timeStringBuff);
}

// ===== PERSISTENT STORAGE FUNCTIONS =====
void initializeCounters() {
  preferences.begin("sensor_data", false);
  heartRateDataPoint = preferences.getInt("heartRate", 1);
  oxygenDataPoint = preferences.getInt("oxygen", 1);
  temperatureDataPoint = preferences.getInt("temperature", 1);
  humidityDataPoint = preferences.getInt("humidity", 1);

  debug("✅ Counters loaded from storage:");
  debug(" - Heart Rate: " + String(heartRateDataPoint));
  debug(" - Oxygen: " + String(oxygenDataPoint));
  debug(" - Temperature: " + String(temperatureDataPoint));
  debug(" - Humidity: " + String(humidityDataPoint));
}
void resetAllCounters() {  // Reset all counters to 1
  preferences.begin("sensor_data", false);

  // Reset in-memory values
  heartRateDataPoint = 1;
  oxygenDataPoint = 1;
  temperatureDataPoint = 1;
  humidityDataPoint = 1;

  // Reset stored values
  preferences.putInt("heartRate", 1);
  preferences.putInt("oxygen", 1);
  preferences.putInt("temperature", 1);
  preferences.putInt("humidity", 1);

  preferences.end();
}
void showCurrentCounters() {
  debug("Current counter values:");
  debug(" - Heart Rate: " + String(heartRateDataPoint));
  debug(" - Oxygen: " + String(oxygenDataPoint));
  debug(" - Temperature: " + String(temperatureDataPoint));
  debug(" - Humidity: " + String(humidityDataPoint));
}

// ===== DATA FUNCTIONS =====
// Send data for a single sensor type
bool sendTimedSensorData(String sensorType, float value, String timestamp) {
  // Get the appropriate data point counter
  int dataPoint = 1;  // Default

  if (sensorType == "heartRate") {
    dataPoint = heartRateDataPoint;
  } else if (sensorType == "oxygen") {
    dataPoint = oxygenDataPoint;
  } else if (sensorType == "temperature") {
    dataPoint = temperatureDataPoint;
  } else if (sensorType == "humidity") {
    dataPoint = humidityDataPoint;
  }

  // Create Firebase path
  String path = "FirebaseIOT/readings/sensors/timed/" + sensorType + "/dataPoint" + String(dataPoint);

  // Send value and timestamp
  bool success = false;

  if (sensorType == "temperature") {
    // Temperature is a float
    success = Firebase.RTDB.setFloat(&fbdo, path + "/value", value);
  } else {
    // All others are integers
    success = Firebase.RTDB.setInt(&fbdo, path + "/value", (int)value);
  }

  if (success && Firebase.RTDB.setString(&fbdo, path + "/timestamp", timestamp)) {
    // Update counter in preferences
    if (sensorType == "heartRate") {
      preferences.putInt("heartRate", ++heartRateDataPoint);
    } else if (sensorType == "oxygen") {
      preferences.putInt("oxygen", ++oxygenDataPoint);
    } else if (sensorType == "temperature") {
      preferences.putInt("temperature", ++temperatureDataPoint);
    } else if (sensorType == "humidity") {
      preferences.putInt("humidity", ++humidityDataPoint);
    }

    return true;
  } else {
    debug("❌ Failed to send " + sensorType + " data: " + fbdo.errorReason());
    return false;
  }
}
// Check connections and send all sensor data
void sendDataToFirebase() {
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    debug("❌ WiFi disconnected! Reconnecting...");
    WiFi.begin(ssid, password);
    delay(3000);
    if (WiFi.status() != WL_CONNECTED) {
      debug("WiFi reconnection failed, skipping this data cycle");
      return;
    }
    debug("WiFi reconnected successfully");
  }
  // Check Firebase connection
  if (!Firebase.ready()) {
    debug("❌ Firebase not ready! Skipping this data cycle");
    return;
  }

  // Get current timestamp
  String timestamp = getISOTimestamp();

  // Generate sensor values once to be used for both raw and timed data
  int heartRate = receivedData.heartRate;
  int oxygen = receivedData.oxygenLevel;
  float temperature = receivedData.bodyTemp / 100.0, 2;
  int humidity = random(40, 80);
  bool appearance = random(0, 2) == 1;
  bool crying = random(0, 2) == 1;

  // Send timed sensor readings
  bool success = true;
  success &= sendTimedSensorData("heartRate", heartRate, timestamp);
  success &= sendTimedSensorData("oxygen", oxygen, timestamp);
  success &= sendTimedSensorData("temperature", temperature, timestamp);
  success &= sendTimedSensorData("humidity", humidity, timestamp);

  // Update raw sensor values with the same readings
  String rawPath = "FirebaseIOT/readings/sensors/raw";
  success &= Firebase.RTDB.setBool(&fbdo, rawPath + "/appearanceState", appearance);
  success &= Firebase.RTDB.setBool(&fbdo, rawPath + "/cryingState", crying);
  success &= Firebase.RTDB.setInt(&fbdo, rawPath + "/oxygen", oxygen);
  success &= Firebase.RTDB.setInt(&fbdo, rawPath + "/humidity", humidity);
  success &= Firebase.RTDB.setFloat(&fbdo, rawPath + "/temperature", temperature);

  if (success) {
    debug("✅ All data sent successfully with consistent values:");
    debug(" - Heart Rate: " + String(heartRate) + " BPM");
    debug(" - Oxygen: " + String(oxygen) + "%");
    debug(" - Temperature: " + String(temperature) + "°C");
    debug(" - Humidity: " + String(humidity) + "%");
    debug(" - Appearance State: " + String(appearance ? "true" : "false"));
    debug(" - Crying State: " + String(crying ? "true" : "false"));
    debug("-------------------------");
    showCurrentCounters();
  } else {
    debug("❌ Some data failed to send");
  }
}
void sendStatusToFirebase() {
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    debug("❌ WiFi disconnected! Reconnecting...");
    WiFi.begin(ssid, password);
    delay(3000);
    if (WiFi.status() != WL_CONNECTED) {
      debug("WiFi reconnection failed, skipping status data");
      return;
    }
    debug("WiFi reconnected successfully");
  }

  // Check Firebase connection
  if (!Firebase.ready()) {
    debug("❌ Firebase not ready! Skipping status data");
    return;
  }

  // Simulated status values — replace with actual ones in your code
  bool automode_status = true;
  bool heatingCooling_status = false;
  bool light_status = true;
  bool swinging_status = false;
  int music_status = 2;
  float temperatureSetPoint_status = 37.5;

  bool autoModeSettings_light_status = true;
  bool autoModeSettings_swinging_status = false;
  int autoModeSettings_music_status = 1;

  // Base path for status
  String basePath = "/FirebaseIOT/readings/status";

  bool success = true;
  success &= Firebase.RTDB.setBool(&fbdo, basePath + "/automode", automode_status);
  success &= Firebase.RTDB.setBool(&fbdo, basePath + "/heatingCooling", heatingCooling_status);
  success &= Firebase.RTDB.setBool(&fbdo, basePath + "/light", light_status);
  success &= Firebase.RTDB.setBool(&fbdo, basePath + "/swinging", swinging_status);
  success &= Firebase.RTDB.setInt(&fbdo, basePath + "/music", music_status);
  success &= Firebase.RTDB.setFloat(&fbdo, basePath + "/temperatureSetPoint", temperatureSetPoint_status);

  // Automode settings
  String autoPath = basePath + "/automodeSettings";
  success &= Firebase.RTDB.setBool(&fbdo, autoPath + "/light", autoModeSettings_light_status);
  success &= Firebase.RTDB.setBool(&fbdo, autoPath + "/swinging", autoModeSettings_swinging_status);
  success &= Firebase.RTDB.setInt(&fbdo, autoPath + "/music", autoModeSettings_music_status);

  if (success) {
    debug("✅ All status values sent successfully.");
    Serial.println("📤 Sent Status Values:");
    Serial.print("automode: ");
    Serial.println(automode_status);
    Serial.print("heatingCooling: ");
    Serial.println(heatingCooling_status);
    Serial.print("light: ");
    Serial.println(light_status);
    Serial.print("swinging: ");
    Serial.println(swinging_status);
    Serial.print("music: ");
    Serial.println(music_status);
    Serial.print("temperatureSetPoint: ");
    Serial.println(temperatureSetPoint_status);
    Serial.print("automodeSettings/light: ");
    Serial.println(autoModeSettings_light_status);
    Serial.print("automodeSettings/swinging: ");
    Serial.println(autoModeSettings_swinging_status);
    Serial.print("automodeSettings/music: ");
    Serial.println(autoModeSettings_music_status);
  } else {
    debug("❌ Some status values failed to send.");
  }
}

void ReadFlags_SendToArduino() {
  // Read manual mode flags
  readFirebaseString(CMD_manualPath + "autoModeFlag", autoModeFlag);
  readFirebaseString(CMD_manualPath + "heatingCoolingFlag", heatingCoolingFlag);
  readFirebaseString(CMD_manualPath + "lightFlag", lightFlag);
  readFirebaseString(CMD_manualPath + "musicFlag", musicFlag);
  readFirebaseString(CMD_manualPath + "swingingFlag", swingingFlag);
  readFirebaseString(CMD_manualPath + "swingingSpeedFlag", swingingSpeedFlag);
  readFirebaseString(CMD_manualPath + "temperatureSetPointFlag", temperatureSetPointFlag);
  // Read auto mode flags
  readFirebaseString(CMD_autoPath + "swingingFlag", autoSwingingFlag);
  readFirebaseString(CMD_autoPath + "lightFlag", autoLightFlag);
  readFirebaseString(CMD_autoPath + "musicFlag", autoMusicFlag);
  readFirebaseString(CMD_autoPath + "swingingSpeedFlag", autoSwingingSpeedFlag);
  // Debug log
  debug("📥 Flags updated:");
  debug("Manual -> light: " + lightFlag + ", swing: " + swingingFlag + ", music: " + musicFlag + ", speed: " + swingingSpeedFlag);
  debug("Auto   -> light: " + autoLightFlag + ", swing: " + autoSwingingFlag + ", music: " + autoMusicFlag + ", speed: " + autoSwingingSpeedFlag);

  SendCMDToArduino();
}
void readFirebaseString(String path, String& targetVariable) {
  if (Firebase.RTDB.getString(&fbdo, path)) {
    targetVariable = fbdo.stringData();

    targetVariable.replace("\"", "");  // Remove quotes
    targetVariable.replace("\\", "");  // Remove backslashes
    targetVariable.trim();             // Remove whitespace or newlines

    debug("✅ Firebase read success - " + path + ": " + targetVariable);
  } else {
    debug("❌ Firebase read failed - " + path + ": " + fbdo.errorReason());
    targetVariable = "";
  }
}

void checkAndSendCommand_manualMode(String flagValue, String flagPath, String valuePath, String commandName) {
  if (flagValue == "1") {
    String tempString;
    readFirebaseString(valuePath, tempString);

    if (tempString != "") {
      SerialToArduino.write(0xAA);  // Start byte
      SerialToArduino.write(0x02);  // Tag for command
      SerialToArduino.println("Manual:" + commandName + ":" + tempString);
      Firebase.RTDB.setString(&fbdo, flagPath, "0");  // Reset flag
    }
  }
}
void checkAndSendCommand_autoMode(String flagValue, String flagPath, String valuePath, String commandName) {
  if (flagValue == "1") {
    String tempString;
    readFirebaseString(valuePath, tempString);

    if (tempString != "") {
      SerialToArduino.write(0xAA);  // Start byte
      SerialToArduino.write(0x02);  // Tag for command
      SerialToArduino.println("Auto:" + commandName + ":" + tempString);
      Firebase.RTDB.setString(&fbdo, flagPath, "0");  // Reset flag
    }
  }
}
void SendCMDToArduino() {
  // Manual commands
  checkAndSendCommand_manualMode(lightFlag, CMD_manualPath + "lightFlag", CMD_manualPath + "light", "light");
  checkAndSendCommand_manualMode(musicFlag, CMD_manualPath + "musicFlag", CMD_manualPath + "music", "music");
  checkAndSendCommand_manualMode(swingingFlag, CMD_manualPath + "swingingFlag", CMD_manualPath + "swinging", "swinging");
  //checkAndSendCommand_manualMode(swingingSpeedFlag, CMD_manualPath + "swingingSpeedFlag", CMD_manualPath + "swingingSpeed", "speed");
  //checkAndSendCommand_manualMode(temperatureSetPointFlag, CMD_manualPath + "temperatureSetPointFlag", CMD_manualPath + "temperatureSetPoint", "temperatureSetPoint");
  checkAndSendCommand_manualMode(autoModeFlag, CMD_manualPath + "autoModeFlag", CMD_manualPath + "autoMode", "autoMode");


  // Auto mode specific commands (optional)
  checkAndSendCommand_autoMode(autoSwingingFlag, CMD_autoPath + "swingingFlag", CMD_manualPath + "Swinging", "Swinging");
  checkAndSendCommand_autoMode(autoLightFlag, CMD_autoPath + "LightFlag", CMD_manualPath + "Light", "Light");
  checkAndSendCommand_autoMode(autoMusicFlag, CMD_autoPath + "MusicFlag", CMD_manualPath + "Music", "Music");
  //checkAndSendCommand_autoMode(autoSwingingSpeedFlag, CMD_autoPath+"SwingingSpeedFlag", CMD_manualPath + "SwingingSpeed", "SwingingSpeed");
}


// ===== READ COMMAND FUNCTION =====
String readFirebaseCommand(String path) {
  if (Firebase.RTDB.getString(&fbdo, path)) {
    String value = fbdo.stringData();
    debug("✅ Firebase read success - " + path + ": " + value);
    return value;
  } else {
    debug("❌ Firebase read failed - " + path + ": " + fbdo.errorReason());
    return "";
  }
}

bool offlineMode = false;
// ===== MAIN FUNCTIONS =====
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 Firebase Multi-Sensor Data Logger ===");

  // Initialize Serial connection to Arduino
  SerialToArduino.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // Initialize WiFi - restart if failed
  if (!setupWiFi()) {
    debug("Restarting due to WiFi connection failure...");
    delay(1000);
    //ESP.restart();
    offlineMode = true;
  }

  // Initialize NTP - retry until successful
  bool ntpSuccess = false;
  int ntpRetries = 0;
  while (!ntpSuccess && ntpRetries < maxNtpRetries) {
    ntpSuccess = setupNTP();
    if (!ntpSuccess) {
      ntpRetries++;
      debug("NTP connection failed, retry " + String(ntpRetries) + "/" + String(maxNtpRetries));
      delay(2000);
    }
  }
  if (!ntpSuccess) {
    debug("Restarting due to NTP failure...");
    delay(1000);
    // ESP.restart();
  }

  // Initialize Firebase - restart if failed
  if (!setupFirebase()) {
    debug("Restarting due to Firebase connection failure...");
    delay(1000);
    //ESP.restart();
  }
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));


  // Initialize persistent counters
  initializeCounters();
  debug("Setup complete! Starting data transmission...");
}

void loop() {
  if (!offlineMode) {
    unsigned long currentMillis = millis();

    // Send data at the specified interval
    if (currentMillis - previousMillis >= dataInterval) {
      previousMillis = currentMillis;
      sendDataToFirebase();
      sendStatusToFirebase();
    }

    ReadFlags_SendToArduino();

    delay(10);
  } else {
    Serial.println("Offline mode");
    // offline mode
    delay(10);
  }
}
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <Preferences.h>
#include <time.h>
#include <HardwareSerial.h>
#include <esp_now.h>

#define TXD2 17
#define RXD2 16

HardwareSerial SerialToArduino(2);


//=============================================================== Recieving data from bracelet and send it to arduino ===================================================================


typedef struct __attribute__((packed)) {
  uint16_t bodyTemp;     // Already stored as integer (value * 100)
  uint16_t ambientTemp;  // Already stored as integer (value * 100)
  uint8_t heartRate;
  uint8_t oxygenLevel;
  uint8_t batteryLevel;
  uint8_t batteryVolt;   // Already stored as integer (value * 10)
} SensorData;

SensorData receivedData;// Create a structure to hold the received data

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {// ESP-NOW callback function that will be executed when data is received
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

void sendDataToArduino() {// send Bracelet datat to arduino
  // Send sensor data with tag
  SerialToArduino.write(0xAA); // Start byte
  SerialToArduino.write(0x01); // Tag for sensor data
  SerialToArduino.write((uint8_t*)&receivedData, sizeof(receivedData));
  Serial.println("ESP32: Sent sensor data to Arduino");
}


//----------------------------------------------------------------------------- END : Recieving data from bracelet and send it to arduino -----------------------------------------------------------------------------------------------------


// =========================================================================== Variables ===============================================================================================


// WiFi and Firebase credentials
const char* ssid = "TP-Link-Boost";        //TP-Link_09F8    TP-Link-Boost    Ahmadac  
const char* password = "18230888";  // 18230888    AhmaAd123 
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


// GLOBAL OBJECTS
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
Preferences preferences;


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


//Flags

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


//----------------------------------------------------------------------------- END : Variables -----------------------------------------------------------------------------------------------------


// =========================================================================== Set Up Functions ===============================================================================================


void debug(String message) {// Print debug messages if debug mode is on
  if (debugMode) {
    Serial.println(message);
  }
}


bool setupWiFi() {//CONNECTIVITY FUNCTIONS
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

String getISOTimestamp() {// Get ISO 8601 timestamp for Firebase data
  time_t now = time(nullptr);
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  char timeStringBuff[30];
  strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(timeStringBuff);
}


void initializeCounters() {// Counter FUNCTIONS (data point counter)
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


//----------------------------------------------------------------------------- END : Set Up Functions -----------------------------------------------------------------------------------------------------



// ============================================================================ HOME ================================================================================================


bool offlineMode = false;
unsigned long offlineStartTime = 0;
const unsigned long offlineRestartDelay = 10000; // restart after 10 seconds offline

void setup() {

  Serial.begin(115200);
  Serial.println("\n=== ESP32 Firebase Multi-Sensor Data Logger ===");

  Serial.println("------------------------- ESP32 V5   Testing recieving status from arduino ---------------------------");

  // Initialize Serial connection to Arduino
  SerialToArduino.begin(9600, SERIAL_8N1, RXD2, TXD2);


  // Initialize WiFi - restart if failed
  if (!setupWiFi()) {
    debug("⚠️Restarting due to WiFi connection failure...");
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
      debug("⚠️NTP connection failed, retry " + String(ntpRetries) + "/" + String(maxNtpRetries));
      delay(2000);
    }
  }
  if (!ntpSuccess) {
    debug("⚠️Restarting due to NTP failure...");
    delay(1000);
    // ESP.restart();
  }


  // Initialize Firebase - restart if failed
  if (!setupFirebase()) {
    debug("⚠️Restarting due to Firebase connection failure...");
    delay(1000);
    //ESP.restart();
  }


  if (esp_now_init() != ESP_OK) {
    Serial.println("⚠️Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));


  // Initialize persistent counters
  initializeCounters();


  debug("✅✅✅Setup complete!✅✅✅");
}


void loop() {

  if (!offlineMode) {

    unsigned long currentMillis = millis();
    
    if (currentMillis - previousMillis >= dataInterval) {// Send data at the specified interval
      previousMillis = currentMillis;

      sendDataToFirebase();
  
    }

    Recieve_CMDFlags_FromFirebase();
    ReadFromFirebase_SendtoArduino_CMD();

    Recieve_ArduinoStatus();


  }
  
else { // offline mode
    static bool offlineStartRecorded = false;
    unsigned long currentOfflineMillis = millis();

    if (!offlineStartRecorded) {
      offlineStartTime = currentOfflineMillis;
      offlineStartRecorded = true;
      Serial.println("⚠️ Offline mode activated. System will restart soon...");
    }

    if (currentOfflineMillis - offlineStartTime >= offlineRestartDelay) {
      Serial.println("🔁 Restarting system due to offline mode timeout...");
      delay(100);
      ESP.restart();
    }

    delay(100);
  }
}


//----------------------------------------------------------------------------- END : HOME -----------------------------------------------------------------------------------------------------


// ============================================================================ Recieve & send -> esp <--> Firebase ================================================================================================


void sendDataToFirebase() {// Check connections and send all sensor data to firebase (timed and raw data )

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
  float temperature = receivedData.bodyTemp / 100.0; 
  int humidity = 1.35 * (receivedData.ambientTemp / 100.0, 2);
  bool appearance = temperature<10? false : true ; 
  bool crying = false;


  // Send timed sensor readings
  bool success = true;
  
  if (heartRate>40)success &= sendTimedSensorData("heartRate", heartRate, timestamp);    // success = success (1)  *   1 (if success) or 0 (error)
  if (oxygen>90)success &= sendTimedSensorData("oxygen", oxygen, timestamp);
  if (temperature>30)success &= sendTimedSensorData("temperature", temperature, timestamp);
  if (humidity >20)success &= sendTimedSensorData("humidity", humidity, timestamp);


  // Update raw sensor values with the same readings
  String rawPath = "FirebaseIOT/readings/sensors/raw";
  success &= Firebase.RTDB.setBool(&fbdo, rawPath + "/appearanceState", appearance);
  success &= Firebase.RTDB.setBool(&fbdo, rawPath + "/cryingState", crying);
  if (heartRate>40)success &= Firebase.RTDB.setInt(&fbdo, rawPath+ "/heartRate", heartRate);
  if (oxygen>90) success &= Firebase.RTDB.setInt(&fbdo, rawPath + "/oxygen", oxygen);
  if (humidity >20) success &= Firebase.RTDB.setInt(&fbdo, rawPath + "/humidity", humidity);
  if (temperature>30) success &= Firebase.RTDB.setFloat(&fbdo, rawPath + "/temperature", temperature);



  if (success) {
    debug("✅ All  Timed and raw sensory  data sent successfully with consistent values:");
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
 bool sendTimedSensorData(String sensorType, float value, String timestamp) {// Send a timed data for a single sensor type (heartRate,oxygen,temperature,humidity ) to the firebase return 1 if success and 0 if failed 
  
  // Get the appropriate data point counter
  int dataPoint = 1;  // Default

  // Set dat point name 
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
    debug("❌ Failed to send " + sensorType + "  Timed data: " + fbdo.errorReason());
    return false;
  }
 }


void sendStatusToFirebase(String status_variable, String value) {// sned the crib status information that came from arduino to Firebase 

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


  // Simulated status values — replace with actual ones in your code   ----> Recieve_ArduinoStatus ()
  //bool automode_status = true;
  //bool heatingCooling_status = false;
  //bool light_status = false;
  //bool swinging_status = false;
  //int music_status = 2;
  
  //float temperatureSetPoint_status = 37.5;

  //bool autoModeSettings_light_status = true;
  //bool autoModeSettings_swinging_status = false;
  //int autoModeSettings_music_status = 1;


  // Base path for status
  String basePath = "/FirebaseIOT/readings/status";


  bool success = true;

  if (status_variable == "automode_status")
  success &= Firebase.RTDB.setString(&fbdo, basePath + "/autoMode", value);
  else if (status_variable == "heatingCooling_status")
  success &= Firebase.RTDB.setString(&fbdo, basePath + "/heatingCooling", value);
  else if (status_variable == "light_status")
  success &= Firebase.RTDB.setString(&fbdo, basePath + "/light", value);
  else if (status_variable == "swinging_status")
  success &= Firebase.RTDB.setString(&fbdo, basePath + "/swinging", value);
  else if (status_variable == "music_status")
  success &= Firebase.RTDB.setString(&fbdo, basePath + "/music", value);
  //else if (status_variable == "temperatureSetPoint_status")
  //success &= Firebase.RTDB.setString(&fbdo, basePath + "/temperatureSetPoint", value);

  // Automode settings
  else if (status_variable == "autoModeSettings_light_status")
  success &= Firebase.RTDB.setString(&fbdo, basePath + "/automodeSettings" + "/light", value);
  else if (status_variable == "autoModeSettings_swinging_status")
  success &= Firebase.RTDB.setString(&fbdo,  basePath + "/automodeSettings" + "/swinging", value);
  else if (status_variable == "autoModeSettings_music_status")
  success &= Firebase.RTDB.setString(&fbdo,  basePath + "/automodeSettings" + "/music", value);


  if (success) {
    debug("Status data : "+status_variable+ " ✅Sended succesfully to FB ");
    debug("");
  } else {
    debug("Status data : "+status_variable+ " ❌ failed to send to FB ");
  }
}



void Recieve_CMDFlags_FromFirebase() {// esp read from firebase the flag state (1 or 0 )  1: there is a CMD   0: no CMD

  // Read manual mode flags
  readFirebaseString(CMD_manualPath + "autoModeFlag", autoModeFlag);
  //readFirebaseString(CMD_manualPath + "heatingCoolingFlag", heatingCoolingFlag);
  readFirebaseString(CMD_manualPath + "lightFlag", lightFlag);
  readFirebaseString(CMD_manualPath + "musicFlag", musicFlag);
  readFirebaseString(CMD_manualPath + "swingingFlag", swingingFlag);
  //readFirebaseString(CMD_manualPath + "swingingSpeedFlag", swingingSpeedFlag);
  //readFirebaseString(CMD_manualPath + "temperatureSetPointFlag", temperatureSetPointFlag);
  // Read auto mode flags
  readFirebaseString(CMD_autoPath + "swingingFlag", autoSwingingFlag);
  readFirebaseString(CMD_autoPath + "lightFlag", autoLightFlag);
  readFirebaseString(CMD_autoPath + "musicFlag", autoMusicFlag);
  //readFirebaseString(CMD_autoPath + "swingingSpeedFlag", autoSwingingSpeedFlag);

  // Debug log
  debug("📥 CMD Flags value from Fireabse :");
  debug("Manual -> light: " + lightFlag + ", swing: " + swingingFlag + ", music: " + musicFlag + ", speed: " + swingingSpeedFlag);
  debug("Auto   -> light: " + autoLightFlag + ", swing: " + autoSwingingFlag + ", music: " + autoMusicFlag + ", speed: " + autoSwingingSpeedFlag);  
}
 void readFirebaseString(String path, String& targetVariable) {
  if (Firebase.RTDB.getString(&fbdo, path)) {
    targetVariable = fbdo.stringData();

    /*targetVariable.replace("\"", "");  // Remove quotes
    targetVariable.replace("\\", "");  // Remove backslashes
    targetVariable.trim();             // Remove whitespace or newlines */

    debug("✅ Firebase read success - " + path + ": " + targetVariable);
  } else {
    debug("❌ Firebase read failed - " + path + ": " + fbdo.errorReason());
    targetVariable = "";
  }
 }


//----------------------------------------------------------------------------- END : Recieve & send -> esp <--> Firebase -----------------------------------------------------------------------------------------------------


// ============================================================================ Recieve & send -> esp <--> Arduino ================================================================================================


void ReadFromFirebase_SendtoArduino_CMD() { 
  // Manual commands
  checkAndSendCommand_manualMode(lightFlag, CMD_manualPath + "lightFlag", CMD_manualPath + "light", "light");
  checkAndSendCommand_manualMode(musicFlag, CMD_manualPath + "musicFlag", CMD_manualPath + "music", "music");
  checkAndSendCommand_manualMode(swingingFlag, CMD_manualPath + "swingingFlag", CMD_manualPath + "swinging", "swinging");
  //checkAndSendCommand_manualMode(swingingSpeedFlag, CMD_manualPath + "swingingSpeedFlag", CMD_manualPath + "swingingSpeed", "speed");
  //checkAndSendCommand_manualMode(temperatureSetPointFlag, CMD_manualPath + "temperatureSetPointFlag", CMD_manualPath + "temperatureSetPoint", "temperatureSetPoint");
  checkAndSendCommand_manualMode(autoModeFlag, CMD_manualPath + "autoModeFlag", CMD_manualPath + "autoMode", "autoMode");


  // Auto mode specific commands (optional)
  checkAndSendCommand_autoMode(autoSwingingFlag, CMD_autoPath + "swingingFlag", CMD_autoPath + "swinging", "swinging");
  checkAndSendCommand_autoMode(autoLightFlag, CMD_autoPath + "lightFlag", CMD_autoPath + "light", "light");
  checkAndSendCommand_autoMode(autoMusicFlag, CMD_autoPath + "musicFlag", CMD_autoPath + "music", "music");
  //checkAndSendCommand_autoMode(autoSwingingSpeedFlag, CMD_autoPath+"SwingingSpeedFlag", CMD_manualPath + "SwingingSpeed", "SwingingSpeed");
}
 void checkAndSendCommand_manualMode(String flagValue, String flagPath, String valuePath, String commandName) {
  if (flagValue == "1") {

    String tempString;
    readFirebaseString(valuePath, tempString);

   debug("!!New Manual mode Flag available!! --> Flag : "+ commandName +tempString );

    if (tempString != "") {
      SerialToArduino.write(0xAA);  // Start byte
      SerialToArduino.write(0x02);  // Tag for command
      SerialToArduino.println("Manual " + commandName + " " + tempString);
      Firebase.RTDB.setString(&fbdo, flagPath, "0");  // Reset flag

      debug("✅" +commandName + tempString + " CMD sended succesfully to arduino (Manual Mode)");
    }
  }
 }
 void checkAndSendCommand_autoMode(String flagValue, String flagPath, String valuePath, String commandName) {
  if (flagValue == "1") {

    debug("!!New Auto mode Flag available!! --> Flag : "+ commandName);

    String tempString;
    readFirebaseString(valuePath, tempString);

    if (tempString != "") {
      SerialToArduino.write(0xAA);  // Start byte
      SerialToArduino.write(0x02);  // Tag for command
      SerialToArduino.println("Auto " + commandName + " " + tempString);
      Firebase.RTDB.setString(&fbdo, flagPath, "0");  // Reset flag

      debug("✅" +commandName+ " CMD sended succesfully to arduino (Auto Mode)");

    }
  }
 }


void Recieve_ArduinoStatus() {
  if (SerialToArduino.available()) {
    uint8_t startByte = SerialToArduino.read();
    if (startByte != 0xAA) return;  // Wait for start byte

    // Wait for tag byte
    unsigned long start = millis();
    while (!SerialToArduino.available() && millis() - start < 500) { }
    if (!SerialToArduino.available()) return;  // timeout -> exit safely

    uint8_t tag = SerialToArduino.read();

    if (tag == 0x03) {  // Status tag
      String cmd = SerialToArduino.readStringUntil('\n');
      cmd.trim();   // remove \r, \n, spaces

      Serial.print("Received data: Raw string: [");
      Serial.print(cmd);
      Serial.println("]");

      // --- Split status into Variable and Value ---
      int spaceIndex = cmd.indexOf(' ');
      String Variable = "";
      String Value = "";

      if (spaceIndex != -1) {
        Variable = cmd.substring(0, spaceIndex);
        Value = cmd.substring(spaceIndex + 1);
        Variable.trim();
        Value.trim();
        Value.toLowerCase();  // optional: make it lowercase for comparison
      }

      Serial.print("Variable: ");
      Serial.println(Variable);
      Serial.print("Value: ");
      Serial.println(Value);

      sendStatusToFirebase(Variable,Value );

    }
  }
}

//----------------------------------------------------------------------------- END : Recieve & send -> esp <--> Arduino -----------------------------------------------------------------------------------------------------
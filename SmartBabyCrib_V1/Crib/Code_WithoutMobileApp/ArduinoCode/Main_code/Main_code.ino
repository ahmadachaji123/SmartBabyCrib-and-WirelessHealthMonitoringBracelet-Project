#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include "DFRobotDFPlayerMini.h"

// ====== MOTOR + ENCODER CONFIG ======
#define RPWM 5
#define LPWM 6
#define R_EN 7
#define L_EN 8
#define MOTOR_SPEED 25
#define MIC_PIN A2
#define CLK 4
#define DT 3
#define SW 2
#define RELAY_LIGHT 11
#define DF_TX 10
#define DF_RX 9

#define ESP_RX 9
#define ESP_TX 10

LiquidCrystal_I2C lcd(0x27, 16, 2);
SoftwareSerial dfSerial(DF_RX, DF_TX);
SoftwareSerial espSerial(ESP_RX, ESP_TX);
DFRobotDFPlayerMini player;

uint8_t arrow[8] = { 0x00, 0x04, 0x06, 0x1F, 0x06, 0x04, 0x00 };

const char* menuItems[] = {
  "O2 level", "HR", "Body T", "Ambient T",
  "Humidity: 70%", "Battery", "Crying", "Motor",
  "Music", "Light", "Cooling/Heating", "Auto Mode"
};
const int menuLength = sizeof(menuItems) / sizeof(menuItems[0]);

enum ScreenState { MENU,
                   MOTOR_OPTIONS,
                   LIGHT_OPTIONS,
                   MUSIC_OPTIONS,
                   AUTO_MODE_OPTIONS };
ScreenState currentScreen = MENU;

int soundThreshold = 5;
int cryingMinDuration = 50;
int counter = 0;
int lastClkState;
bool swPressed = false;
unsigned long swPressedTime = 0;
unsigned long lastSensorUpdate = 0;
unsigned long lastCryCheck = 0;
const unsigned long cryCheckInterval = 25000;  // 10 seconds
bool babyCryDetected = false;
const unsigned long sensorTimeout = 3000;  // 3 seconds
bool swHandled = false;
bool detectBabyCry(int soundThreshold = 25, int cryingMinDuration = 50);
bool motorRunning = false;
bool returningToHome = false;
int motorToggle = 0;
float initialAngle = 0;
bool swingRight = true;
bool lightOn = false;
int lightToggle = 0;
int musicSelection = 0;
bool autoModeOn = false;
int autoModeToggle = 0;

// ====== SENSOR DATA STRUCT ======
typedef struct __attribute__((packed)) {
  uint16_t bodyTemp;
  uint16_t ambientTemp;
  uint8_t heartRate;
  uint8_t oxygenLevel;
  uint8_t batteryLevel;
  uint8_t batteryVolt;
} SensorData;

SensorData incomingData;
bool newSensorData = false;

// ====== AS5600 SENSOR ======
class AS5600Sensor {
public:
  const uint8_t address = 0x36;

  bool begin() {
    Wire.begin();
    Wire.setClock(400000L);
    return checkMagnet();
  }

  bool checkMagnet() {
    Wire.beginTransmission(address);
    Wire.write(0x0B);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)address, (uint8_t)1);
    if (Wire.available()) {
      byte status = Wire.read();
      return status & 0x20;
    }
    return false;
  }

  float readAngle() {
    Wire.beginTransmission(address);
    Wire.write(0x0C);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)address, (uint8_t)2);
    if (Wire.available() >= 2) {
      byte highByte = Wire.read();
      byte lowByte = Wire.read();
      int raw = ((highByte & 0x0F) << 8) | lowByte;
      return raw * (360.0 / 4096.0);
    }
    return -1;
  }
};

AS5600Sensor encoder;

// === SONG DATA ===
const int speakerPin = A3;

int melody1[] = { 262, 262, 392, 392, 440, 440, 392 };
int duration1[] = { 500, 500, 500, 500, 500, 500, 1000 };
int melody2[] = { 330, 294, 262, 294, 330, 330, 330 };
int duration2[] = { 400, 400, 400, 400, 400, 400, 800 };
int melody3[] = { 264, 264, 297, 264, 352, 330 };
int duration3[] = { 350, 350, 700, 700, 700, 1400 };

const int NUM_SONGS = 3;
int* melodies[NUM_SONGS] = { melody1, melody2, melody3 };
int* durations[NUM_SONGS] = { duration1, duration2, duration3 };
int melodyLengths[NUM_SONGS] = {
  sizeof(melody1) / sizeof(int),
  sizeof(melody2) / sizeof(int),
  sizeof(melody3) / sizeof(int)
};

int* currentMelody = nullptr;
int* currentDurations = nullptr;
int currentSongLength = 0;
int currentNote = 0;
bool isTonePlaying = false;
bool isPlaying = false;
unsigned long previousMillis = 0;
unsigned long autoModeStartTime = 0;
bool autoModeActive = false;

void setup() {
  pinMode(CLK, INPUT_PULLUP);
  pinMode(DT, INPUT_PULLUP);
  pinMode(SW, INPUT_PULLUP);
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(RELAY_LIGHT, OUTPUT);
  pinMode(speakerPin, OUTPUT);

  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  digitalWrite(RELAY_LIGHT, LOW);

  lcd.init();
  lcd.backlight();
  lcd.createChar(0, arrow);

  Serial.begin(9600);
  dfSerial.begin(9600);
  espSerial.begin(9600);

  if (player.begin(dfSerial)) {
    player.volume(25);
  }

  if (!encoder.begin()) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Encoder Error!");
    while (1)
      ;
  }

  lastClkState = digitalRead(CLK);
  updateMenuDisplay();
}

void loop() {

  handleEncoderRotation();
  handleButtonPress();
  if (motorRunning) handleMotorSwing();
  if (returningToHome) returnToInitialAngle();
  if (isPlaying) updatePlayback();
  receiveSensorData();
  if (millis() - lastCryCheck >= cryCheckInterval) {
    babyCryDetected = detectBabyCry();
    lastCryCheck = millis();
    updateMenuDisplay();
  }
  if (autoModeOn) {
    // If a cry is detected and Auto Mode isn't already running
    if (babyCryDetected && !autoModeActive) {
      autoModeStartTime = millis();
      autoModeActive = true;

      // Start motor
      motorRunning = true;
      initialAngle = encoder.readAngle();
      swingRight = true;
      returningToHome = false;

      // Start music (choose song 1 for now)
      playSong(0);
    }

    // Stop after exactly 1 minute
    if (autoModeActive && millis() - autoModeStartTime >= 60000) {
      stopMotor();
      stopPlayback();
      autoModeActive = false;
      motorRunning = false;
      returningToHome = true;  // Go back to initial angle
    }
  }
}
bool detectBabyCry(int soundThreshold, int cryingMinDuration) {
  static unsigned long soundStart = 0;
  static bool isLoud = false;

  int micValue = analogRead(MIC_PIN);
  if (micValue > soundThreshold) {
    if (!isLoud) {
      isLoud = true;
      soundStart = millis();
    }
  } else {
    if (isLoud) {
      unsigned long duration = millis() - soundStart;
      isLoud = false;
      if (duration >= cryingMinDuration) {
        Serial.println("🍼 Baby cry detected!");
        return true;
      }
    }
  }
  return false;
}


void playSong(int songNumber) {
  if (songNumber >= 0 && songNumber < NUM_SONGS) {
    currentMelody = melodies[songNumber];
    currentDurations = durations[songNumber];
    currentSongLength = melodyLengths[songNumber];
    currentNote = 0;
    isTonePlaying = false;
    isPlaying = true;
  }
}

void updatePlayback() {
  unsigned long currentMillis = millis();

  if (!isTonePlaying && currentNote < currentSongLength) {
    tone(speakerPin, currentMelody[currentNote]);
    previousMillis = currentMillis;
    isTonePlaying = true;
  }

  if (isTonePlaying && (currentMillis - previousMillis >= currentDurations[currentNote])) {
    noTone(speakerPin);
    currentNote++;
    isTonePlaying = false;

    if (currentNote >= currentSongLength) {
      currentNote = 0;  // Restart the song instead of stopping
    }
  }
}

void stopPlayback() {
  noTone(speakerPin);
  isPlaying = false;
}
void handleEncoderRotation() {
  int currentClkState = digitalRead(CLK);
  if (currentClkState != lastClkState) {
    lastClkState = currentClkState;

    if (digitalRead(DT) != currentClkState) {
      if (currentScreen == MENU && counter < menuLength - 1) counter++;
      else if (currentScreen == MUSIC_OPTIONS && musicSelection < 3) musicSelection++;
    } else {
      if (currentScreen == MENU && counter > 0) counter--;
      else if (currentScreen == MUSIC_OPTIONS && musicSelection > 0) musicSelection--;
    }

    if (currentScreen == MENU) updateMenuDisplay();
    else if (currentScreen == MUSIC_OPTIONS) updateMusicSelectionDisplay();
  }
}

void handleButtonPress() {
  if (digitalRead(SW) == LOW) {
    if (!swPressed) {
      swPressed = true;
      swPressedTime = millis();
      swHandled = false;
    } else if (!swHandled && (millis() - swPressedTime > 1200)) {
      swHandled = true;
      currentScreen = MENU;
      updateMenuDisplay();
    }
  } else if (swPressed) {
    swPressed = false;
    if (!swHandled) {
      swHandled = true;

      if (currentScreen == MENU) {
        if (strcmp(menuItems[counter], "Motor") == 0) {
          currentScreen = MOTOR_OPTIONS;
          updateMotorOptionDisplay();
        } else if (strcmp(menuItems[counter], "Auto Mode") == 0) {
          currentScreen = AUTO_MODE_OPTIONS;
          updateAutoModeDisplay();
        } else if (strcmp(menuItems[counter], "Light") == 0) {
          currentScreen = LIGHT_OPTIONS;
          updateLightOptionDisplay();
        } else if (strcmp(menuItems[counter], "Music") == 0) {
          currentScreen = MUSIC_OPTIONS;
          musicSelection = 0;
          updateMusicSelectionDisplay();
        }
      } else if (currentScreen == MOTOR_OPTIONS) {
        motorRunning = (motorToggle == 0);
        motorToggle = 1 - motorToggle;

        if (motorRunning) {
          initialAngle = encoder.readAngle();
          swingRight = true;
          returningToHome = false;
        } else {
          returningToHome = true;
        }

        currentScreen = MENU;
        counter = 7;
        updateMenuDisplay();
      } else if (currentScreen == LIGHT_OPTIONS) {
        lightOn = (lightToggle == 0);
        digitalWrite(RELAY_LIGHT, lightOn ? HIGH : LOW);
        lightToggle = 1 - lightToggle;
        currentScreen = MENU;
        counter = 9;
        updateMenuDisplay();
      } else if (currentScreen == MUSIC_OPTIONS) {
        if (musicSelection < NUM_SONGS) {
          playSong(musicSelection);
        } else {
          stopPlayback();
        }
        currentScreen = MENU;
        counter = 8;
        updateMenuDisplay();
      } else if (currentScreen == AUTO_MODE_OPTIONS) {
        autoModeOn = (autoModeToggle == 0);
        autoModeToggle = 1 - autoModeToggle;

        if (!autoModeOn) {
          // Instantly stop everything if Auto Mode is turned OFF
          stopPlayback();
          stopMotor();
          motorRunning = false;
          returningToHome = true;
          autoModeActive = false;
        }

        currentScreen = MENU;
        counter = 11;
        updateMenuDisplay();
      }
    }
  }
}
void updateAutoModeDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Auto Mode:");
  lcd.setCursor(1, 1);
  lcd.print(autoModeToggle == 0 ? "Turn ON" : "Turn OFF");
}

void updateMenuDisplay() {
  lcd.clear();
  int pageIndex = counter / 2;
  int startIndex = pageIndex * 2;

  for (int i = 0; i < 2; i++) {
    int itemIndex = startIndex + i;
    if (itemIndex >= menuLength) break;

    lcd.setCursor(0, i);
    lcd.print(" ");
    if (itemIndex == counter) lcd.setCursor(0, i), lcd.write(byte(0));
    lcd.setCursor(1, i);
    lcd.print(menuItems[itemIndex]);

    if (itemIndex != 7 && itemIndex != 8 && itemIndex != 9 && itemIndex != 10 && itemIndex != 11) {
      lcd.print(":");
    }
    if (itemIndex == 6) {
      lcd.print(":");
      lcd.print(babyCryDetected ? "yes" : "no");
    } else if (itemIndex == 11) {
      lcd.print(":");
      lcd.print(autoModeOn ? "ON" : "OFF");
    }


    if (newSensorData) {
      switch (itemIndex) {
        case 0:
          if (incomingData.oxygenLevel > 0) {
            lcd.print(incomingData.oxygenLevel);
            lcd.print("%");
          }
          break;
        case 1:
          if (incomingData.heartRate > 0) {
            lcd.print(incomingData.heartRate);
            lcd.print("bpm");
          }
          break;
        case 2:
          if (incomingData.bodyTemp > 0) {
            lcd.print(incomingData.bodyTemp / 100.0, 1);
            lcd.print("C");
          }
          break;
        case 3:
          if (incomingData.ambientTemp > 0) {
            lcd.print(incomingData.ambientTemp / 100.0, 1);
            lcd.print("C");
          }
          break;
        case 5:
          if (incomingData.batteryLevel > 0) {
            lcd.print(incomingData.batteryLevel);
            lcd.print("%");
          }
          break;
      }
    }
  }
}

void updateMotorOptionDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Press to:");
  lcd.setCursor(1, 1);
  lcd.print(motorToggle == 0 ? "Turn ON" : "Turn OFF");
}

void updateLightOptionDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Press to:");
  lcd.setCursor(1, 1);
  lcd.print(lightToggle == 0 ? "Turn ON" : "Turn OFF");
}

void updateMusicSelectionDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Select Track:");
  lcd.setCursor(0, 1);
  lcd.write(byte(0));

  if (musicSelection < NUM_SONGS) {
    lcd.print(" Track ");
    lcd.print(musicSelection + 1);
  } else {
    lcd.print(" Stop Song");
  }
}

void handleMotorSwing() {
  float angle = encoder.readAngle();
  float targetAngle = swingRight ? initialAngle + 3 : initialAngle - 3;

  if (targetAngle > 360) targetAngle -= 360;
  if (targetAngle < 0) targetAngle += 360;

  float diff = angleDiff(angle, targetAngle);

  if (abs(diff) < 1.0) {
    swingRight = !swingRight;
    stopMotor();
    delay(200);
  } else {
    if (diff > 0) {
      analogWrite(RPWM, MOTOR_SPEED);
      analogWrite(LPWM, 0);
    } else {
      analogWrite(RPWM, 0);
      analogWrite(LPWM, MOTOR_SPEED);
    }
  }
}

void returnToInitialAngle() {
  float currentAngle = encoder.readAngle();
  float error = angleDiff(currentAngle, initialAngle);

  if (abs(error) < 1.0) {
    stopMotor();
    returningToHome = false;
  } else {
    if (error > 0) {
      analogWrite(RPWM, MOTOR_SPEED);
      analogWrite(LPWM, 0);
    } else {
      analogWrite(RPWM, 0);
      analogWrite(LPWM, MOTOR_SPEED);
    }
  }
}

float angleDiff(float from, float to) {
  float diff = fmod((to - from + 540), 360) - 180;
  return diff;
}

void stopMotor() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}

void receiveSensorData() {
  if (espSerial.available()) {
    if (espSerial.read() != 0xAA) return;

    while (!espSerial.available())
      ;
    uint8_t tag = espSerial.read();

    if (tag == 0x01) {
      while (espSerial.available() < sizeof(incomingData))
        ;
      espSerial.readBytes((char*)&incomingData, sizeof(incomingData));
      newSensorData = true;
      updateMenuDisplay();
    }
  }
}

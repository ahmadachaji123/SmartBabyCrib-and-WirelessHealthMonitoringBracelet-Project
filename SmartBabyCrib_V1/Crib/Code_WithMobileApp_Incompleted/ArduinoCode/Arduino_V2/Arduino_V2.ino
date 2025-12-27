#include <SoftwareSerial.h>
#include "DFRobotDFPlayerMini.h"
#include <Wire.h>

#define light 11

SoftwareSerial espSerial(9, 10);  // RX, TX

typedef struct __attribute__((packed)) {
  uint16_t bodyTemp;     // Already stored as integer (value * 100)
  uint16_t ambientTemp;  // Already stored as integer (value * 100)
  uint8_t heartRate;
  uint8_t oxygenLevel;
  uint8_t batteryLevel;
  uint8_t batteryVolt;  // Already stored as integer (value * 10)
} SensorData;

SensorData incomingData;
int setLightMode = -1;


const int speakerPin = A3;
const int NUM_SONGS = 3;  // Only 3 songs now
// SONG 0 - "Twinkle Twinkle Little Star"
int melody1[] = {
  262, 262, 392, 392, 440, 440, 392,
  349, 349, 330, 330, 294, 294, 262,
  392, 392, 349, 349, 330, 330, 294,
  392, 392, 349, 349, 330, 330, 294,
  262, 262, 392, 392, 440, 440, 392,
  349, 349, 330, 330, 294, 294, 262,
  294, 294, 262, 262, 294, 294, 330,
  330, 349, 330, 294, 262, 294, 330,
  392, 392, 349, 330, 294, 262, 262
};
int duration1[] = {
  500, 500, 500, 500, 500, 500, 1000,
  500, 500, 500, 500, 500, 500, 1000,
  500, 500, 500, 500, 500, 500, 1000,
  500, 500, 500, 500, 500, 500, 1000,
  500, 500, 500, 500, 500, 500, 1000,
  500, 500, 500, 500, 500, 500, 1000,
  500, 500, 500, 500, 500, 500, 700,
  500, 500, 500, 500, 500, 500, 1000,
  500, 500, 500, 500, 500, 800, 1000
};
// SONG 1 - "Mary Had a Little Lamb"
int melody2[] = {
  330, 294, 262, 294, 330, 330, 330, 294, 294, 294, 330, 392, 392,
  330, 330, 330, 294, 262, 262, 294, 294, 330, 330, 330, 294, 330,
  330, 294, 262, 262, 330, 330, 330, 294, 294, 294, 330, 392, 392,
  330, 330, 330, 294, 262, 262
};
int duration2[] = {
  400, 400, 400, 400, 400, 400, 800, 400, 400, 800, 400, 400, 800,
  400, 400, 400, 400, 400, 400, 800, 400, 400, 400, 400, 400, 400,
  800, 400, 400, 400, 400, 400, 400, 800, 400, 400, 400, 400, 400,
  400, 800, 400, 400, 400, 400, 400, 400
};
// SONG 2 - "Happy Birthday"
int melody3[] = {
  264, 264, 297, 264, 352, 330, 264, 264, 297, 264, 396, 352,
  264, 264, 297, 264, 352, 330, 264, 264, 297, 264, 396, 352,
  264, 264, 297, 264, 352, 330, 264, 264, 297, 264, 396, 352,
  264, 264, 297, 264, 352, 330
};
int duration3[] = {
  350, 350, 700, 700, 700, 1400, 350, 350, 700, 700, 700, 1400,
  350, 350, 700, 700, 700, 1400, 350, 350, 700, 700, 700, 1400,
  350, 350, 700, 700, 700, 1400, 350, 350, 700, 700, 700, 1400,
  350, 350, 700, 700, 700, 1400
};
// Arrays for all songs
int* melodies[NUM_SONGS] = { melody1, melody2, melody3 };
int* durations[NUM_SONGS] = { duration1, duration2, duration3 };
int melodyLengths[NUM_SONGS] = {
  sizeof(melody1) / sizeof(int), sizeof(melody2) / sizeof(int), sizeof(melody3) / sizeof(int)
};
// Playback state
int* currentMelody;
int* currentDurations;
int currentSongLength;
int currentNote = 0;
bool isTonePlaying = false;
bool isPlaying = false;
unsigned long previousMillis = 0;


// Define the BTS7960 control pins
const int RPWM = 5;  // Right PWM pin connected to Arduino pin 3
const int LPWM = 6;  // Left PWM pin connected to Arduino pin 5
const int R_EN = 7;  // Right Enable pin connected to Arduino pin 2
const int L_EN = 8;  // Left Enable pin connected to Arduino pin 4

void setup() {
  Serial.begin(115200);
  espSerial.begin(9600);
  pinMode(light, OUTPUT);
  digitalWrite(light, LOW);

  pinMode(speakerPin, OUTPUT);
  

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  // Enable the motor driver
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);



}

void loop() {
  if (espSerial.available()) {
    if (espSerial.read() != 0xAA) return;  // Wait for start byte

    while (!espSerial.available())
      ;
    uint8_t tag = espSerial.read();

    if (tag == 0x01) {
      while (espSerial.available() < sizeof(incomingData))
        ;
      espSerial.readBytes((char*)&incomingData, sizeof(incomingData));

      Serial.println("Arduino: Got sensor struct");
      Serial.print("Body Temp: ");
      Serial.println(incomingData.bodyTemp / 100.0, 2);
      Serial.print("Ambient Temp: ");
      Serial.println(incomingData.ambientTemp / 100.0, 2);
      Serial.print("Heart Rate: ");
      Serial.println(incomingData.heartRate);
      Serial.print("Oxygen Level: ");
      Serial.println(incomingData.oxygenLevel);
      Serial.print("Battery Level: ");
      Serial.println(incomingData.batteryLevel);
      Serial.print("Battery Voltage: ");
      Serial.println(incomingData.batteryVolt / 10.0, 1);
      Serial.println("----------------------");

    } else if (tag == 0x02) {
      String cmd = espSerial.readStringUntil('\n');

      Serial.print("Arduino: Raw cmd string: [");
      Serial.print(cmd);
      Serial.println("]");

      int modeIndex = cmd.indexOf(':');
      int typeIndex = cmd.indexOf(':', modeIndex + 1);

      if (modeIndex != -1 && typeIndex != -1) {
        String mode = cmd.substring(0, modeIndex);
        String type = cmd.substring(modeIndex + 1, typeIndex);
        String action = cmd.substring(typeIndex + 1);
        action.replace("\"", "");  // Remove quotes
        action.replace("\\", "");  // Remove backslashes
        action.trim();             // Remove whitespace or newlines

        action.trim();         // Remove whitespace
        action.toLowerCase();  // Normalize

        Serial.print("Mode: ");
        Serial.println(mode);
        Serial.print("Type: ");
        Serial.println(type);
        Serial.print("Action: ");
        Serial.println(action);

        if (mode == "Manual") {
          if (type == "light") {
            if (action == "on") {
              setLightMode = 1;
              Serial.println(" Turning ON the light.");
            } else if (action == "off") {
              setLightMode = 0;
              Serial.println(" Turning OFF the light.");
            }
          } else if (type == "music") {
            if (action == "0") {
              playSong(0);
              stopPlayback();
              Serial.println(" Turning off the music.");
            } else if (action == "1") {
              playSong(1);
              Serial.println(" Turning ON the music 1.");
            } else if (action == "2") {
              playSong(2);
              Serial.println(" Turning ON the music 2");
            } else if (action == "3") {
              playSong(3);
              Serial.println("  Turning ON the music 3");
            }
          } else if (type == "swinging") {
            if (action == "on") {
             
              Serial.println(" swinging is set on .");
            } else if (action == "off") {
              analogWrite(RPWM, 0);
              analogWrite(LPWM, 0);
              Serial.println(" swinging is set off");
            }
          } else if (type == "autoMode") {
            if (action == "on") {
              Serial.println(" automode is set on .");
            } else if (action == "off") {
              Serial.println(" automode is set off");
            }
          }
        } else if (mode == "Auto") {
          if (type == "light") {
            if (action == "on") {
              digitalWrite(light, HIGH);
              Serial.println(" automode_set ON the light.");
            } else if (action == "off") {
              digitalWrite(light, LOW);
              Serial.println(" automode_Turning OFF the light.");
            }
          } else if (type == "music") {
            if (action == "0") {
              Serial.println("automode_ Turning off the music.");
            } else if (action == "1") {
              Serial.println(" automode_Turning ON the music 1.");
            } else if (action == "2") {
              Serial.println(" automode_Turning ON the music 2");
            } else if (action == "3") {
              Serial.println(" automode_ Turning ON the music 3");
            }
          } else if (type == "swinging") {
            if (action == "on") {
              Serial.println(" automode_swinging is set on .");
            } else if (action == "off") {
              Serial.println("automode_ swinging is set off");
            }
          }
        }
      }
    }
  }
  updatePlayback();
}
// --- FUNCTIONS ---

void playSong(int songNumber) {
  songNumber -= 1; // Adjust so song 1 refers to index 0, song 2 to index 1, etc.

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
  if (!isPlaying) return;

  unsigned long currentMillis = millis();

  if (!isTonePlaying) {
    tone(speakerPin, currentMelody[currentNote]);
    previousMillis = currentMillis;
    isTonePlaying = true;
  }

  if (currentMillis - previousMillis >= currentDurations[currentNote]) {
    noTone(speakerPin);
    currentNote++;
    isTonePlaying = false;

    if (currentNote >= currentSongLength) {
      stopPlayback();
    }
  }
}

void stopPlayback() {
  noTone(speakerPin);
  isPlaying = false;
}


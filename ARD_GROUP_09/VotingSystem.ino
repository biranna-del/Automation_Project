#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Adafruit_Fingerprint.h>

// Reduce the IR receive buffer; sufficient for NEC.
#define RAW_BUFFER_LENGTH 100
#define DECODE_NEC
#include <IRremote.hpp>

// ---------- Pins ----------
const byte LED_PIN = 4;
const byte BUTTON_PIN = 5;
const byte TRIG_PIN = 7;
const byte IR_PIN = 8;
const byte ECHO_PIN = 9;
const byte SERVO_PIN = 10;

// Sensor TX -> D11
// Sensor RX -> D12
SoftwareSerial fingerSerial(11, 12);
Adafruit_Fingerprint finger(&fingerSerial);

hd44780_I2Cexp lcd;
Servo gateServo;

bool lcdReady = false;
bool fingerprintReady = false;

// ---------- Servo settings ----------
const uint16_t GATE_CLOSED_US = 1000;
const uint16_t GATE_OPEN_US = 1500;

// ---------- Remote ----------
const uint16_t CLOSE_CODE = 0x46;
uint16_t okCode = 0;
bool okLearned = false;

// ---------- States ----------
enum {
  CLOSED,
  ARMING,
  WAITING,
  OPENING,
  VERIFY_FINGER,
  READY,
  ACCEPTED,
  CLOSING
};

byte state = CLOSED;

uint16_t voteCount = 0;
uint16_t matchedFingerID = 0;

// ---------- Distance ----------
float distanceCm = 0;
bool echoReceived = false;
bool distanceValid = false;
bool freshDistance = false;

unsigned long lastDistanceTime = 0;

bool clearTiming = false;
unsigned long clearStarted = 0;
byte nearSamples = 0;

// ---------- Timers ----------
unsigned long gateTime = 0;
unsigned long voteTime = 0;

const unsigned long GATE_MOVE_MS = 600;
const unsigned long CLOSE_AFTER_VOTE_MS = 10000;
const unsigned long CLEAR_TIME_MS = 1000;

// ---------- Fingerprint polling ----------
unsigned long lastFingerAttempt = 0;
unsigned long fingerInterval = 400;

// ---------- Button debounce ----------
int lastButtonReading = HIGH;
int stableButtonState = HIGH;
unsigned long buttonChangeTime = 0;

// ---------- LCD: strings stay in Flash ----------
void showMessage(
  const __FlashStringHelper* line1,
  const __FlashStringHelper* line2
) {
  if (!lcdReady) return;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

void changeState(byte newState) {
  state = newState;

  switch (state) {
    case CLOSED:
      showMessage(
        F("Voting Closed"),
        F("Press OK to open")
      );
      Serial.println(F("Voting closed"));
      break;

    case ARMING:
      matchedFingerID = 0;
      clearTiming = false;
      nearSamples = 0;

      showMessage(
        F("Voting Open"),
        F("Clear entrance")
      );
      Serial.println(F("Waiting for clear entrance"));
      break;

    case WAITING:
      nearSamples = 0;

      showMessage(
        F("Voting Open"),
        F("Waiting voter")
      );
      Serial.println(F("Ready for next voter"));
      break;

    case OPENING:
      showMessage(
        F("Voter Arrived"),
        F("Opening gate...")
      );
      Serial.println(F("Opening gate"));
      break;

    case VERIFY_FINGER:
      matchedFingerID = 0;
      fingerInterval = 400;
      lastFingerAttempt = millis() - fingerInterval;

      showMessage(
        F("Waiting for ID"),
        F("Place finger")
      );
      Serial.println(F("Waiting for ID"));
      break;

    case READY:
      if (lcdReady) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("ID verified: "));
        lcd.print(matchedFingerID);
        lcd.setCursor(0, 1);
        lcd.print(F("Press & vote"));
      }

      // A button held before identification cannot vote.
      lastButtonReading = digitalRead(BUTTON_PIN);
      stableButtonState = lastButtonReading;
      buttonChangeTime = millis();

      Serial.print(F("Verified ID: "));
      Serial.println(matchedFingerID);
      break;

    case ACCEPTED:
      showMessage(
        F("Vote Accepted"),
        F("Closing in 10s")
      );

      Serial.print(F("Total votes: "));
      Serial.println(voteCount);
      break;

    case CLOSING:
      showMessage(
        F("Closing gate..."),
        F("Vote completed")
      );
      Serial.println(F("Closing gate"));
      break;
  }
}

// ---------- Manager closes from any state ----------
void closeImmediately() {
  gateServo.writeMicroseconds(GATE_CLOSED_US);
  digitalWrite(LED_PIN, LOW);

  clearTiming = false;
  nearSamples = 0;
  gateTime = 0;
  voteTime = 0;
  matchedFingerID = 0;

  changeState(CLOSED);

  Serial.print(F("Manager closed. Total: "));
  Serial.println(voteCount);
}

// ---------- Remote ----------
void checkRemote() {
  if (!IrReceiver.decode()) return;

  uint16_t command = IrReceiver.decodedIRData.command;

  bool valid =
    IrReceiver.decodedIRData.protocol == NEC &&
    !(IrReceiver.decodedIRData.flags &
      IRDATA_FLAGS_IS_REPEAT);

  IrReceiver.resume();

  if (!valid) return;

  if (command == CLOSE_CODE) {
    closeImmediately();
    return;
  }

  // Press OK first after startup.
  if (!okLearned && state == CLOSED) {
    okCode = command;
    okLearned = true;

    Serial.print(F("OK learned: 0x"));
    Serial.println(okCode, HEX);
  }

  if (okLearned &&
      command == okCode &&
      state == CLOSED) {
    changeState(ARMING);
  }
}

// ---------- Distance measurement ----------
void readDistance() {
  if (millis() - lastDistanceTime < 100UL) return;

  lastDistanceTime = millis();

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration =
    pulseIn(ECHO_PIN, HIGH, 25000UL);

  freshDistance = true;
  echoReceived = duration != 0;

  if (!echoReceived) {
    distanceValid = false;
    return;
  }

  distanceCm = duration / 58.0;
  distanceValid =
    distanceCm >= 2.0 && distanceCm <= 400.0;
}

// ---------- Fingerprint diagnostic: Serial only ----------
void reportFingerprintError(uint8_t code) {
  Serial.print(F("Fingerprint error: 0x"));
  Serial.println(code, HEX);

  if (code == FINGERPRINT_PACKETRECIEVEERR) {
    fingerprintReady = false;
  }

  fingerInterval = 1500;
  lastFingerAttempt = millis();
}

// ---------- Verify an existing fingerprint ----------
void checkFingerprint() {
  if (state != VERIFY_FINGER) return;

  if (millis() - lastFingerAttempt < fingerInterval) {
    return;
  }

  lastFingerAttempt = millis();
  fingerInterval = 400;

  if (!fingerprintReady) {
    fingerprintReady = finger.verifyPassword();

    checkRemote();
    if (state != VERIFY_FINGER) return;

    if (!fingerprintReady) {
      Serial.println(F("Fingerprint not connected"));

      fingerInterval = 2000;
      lastFingerAttempt = millis();
      return;
    }
  }

  uint8_t result = finger.getImage();

  checkRemote();
  if (state != VERIFY_FINGER) return;

  if (result == FINGERPRINT_NOFINGER) return;

  if (result != FINGERPRINT_OK) {
    reportFingerprintError(result);
    return;
  }

  result = finger.image2Tz(1);

  checkRemote();
  if (state != VERIFY_FINGER) return;

  if (result != FINGERPRINT_OK) {
    reportFingerprintError(result);
    return;
  }

  result = finger.fingerFastSearch();

  checkRemote();
  if (state != VERIFY_FINGER) return;

  if (result == FINGERPRINT_NOTFOUND) {
    Serial.println(F("No saved fingerprint match"));

    fingerInterval = 1500;
    lastFingerAttempt = millis();
    return;
  }

  if (result != FINGERPRINT_OK) {
    reportFingerprintError(result);
    return;
  }

  matchedFingerID = finger.fingerID;
  changeState(READY);
}

// ---------- Voting button ----------
void checkButton() {
  int reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonReading) {
    lastButtonReading = reading;
    buttonChangeTime = millis();
  }

  if (millis() - buttonChangeTime >= 50UL &&
      reading != stableButtonState) {

    stableButtonState = reading;

    if (stableButtonState == LOW && state == READY) {
      voteCount++;
      voteTime = millis();
      changeState(ACCEPTED);
    }
  }
}

// ---------- Voting cycle ----------
void updateVoting() {
  unsigned long now = millis();

  switch (state) {
    case ARMING:
      if (freshDistance) {
        // No echo can occur when the entrance is empty.
        // It only arms detection; it never opens the gate.
        bool entranceClear =
          !echoReceived ||
          (distanceValid && distanceCm >= 60.0);

        if (entranceClear) {
          if (!clearTiming) {
            clearTiming = true;
            clearStarted = now;
          }

          if (now - clearStarted >= CLEAR_TIME_MS) {
            changeState(WAITING);
          }
        } else {
          clearTiming = false;
        }
      }
      break;

    case WAITING:
      if (freshDistance) {
        bool someoneNear =
          distanceValid && distanceCm <= 40.0;

        if (someoneNear) {
          nearSamples++;

          // Require 3 consecutive near measurements.
          if (nearSamples >= 3) {
            gateServo.writeMicroseconds(GATE_OPEN_US);
            gateTime = now;
            changeState(OPENING);
          }
        } else {
          nearSamples = 0;
        }
      }
      break;

    case OPENING:
      if (now - gateTime >= GATE_MOVE_MS) {
        changeState(VERIFY_FINGER);
      }
      break;

    case ACCEPTED:
      // Close independently of the distance sensor.
      if (now - voteTime >= CLOSE_AFTER_VOTE_MS) {
        gateServo.writeMicroseconds(GATE_CLOSED_US);
        gateTime = now;
        changeState(CLOSING);
      }
      break;

    case CLOSING:
      if (now - gateTime >= GATE_MOVE_MS) {
        // Return automatically to the next voting cycle.
        changeState(ARMING);
      }
      break;

    default:
      break;
  }

  freshDistance = false;
}

// ---------- Setup ----------
void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(TRIG_PIN, LOW);

  Serial.begin(9600);

  // At startup, send ONLY the closed-gate command.
  gateServo.attach(SERVO_PIN);
  gateServo.writeMicroseconds(GATE_CLOSED_US);

  lcdReady = lcd.begin(16, 2) == 0;

  if (lcdReady) {
    lcd.backlight();
  } else {
    Serial.println(F("LCD initialization failed"));
  }

  finger.begin(57600);

  IrReceiver.begin(IR_PIN, DISABLE_LED_FEEDBACK);

  lastButtonReading = digitalRead(BUTTON_PIN);
  stableButtonState = lastButtonReading;

  changeState(CLOSED);
  Serial.println(F("Press OK first"));
}

// ---------- Main loop ----------
void loop() {
  checkRemote();
  readDistance();
  updateVoting();
  checkFingerprint();
  checkButton();

  bool nearVoter =
    state != CLOSED &&
    distanceValid &&
    distanceCm <= 40.0;

  digitalWrite(LED_PIN, nearVoter ? HIGH : LOW);
}

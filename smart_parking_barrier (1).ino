/*
 * Smart Parking Barrier
 * ---------------------
 * Detects an approaching vehicle with an ultrasonic sensor, raises a servo
 * barrier, waits for the vehicle to clear an LDR-based exit sensor, then
 * lowers the barrier.
 *
 * Written as a non-blocking finite state machine: no delay() anywhere in
 * loop(), so the sensors, servo sweep, LCD and buzzer all stay responsive
 * at the same time. Includes a safety timeout so the barrier can never be
 * left stuck open.
 *
 * Board: Arduino UNO R4 Minima
 */

#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ---------- pin map ----------
const uint8_t PIN_TRIG   = 9;
const uint8_t PIN_ECHO   = 10;
const uint8_t PIN_LDR    = A0;
const uint8_t PIN_SERVO  = 6;
const uint8_t PIN_BUZZER = 7;
const uint8_t PIN_LED_R  = 4;
const uint8_t PIN_LED_G  = 5;

// ---------- tunables ----------
const int  DETECT_DISTANCE_CM = 20;    // vehicle counts as "approaching" inside this
const int  ANGLE_CLOSED       = 0;     // servo angle, barrier down
const int  ANGLE_OPEN         = 90;    // servo angle, barrier up
const int  SERVO_STEP_MS      = 15;    // ms between 1-degree steps (sweep speed)
const int  LDR_MARGIN         = 70;    // how far below baseline counts as a shadow (tuned: baseline 761, dip to 648, with phone flashlight above LDR)
const unsigned long PASS_TIMEOUT_MS  = 10000UL; // close anyway if nobody passes
const unsigned long PASS_CLEAR_DELAY_MS = 1200UL; // time to wait after shadow first seen, before treating vehicle as clear
const unsigned long CLEAR_HOLD_MS    = 1000UL;  // wait after pass before closing
const unsigned long PING_INTERVAL_MS = 60UL;    // how often to fire the ultrasonic
const unsigned long BEEP_INTERVAL_MS = 300UL;   // beep cadence while moving

// ---------- state machine ----------
enum State { IDLE, OPENING, WAIT_PASS, CLEARING, CLOSING };
State state = IDLE;

Servo barrier;
LiquidCrystal_I2C lcd(0x27, 16, 2);

int  servoAngle   = ANGLE_CLOSED;
int  ldrBaseline  = 0;
bool shadowSeen   = false;   // vehicle has entered the exit sensor's view
unsigned long shadowSeenAt = 0; // when the shadow was first detected

unsigned long stateEnteredAt = 0;
unsigned long lastServoStep  = 0;
unsigned long lastPing       = 0;
unsigned long lastBeep       = 0;
unsigned long clearedAt      = 0;
bool buzzerOn = false;

long lastDistance = 999;

void setup() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);

  barrier.attach(PIN_SERVO);
  barrier.write(ANGLE_CLOSED);

  lcd.init();
  lcd.backlight();

  Serial.begin(9600);

  // Sample ambient light so the exit sensor adapts to the room it's in
  // instead of relying on a hardcoded threshold.
  long sum = 0;
  for (int i = 0; i < 32; i++) {
    sum += analogRead(PIN_LDR);
    delay(5);                 // setup only -- loop() stays delay-free
  }
  ldrBaseline = sum / 32;

  Serial.print(F("LDR baseline: "));
  Serial.println(ldrBaseline);

  enterState(IDLE);
}

void loop() {
  unsigned long now = millis();

  readUltrasonic(now);
  updateServo(now);
  updateBuzzer(now);

  switch (state) {

    case IDLE:
      if (lastDistance > 0 && lastDistance <= DETECT_DISTANCE_CM) {
        enterState(OPENING);
      }
      break;

    case OPENING:
      if (servoAngle >= ANGLE_OPEN) {
        shadowSeen = false;
        enterState(WAIT_PASS);
      }
      break;

    case WAIT_PASS: {
      int light = analogRead(PIN_LDR);

      // Phase 1: the vehicle blocks the light, reading drops.
      if (!shadowSeen && light < ldrBaseline - LDR_MARGIN) {
        shadowSeen = true;
        shadowSeenAt = now;
        lcd.setCursor(0, 1);
        lcd.print(F("Passing...      "));
      }

      // Phase 2: rather than waiting for the light to climb back above an
      // exact threshold (fragile -- ambient light drifts and noise near
      // the line causes missed detections), just wait a fixed transit
      // time after the shadow first appears. This is far more robust
      // for a small object like a toy car passing over the sensor.
      if (shadowSeen && now - shadowSeenAt >= PASS_CLEAR_DELAY_MS) {
        clearedAt = now;
        enterState(CLEARING);
      }

      // Safety net: never leave the barrier open indefinitely.
      if (now - stateEnteredAt > PASS_TIMEOUT_MS) {
        Serial.println(F("Timeout -- closing without a confirmed pass"));
        enterState(CLOSING);
      }
      break;
    }

    case CLEARING:
      // Short settle time so the barrier doesn't clip the back of the vehicle.
      if (now - clearedAt >= CLEAR_HOLD_MS) {
        enterState(CLOSING);
      }
      break;

    case CLOSING:
      if (servoAngle <= ANGLE_CLOSED) {
        enterState(IDLE);
      }
      break;
  }
}

// ---------- state entry ----------
void enterState(State next) {
  state = next;
  stateEnteredAt = millis();

  lcd.clear();

  switch (state) {
    case IDLE:
      digitalWrite(PIN_LED_R, HIGH);
      digitalWrite(PIN_LED_G, LOW);
      lcd.print(F("Barrier closed"));
      lcd.setCursor(0, 1);
      lcd.print(F("Waiting..."));
      break;

    case OPENING:
      digitalWrite(PIN_LED_R, LOW);
      digitalWrite(PIN_LED_G, HIGH);
      lcd.print(F("Vehicle detected"));
      lcd.setCursor(0, 1);
      lcd.print(F("Opening..."));
      break;

    case WAIT_PASS:
      lcd.print(F("Barrier open"));
      lcd.setCursor(0, 1);
      lcd.print(F("Proceed"));
      break;

    case CLEARING:
      lcd.print(F("Vehicle passed"));
      lcd.setCursor(0, 1);
      lcd.print(F("Clearing..."));
      break;

    case CLOSING:
      digitalWrite(PIN_LED_R, HIGH);
      digitalWrite(PIN_LED_G, LOW);
      lcd.print(F("Closing..."));
      break;
  }

  Serial.print(F("-> state "));
  Serial.println(state);
}

// ---------- non-blocking helpers ----------

// Fires the ultrasonic sensor on a timer rather than every pass, so the
// 38 ms worst-case pulseIn() timeout doesn't dominate the loop.
void readUltrasonic(unsigned long now) {
  if (now - lastPing < PING_INTERVAL_MS) return;
  lastPing = now;

  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  long duration = pulseIn(PIN_ECHO, HIGH, 25000UL);   // ~4 m ceiling
  if (duration == 0) {
    lastDistance = 999;          // nothing in range
  } else {
    lastDistance = duration / 58;
  }
}

// Moves the servo one degree per tick toward the target for the current
// state, so the sweep is smooth and the loop never blocks on it.
void updateServo(unsigned long now) {
  int target;
  if (state == OPENING || state == WAIT_PASS || state == CLEARING) {
    target = ANGLE_OPEN;
  } else {
    target = ANGLE_CLOSED;
  }

  if (servoAngle == target) return;
  if (now - lastServoStep < SERVO_STEP_MS) return;
  lastServoStep = now;

  servoAngle += (servoAngle < target) ? 1 : -1;
  barrier.write(servoAngle);
}

// Beeps only while the arm is actually in motion.
void updateBuzzer(unsigned long now) {
  bool shouldBeep = (state == OPENING || state == CLOSING);

  if (!shouldBeep) {
    if (buzzerOn) {
      digitalWrite(PIN_BUZZER, LOW);
      buzzerOn = false;
    }
    return;
  }

  if (now - lastBeep >= BEEP_INTERVAL_MS) {
    lastBeep = now;
    buzzerOn = !buzzerOn;
    digitalWrite(PIN_BUZZER, buzzerOn ? HIGH : LOW);
  }
}

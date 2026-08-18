#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ── OLED ─────────────────────────────
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ── Motor Pins ───────────────────────
#define ENA 3
#define IN1 2
#define IN2 4

#define ENB 11
#define IN3 6
#define IN4 7

#define STBY 13

// ── Buttons ──────────────────────────
#define BTN_START 10
#define BTN_STOP  12

// ── Sensors ──────────────────────────
const byte sensorPin[8] = {A7, A6, A5, A4, A3, A2, A1, A0};

// ── PID CONFIG ───────────────────────
float Kp = 0.16;
float Kd = 0.34;

float lastError = 0;

// ── Speed ───────────────────────────
int BASE_SPEED = 160;
int MAX_SPEED  = 255;
int MOTOR_MIN  = 80;
int SEARCH_SPEED = 130;

// ── Junction / branch handling ──────
int JUNCTION_TURN_SPEED = 150;
int JUNCTION_HOLD_MS    = 220;

// ── Calibration ─────────────────────
int sensorMin[8], sensorMax[8], sensorThresh[8];
char sensorStr[9];

// ── State ───────────────────────────
bool running = false;

// ── Lost-line memory ────────────────
int lastDir = 1;

// ── Stop bar ────────────────────────
bool allBlackForward = false;

// ═════════ MOTOR ═════════

int applyDeadzone(int spd) {
  if (spd == 0) return 0;
  return constrain(spd, MOTOR_MIN, MAX_SPEED);
}

void motorLeft(int spd, bool fwd) {
  digitalWrite(IN1, fwd);
  digitalWrite(IN2, !fwd);
  analogWrite(ENA, applyDeadzone(spd));
}

void motorRight(int spd, bool fwd) {
  digitalWrite(IN3, fwd);
  digitalWrite(IN4, !fwd);
  analogWrite(ENB, applyDeadzone(spd));
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ═════════ SENSOR ═════════

void readSensors() {
  for (int i = 0; i < 8; i++) {
    sensorStr[i] = (analogRead(sensorPin[i]) > sensorThresh[i]) ? '1' : '0';
  }
  sensorStr[8] = '\0';
}

float getPosition() {
  int sum = 0, count = 0;
  for (int i = 0; i < 8; i++) {
    if (sensorStr[i] == '1') {
      sum += i * 1000;
      count++;
    }
  }
  if (count == 0) return -1;
  return (float)sum / count;
}

bool allBlack() {
  for (int i = 0; i < 8; i++) {
    if (sensorStr[i] == '0') return false;
  }
  return true;
}

// ═════════ IMPROVED DETECTION ═════════

// ✅ Real junction (wide line, not zigzag edge)
bool isJunction() {
  int count = 0;
  for (int i = 0; i < 8; i++) {
    if (sensorStr[i] == '1') count++;
  }
  return (count >= 5);
}

// ✅ strong center detection
bool centerActive() {
  return (sensorStr[3] == '1' && sensorStr[4] == '1');
}

// ✅ clean branch detection (ignore noise)
int sideBranch() {
  int leftCount = 0, rightCount = 0;

  for (int i = 0; i <= 2; i++)
    if (sensorStr[i] == '1') leftCount++;

  for (int i = 5; i <= 7; i++)
    if (sensorStr[i] == '1') rightCount++;

  if (rightCount >= 2) return 1;
  if (leftCount >= 2) return -1;

  return 0;
}

// ═════════ CALIBRATION ═════════

void calibrate() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("WHITE...");
  display.display();
  delay(2000);

  for (int i = 0; i < 8; i++)
    sensorMin[i] = analogRead(sensorPin[i]);

  display.clearDisplay();
  display.setCursor(0,0);
  display.println("BLACK...");
  display.display();
  delay(2000);

  for (int i = 0; i < 8; i++)
    sensorMax[i] = analogRead(sensorPin[i]);

  for (int i = 0; i < 8; i++)
    sensorThresh[i] = (sensorMin[i] + sensorMax[i]) / 2;
}

// ═════════ SETUP ═════════

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);

  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    // OLED not found — halt here so the issue is obvious
    for (;;);
  }

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Calibrating...");
  display.display();

  calibrate();

  display.clearDisplay();
  display.setCursor(0,0);
  display.println("READY");
  display.display();
}

// ═════════ LOOP ═════════

void loop() {

  if (digitalRead(BTN_START) == LOW) {
    running = true;
    delay(200);
  }

  if (digitalRead(BTN_STOP) == LOW) {
    running = false;
    stopMotors();
  }

  if (!running) return;

  readSensors();

  // ✅ STOP BAR
  if (allBlack()) {
    if (!allBlackForward) {
      motorLeft(BASE_SPEED, true);
      motorRight(BASE_SPEED, true);
      delay(150);
      allBlackForward = true;
    }

    readSensors();
    if (allBlack()) {
      stopMotors();
      running = false;
      return;
    }
  } else {
    allBlackForward = false;
  }

  // 🔀 FULL JUNCTION
  if (isJunction()) {
    motorLeft(JUNCTION_TURN_SPEED, true);
    motorRight(JUNCTION_TURN_SPEED, false);
    delay(JUNCTION_HOLD_MS);
    lastDir = 1;
    return;
  }

  // 🔀 CENTER + SIDE BRANCH
  if (centerActive()) {
    int branch = sideBranch();

    if (branch == 1) {
      motorLeft(JUNCTION_TURN_SPEED, true);
      motorRight(JUNCTION_TURN_SPEED, false);
      delay(JUNCTION_HOLD_MS);
      lastDir = 1;
      return;
    }

    if (branch == -1) {
      motorLeft(JUNCTION_TURN_SPEED, false);
      motorRight(JUNCTION_TURN_SPEED, true);
      delay(JUNCTION_HOLD_MS);
      lastDir = -1;
      return;
    }
  }

  float pos = getPosition();

  // ✅ STRONG LOST LINE RECOVERY
  if (pos == -1) {

    if (lastDir > 0) {
      motorLeft(SEARCH_SPEED, true);
      motorRight(SEARCH_SPEED, false);
    } else {
      motorLeft(SEARCH_SPEED, false);
      motorRight(SEARCH_SPEED, true);
    }

    return;
  }

  // 🎯 PID
  float error = pos - 3500;

  // ✅ direction memory — only update on significant deviation
  if (error > 600) lastDir = 1;
  else if (error < -600) lastDir = -1;

  float derivative = error - lastError;
  float correction = Kp * error + Kd * derivative;
  lastError = error;

  int leftSpeed  = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;

  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  motorLeft(leftSpeed, true);
  motorRight(rightSpeed, true);
}

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_TCS34725.h"
#include <ESP32Servo.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ================= 1. QUICK CONFIG SWITCHES =================
#define BLACK_IS_LOW     true   // 黑线输出 LOW(0) 填 true
#define INVERT_PID_STEER false  // 纠偏方向反了改成 true
#define ENABLE_BRANCHES  true   // 调试纯循线可先改 false，正式任务改 true
#define DEBUG_SERIAL     true   // 串口打印调试信息

// 连续扫到多少次全白 (11111) 才停机（每 1 次 = 10ms，50 次 ≈ 0.5 秒）
#define MAX_WHITE_COUNT  150

// ================= 2. PIN DEFINITIONS =================
#define OLED_SDA 21
#define OLED_SCL 22
#define TCS_SDA  5
#define TCS_SCL  15

#define IR_1 32
#define IR_2 33
#define IR_3 34
#define IR_4 35
#define IR_5 36

#define SELECT_BTN 0

#define TRIG_PIN 13
#define ECHO_PIN 14

#define PWMA 16
#define AIN1 17
#define AIN2 26
#define PWMB 18
#define BIN1 19
#define BIN2 23

#define BUZZER_PIN 27
#define SERVO_PIN  25

// ================= 3. OBJECTS & TUNING CONSTANTS =================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

TwoWire I2C_Color = TwoWire(1);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Servo gripperServo;

// --- PID Constants ---
float Kp = 24.0;
float Ki = 0.05;
float Kd = 18.0;
float lastError = 0.0, integral = 0.0;

// --- Speeds (0 - 255) ---
int baseSpeed     = 105;
int minCurveSpd   = 80;
int turnSpeed     = 115;
int approachSpd   = 85;
int maxMotorPWM   = 165;
int allWhiteCount = 0;

// --- Ultrasonic Thresholds (cm) ---
const float OBSTACLE_STOP_CM   = 15.0; // 小于 15cm 停下并只响 1 次
const float OBSTACLE_CLEAR_CM  = 22.0; // 大于 22cm 才算清空
int obstacleHitCount   = 0;
int obstacleClearCount = 0;

// --- Servo Angles ---
int servoOpenAngle = 95;  // 张开角度（开机/重启/未抓球时保持）
int servoGrabAngle = 0;   // 闭合角度（仅扫到目标色才闭合）

// --- Timing & Line Memory ---
int lastLineSide = 0;
int branchConfirmCount = 0;
unsigned long branchCooldownUntil = 0;
unsigned long lineLostTime   = 0;
unsigned long lastPidTime    = 0;
unsigned long lastPingTime   = 0;
unsigned long lastDebugTime  = 0;
const unsigned long PID_INTERVAL_MS = 10;
const unsigned long TURN_TIMEOUT_MS = 1500;

// State Machine
enum RobotState {
  STATE_AUTO_TRACK,
  STATE_OBSTACLE_WAIT,
  STATE_CHECK_BALL,
  STATE_GRAB_AND_RETURN,
  STATE_MISSION_DONE
};

RobotState currentState = STATE_AUTO_TRACK;
String targetColor = "RED";
bool onBranch      = false;
bool ballCollected = false;
int branchCount    = 0;

// ================= 4. HELPER FUNCTIONS =================
int readIR(int pin) {
  int raw = digitalRead(pin);
  return BLACK_IS_LOW ? !raw : raw; // 恒为 1 = 黑线，0 = 白底
}

void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed, -maxMotorPWM, maxMotorPWM);
  rightSpeed = constrain(rightSpeed, -maxMotorPWM, maxMotorPWM);

  if (leftSpeed > 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    analogWrite(PWMA, leftSpeed);
  } else if (leftSpeed < 0) {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -leftSpeed);
  } else {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, 0);
  }

  if (rightSpeed > 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    analogWrite(PWMB, rightSpeed);
  } else if (rightSpeed < 0) {
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -rightSpeed);
  } else {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, 0);
  }
}

void stopMotors() {
  setMotors(0, 0);
}

void resetPID() {
  lastError = 0.0;
  integral = 0.0;
  lastLineSide = 0;
  lineLostTime = 0;
  allWhiteCount = 0;
}

void beepAlert(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
  }
}

void updateOLED(String line1, String line2) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Target: "); display.print(targetColor);
  display.print(ballCollected ? " [HOLD]" : " [OPEN]");
  display.setTextSize(2);
  display.setCursor(0, 18);
  display.println(line1);
  display.setTextSize(1);
  display.setCursor(0, 46);
  display.println(line2);
  display.display();
}

void pivotUntilCenterLine(int leftSpd, int rightSpd, unsigned long blindDelayMs) {
  setMotors(leftSpd, rightSpd);
  delay(blindDelayMs);

  unsigned long startWait = millis();
  while (readIR(IR_3) == 0 && (millis() - startWait < TURN_TIMEOUT_MS)) {
    setMotors(leftSpd, rightSpd);
    delay(5);
  }
  stopMotors();
  resetPID();
}

// ================= 5. STARTUP TARGET COLOR SELECTOR =================
void selectTargetColorAtBoot() {
  String colors[3] = {"RED", "GREEN", "BLUE"};
  int idx = 0;
  targetColor = colors[idx];

  gripperServo.write(servoOpenAngle);

  unsigned long lastPressTime = millis();
  updateOLED("TARGET: " + targetColor, "Tap BOOT to change");

  while (millis() - lastPressTime < 3500) {
    if (digitalRead(SELECT_BTN) == LOW) {
      delay(40);
      if (digitalRead(SELECT_BTN) == LOW) {
        idx = (idx + 1) % 3;
        targetColor = colors[idx];
        updateOLED("TARGET: " + targetColor, "Tap BOOT to change");
        beepAlert(1);
        lastPressTime = millis();
        while (digitalRead(SELECT_BTN) == LOW) delay(10);
      }
    }
    delay(10);
  }

  updateOLED("READY!", "Go: " + targetColor + " Ball");
  beepAlert(2);
  delay(400);
}

// ================= 6. SENSORS (FILTERED ULTRASONIC & SENSITIVE RGB) =================
float getRawDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 15000);
  if (duration == 0) return 999.0;
  return duration * 0.034 / 2.0;
}

float getFilteredDistanceCM() {
  float d1 = getRawDistanceCM(); delay(10);
  float d2 = getRawDistanceCM(); delay(10);
  float d3 = getRawDistanceCM();

  if ((d1 >= d2 && d1 <= d3) || (d1 <= d2 && d1 >= d3)) return d1;
  if ((d2 >= d1 && d2 <= d3) || (d2 <= d1 && d2 >= d3)) return d2;
  return d3;
}

String detectBallColor() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  if (c == 0) c = 1;

  float rPct = (float)r / c * 100.0;
  float gPct = (float)g / c * 100.0;
  float bPct = (float)b / c * 100.0;

  Serial.printf("RGB -> R:%d(%.1f%%) G:%d(%.1f%%) B:%d(%.1f%%) C:%d\n",
                r, rPct, g, gPct, b, bPct, c);

  if (c < 40) return "DARK";

  if ((r > g * 1.25 && r > b * 1.25) || (rPct > 37.0 && r > g && r > b)) {
    return "RED";
  }
  if ((g > r * 1.15 && g > b * 1.15) || (gPct > 36.0 && g > r && g > b)) {
    return "GREEN";
  }
  if ((b > r * 1.15 && b > g * 1.15) || (bPct > 36.0 && b > r && b > g)) {
    return "BLUE";
  }
  return "UNKNOWN";
}

// ================= 7. ENHANCED PID & NAVIGATION =================
void exitBranchToMainTrack() {
  setMotors(-baseSpeed, -baseSpeed);
  delay(720);
  stopMotors();
  delay(120);

  pivotUntilCenterLine(-turnSpeed, turnSpeed, 380);
  onBranch = false;
  branchCooldownUntil = millis() + 1500;
}

void runPIDLineFollow() {
  if (millis() - lastPidTime < PID_INTERVAL_MS) return;
  lastPidTime = millis();

  // 1. 仅在主赛道检测障碍物
  if (!onBranch && (millis() - lastPingTime >= 80)) {
    lastPingTime = millis();
    float dist = getRawDistanceCM();

    if (dist > 1.5 && dist < OBSTACLE_STOP_CM) {
      obstacleHitCount++;
      if (obstacleHitCount >= 3) {
        obstacleHitCount = 0;
        obstacleClearCount = 0;
        stopMotors();
        updateOLED("OBSTACLE!", "Dist: " + String(dist, 1) + " cm");
        beepAlert(1);
        currentState = STATE_OBSTACLE_WAIT;
        return;
      }
    } else {
      obstacleHitCount = 0;
    }
  }

  // 2. 读取标准化后的 5 路红外（1 = 黑线，0 = 白底）
  int s1 = readIR(IR_1);
  int s2 = readIR(IR_2);
  int s3 = readIR(IR_3);
  int s4 = readIR(IR_4);
  int s5 = readIR(IR_5);

  // A. 未抓球时，只要 5 路全黑 -> 立刻停下来进入 5 秒测色模式
  if (!ballCollected && (s1 && s2 && s3 && s4 && s5)) {
    stopMotors();
    delay(150);
    currentState = STATE_CHECK_BALL;
    return;
  }

  // B. 拿球后主赛道终点卸球区检测
  if (!onBranch && ballCollected && (s1 && s2 && s3 && s4 && s5)) {
    stopMotors();
    delay(200);
    gripperServo.write(servoOpenAngle);
    delay(400);
    setMotors(-baseSpeed, -baseSpeed);
    delay(400);
    stopMotors();
    currentState = STATE_MISSION_DONE;
    updateOLED("COMPLETE!", "Delivered " + targetColor);
    beepAlert(4);
    return;
  }

  // C. 左侧 T 字分支入口（带冷却时间 + 连续 3 次防抖）
  if (ENABLE_BRANCHES && !onBranch && !ballCollected && millis() > branchCooldownUntil) {
    if (s1 == 1 && s2 == 1 && s3 == 1 && s5 == 0) {
      branchConfirmCount++;
      if (branchConfirmCount >= 3) {
        branchConfirmCount = 0;
        branchCount++;
        updateOLED("BRANCH #" + String(branchCount), "Entering...");
        setMotors(baseSpeed, baseSpeed);
        delay(210);
        pivotUntilCenterLine(-turnSpeed, turnSpeed, 380);
        onBranch = true;
        return;
      }
    } else {
      branchConfirmCount = 0;
    }
  }

  // D. 右侧 90 度直角弯
  if (!onBranch && s1 == 0 && s3 == 1 && s4 == 1 && s5 == 1) {
    setMotors(baseSpeed, baseSpeed);
    delay(210);
    pivotUntilCenterLine(turnSpeed, -turnSpeed, 320);
    return;
  }

  // E. 细黑线防夹缝处理：连续扫到 MAX_WHITE_COUNT 次全白 (11111) 才真正停机
  int activeCount = s1 + s2 + s3 + s4 + s5;
  if (activeCount == 0) {
    allWhiteCount++;

    if (allWhiteCount < MAX_WHITE_COUNT) {
      setMotors(minCurveSpd, minCurveSpd);
    } else {
      allWhiteCount = MAX_WHITE_COUNT;
      stopMotors();
    }
    return;
  }

  // 只要重新扫到黑线，立刻把全白计数清零
  allWhiteCount = 0;

  // F. 加权 PID 计算
  float error = ((-2.0 * s1) + (-1.0 * s2) + (0.0 * s3) + (1.0 * s4) + (2.0 * s5)) / activeCount;

  if (error < -0.3) lastLineSide = -1;
  else if (error > 0.3) lastLineSide = 1;
  else lastLineSide = 0;

  if ((error > 0 && lastError < 0) || (error < 0 && lastError > 0)) {
    integral = 0;
  }
  integral = constrain(integral + error, -35.0, 35.0);

  float derivative = error - lastError;
  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  if (INVERT_PID_STEER) correction = -correction;

  int targetSpd = map((int)(abs(error) * 100), 0, 200, baseSpeed, minCurveSpd);
  targetSpd = constrain(targetSpd, minCurveSpd, baseSpeed);

  int leftOut  = targetSpd + (int)correction;
  int rightOut = targetSpd - (int)correction;
  setMotors(leftOut, rightOut);

  if (DEBUG_SERIAL && (millis() - lastDebugTime >= 200)) {
    lastDebugTime = millis();
    Serial.printf("IR(1=Blk): %d%d%d%d%d | Err: %.2f | L:%d R:%d\n",
                  s1, s2, s3, s4, s5, error, leftOut, rightOut);
  }
}

void handleObstacleWait() {
  stopMotors();
  float dist = getFilteredDistanceCM();
  delay(80);

  if (dist > OBSTACLE_CLEAR_CM) {
    obstacleClearCount++;
    if (obstacleClearCount >= 10) {
      obstacleClearCount = 0;
      if (!ballCollected) {
        gripperServo.write(servoOpenAngle);
      }
      updateOLED("PATH CLEAR", "Resuming PID...");
      delay(250);
      resetPID();
      currentState = STATE_AUTO_TRACK;
    }
  } else {
    obstacleClearCount = 0;
  }
}

void handleBallCheck() {
  stopMotors();
  unsigned long checkStart = millis();
  int matchCount = 0;
  String lastSeen = "UNKNOWN";

  while (millis() - checkStart < 5000) {
    int secLeft = 5 - (millis() - checkStart) / 1000;
    lastSeen = detectBallColor();
    updateOLED("CHK (" + String(secLeft) + "s)", "Seen: " + lastSeen);

    if (lastSeen == targetColor) {
      matchCount++;
      if (matchCount >= 2) {
        updateOLED("LOCKED: " + lastSeen, "Closing gripper...");
        beepAlert(3);
        currentState = STATE_GRAB_AND_RETURN;
        return;
      }
    } else {
      matchCount = 0;
    }
    delay(150);
  }

  updateOLED("WRONG: " + lastSeen, "Backing up...");
  beepAlert(1);
  gripperServo.write(servoOpenAngle);
  exitBranchToMainTrack();
  currentState = STATE_AUTO_TRACK;
  updateOLED("AUTO PID", "Searching " + targetColor);
}

void handleGrabAndReturn() {
  gripperServo.write(servoOpenAngle);
  delay(200);
  setMotors(approachSpd, approachSpd);
  delay(310);
  stopMotors();
  delay(150);

  gripperServo.write(servoGrabAngle);
  delay(500);
  ballCollected = true;
  updateOLED("GRABBED!", "Returning to track");

  exitBranchToMainTrack();
  currentState = STATE_AUTO_TRACK;
  updateOLED("DELIVERING", targetColor + " Ball");
}

// ================= Opening Music =================
// 音乐配置
const int NOTE_B4 = 494;
const int NOTE_D5 = 587;
const int NOTE_E5 = 659;
const int beat = 400;

struct Note {
  int freq;     // 频率 (Hz)，0 表示停顿/休止
  float beats;  // 节拍数
};

// 曲谱列表：填 {频率, 节拍}；如果中间某段要停顿，填 {0, 0.5f}（毫秒级停顿可直接用 pauseMs）
const Note MELODY[] = {
  {NOTE_E5, 0.25f},
  {NOTE_E5, 0.25f},
  {NOTE_E5, 0.25f},
  {NOTE_E5, 0.25f},
  {NOTE_D5, 0.5f},
  {NOTE_D5, 0.5f},
  {NOTE_B4, 0.5f},
  {NOTE_B4, 0.5f},
  {NOTE_D5, 1.0f},

  {0, 0.5f},        // 对应原代码里的 delay(200) 停顿（200ms = 0.5 拍 @ 400ms/拍）

  {NOTE_E5, 0.25f},
  {NOTE_E5, 0.25f},
  {NOTE_E5, 0.25f},
  {NOTE_E5, 0.25f},
  {NOTE_D5, 0.5f},
  {NOTE_D5, 0.5f},
  {NOTE_B4, 0.5f},
  {NOTE_B4, 0.5f},
  {NOTE_D5, 1.0f}
};

const int MELODY_LEN = sizeof(MELODY) / sizeof(MELODY[0]);

void playCleanTone(int freq, float durationBeat) {
  int durationMs = static_cast<int>(beat * durationBeat);

  if (freq > 0) {
    tone(buzzerPin, freq);
    delay(durationMs);
    noTone(buzzerPin);
  } else {
    delay(durationMs);
  }

  digitalWrite(buzzerPin, HIGH); // 强制拉高静音
  if (freq > 0) {
    delay(50);
  }
}

void PlayMusic() {
  for (int i = 0; i < MELODY_LEN; i++) {
    playCleanTone(MELODY[i].freq, MELODY[i].beats);
  }
  digitalWrite(buzzerPin, HIGH);
}

// ================= 8. SETUP & MAIN LOOP =================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  gripperServo.setPeriodHertz(50);
  gripperServo.attach(SERVO_PIN, 500, 2400);
  gripperServo.write(servoOpenAngle);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);
  pinMode(SELECT_BTN, INPUT_PULLUP);

  Serial.begin(115200);
  delay(300);
  Serial.println("\n--- BOOTING AUTONOMOUS ROBOT ---");

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  stopMotors();

  pinMode(IR_1, INPUT); pinMode(IR_2, INPUT); pinMode(IR_3, INPUT);
  pinMode(IR_4, INPUT); pinMode(IR_5, INPUT);

  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(50);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("WARNING: OLED not found at 0x3C");
  } else {
    display.clearDisplay();
    display.display();
  }

  I2C_Color.begin(TCS_SDA, TCS_SCL);
  I2C_Color.setTimeOut(50);
  if (!tcs.begin(TCS34725_ADDRESS, &I2C_Color)) {
    Serial.println("WARNING: TCS34725 not found on GPIO 5/15");
  }

  PlayMusic();
  
  selectTargetColorAtBoot();
  currentState = STATE_AUTO_TRACK;
}

void loop() {
  switch (currentState) {
    case STATE_AUTO_TRACK:
      runPIDLineFollow();
      break;
    case STATE_OBSTACLE_WAIT:
      handleObstacleWait();
      break;
    case STATE_CHECK_BALL:
      handleBallCheck();
      break;
    case STATE_GRAB_AND_RETURN:
      handleGrabAndReturn();
      break;
    case STATE_MISSION_DONE:
      stopMotors();
      break;
  }
}
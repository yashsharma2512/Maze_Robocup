/*
  ---------------- ROBOT WALL FOLLOWER ----------------
  Features:
    - MPU6050 for orientation
    - Ultrasonic sensors for wall and obstacle detection
    - L298N Motor Driver for movement
    - NeoPixel LEDs for status indication
    - PID-based turning
    - Simple wall alignment while moving straight
*/

// ------------------- Libraries -------------------
#include <Wire.h>
#include <MPU6050_light.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_AS7341.h>


// -------------------- AS7341 --------------------
Adafruit_AS7341 as7341;

// ------------------- NeoPixel -------------------
#define LED_PIN 33
#define LED_COUNT 7
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// ------------------- Ultrasonic Pins -------------------
#define TRIG_LEFT 3
#define ECHO_LEFT 2
#define TRIG_FRONT 9
#define ECHO_FRONT 8

// ------------------- Motor Driver (L298N) Pins -------------------
#define ENB 22  // Right motor speed
#define IN3 23
#define IN4 24
#define ENA 27  // Left motor speed
#define IN1 26
#define IN2 25

// ------------------- MPU6050 -------------------
#define LEFT -1
#define RIGHT 1
#define UTURN 2

#define kp_turn 9
#define kd_turn 3.5
#define MAXSPEED_turning 135
#define BASESPEED_move 100

MPU6050 mpu(Wire);

// ------------------- Globals -------------------
#define Kp_run 8
int distLeftLAST = 0;
int distFrontLAST = 0;
int wallOffset = 6;

// =============================================================
// =============== UTILITY FUNCTIONS ===========================
// =============================================================

// ------------------------ AS7341 ------------------------
void initAS7341Sensor() {
  if (!as7341.begin()) {
    Serial.println("Could not find AS7341");
    while (1) { delay(10); }
  }

  as7341.setATIME(10);
  as7341.setASTEP(30);
  as7341.setGain(AS7341_GAIN_512X);
  as7341.setLEDCurrent(60);
  as7341.enableLED(false);
  delay(100);
  as7341.enableLED(true);
}

int readColor() {
  uint16_t readings[12];

  if (!as7341.readAllChannels()) {
    Serial.println("Error reading all channels!");
    return -1;
  }


  uint16_t red = as7341.getChannel(AS7341_CHANNEL_630nm_F7);
  uint16_t green = as7341.getChannel(AS7341_CHANNEL_555nm_F5);
  uint16_t blue = as7341.getChannel(AS7341_CHANNEL_480nm_F3);

  int RG = (red + green) / 2;
  Serial.println(blue);
  Serial.print(',');
  Serial.println(RG);
  Serial.print(' ');
  if (red < 45 and RG < 45)
    if (blue > 38)
      return 1;
    else
      return 0;
  return -1;
}

// ------------------- NeoPixel Helpers -------------------
void clearStrip() {
  strip.clear();
  delay(10);
  strip.show();
  delay(10);
}

void showColor(uint32_t color) {
  clearStrip();
  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

// ------------------- Wait for Button Input -------------------
void waitForInput() {
  int KEY_INPUT = 21;
  pinMode(KEY_INPUT, INPUT_PULLUP);

  // Wait until button is pressed
  while (digitalRead(KEY_INPUT)) {
    delay(20);
  }
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);

  // Wait until button is released
  while (!digitalRead(KEY_INPUT))
    ;
  delay(100);
}

// ------------------- Ultrasonic Distance -------------------
long getDistanceOnce(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000);
  if (duration == 0) return 200;  // Timeout -> return large distance
  return duration * 0.034 / 2;    // Convert to cm
}

// =============================================================
// =============== MOTOR FUNCTIONS =============================
// =============================================================

// ------------------- Set Motor Speeds -------------------
void setMotors(int leftSpeed, int rightSpeed) {
  // Left Motor
  if (leftSpeed >= 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, leftSpeed);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, -leftSpeed);
  }

  // Right Motor
  if (rightSpeed >= 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, rightSpeed);
  } else {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, -rightSpeed);
  }
}

// ------------------- Stop Motors -------------------
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// =============================================================
// =============== MOVEMENT FUNCTIONS ==========================
// =============================================================

// ------------------- Move Straight (Wall Alignment) -------------------
void moveStraightAligned(int leftDistance) {
  int error = 0;

  // Only correct if wall is detected
  if (leftDistance < 12) {
    error = leftDistance - wallOffset;
  }

  // Serial.println(error);

  int L = BASESPEED_move - error * Kp_run;
  int R = BASESPEED_move + error * Kp_run;

  L = constrain(L, -255, 255);
  R = constrain(R, -255, 255);

  setMotors(L, R);
}

// ------------------- Turn Robot -------------------
void turnBot(int degrees, int direction) {
  stopMotors();
  delay(5);

  mpu.resetAngleZ();

  unsigned long timer = 0;
  unsigned long timerExit = 0;
  bool enableExitTimer = false;
  float _pe = 0;

  unsigned long MAX_TIMER = millis();

  while (true) {
    mpu.update();

    if ((millis() - timer) > 10) {
      int error = abs(degrees) - abs(mpu.getAngleZ());

      if (error <= 0 && !enableExitTimer) {
        enableExitTimer = true;
        timerExit = millis();
      }

      // PID terms
      float _ce = error - _pe;
      _pe = error;

      float _p = error * kp_turn;
      float _d = _ce * kd_turn;

      int drive = _p + _d;

      // Speed control
      float turning = MAXSPEED_turning;
      int minSpeed = 60;

      drive = constrain(drive, -turning, turning);

      // Enforce minimum speed
      if (drive > 0 && drive < minSpeed) drive = minSpeed;
      if (drive < 0 && drive > -minSpeed) drive = -minSpeed;

      setMotors(direction * drive, -direction * drive);

      // Exit conditions
      if (enableExitTimer && (millis() - timerExit) > 200) break;
      if ((millis() - MAX_TIMER) > 1700) break;

      timer = millis();
    }
  }

  stopMotors();
  delay(100);
  mpu.resetAngleZ();
}

// =============================================================
// =============== MPU FUNCTIONS ===============================
// =============================================================
void beginMpu() {
  Wire.begin();
  byte status = mpu.begin(1, 0);

  Serial.print(F("MPU6050 status: "));
  Serial.println(status);

  if (status != 0) {
    while (1) {
      Serial.println("MPU ERROR!");
      delay(500);
    }
  }

  delay(100);
  Serial.println("MPU START OK");
}

// =============================================================
// =============== SETUP =======================================
// =============================================================
void setup() {
  Serial.begin(115200);

  // ------------------- Board-specific setup -------------------
  // Configure PIO (Programmable I/O) on RP2350 at base pin 16
  // ⚠️ Keep this if you’re using RP2350; not required on Arduino/ESP32
  pio_set_gpio_base(pio0, 16);

  // ------------------- NeoPixel -------------------
  strip.begin();
  strip.show();
  strip.setBrightness(50);

  // ------------------- Motors -------------------
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopMotors();

  // ------------------- Ultrasonic -------------------
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  // -------------------- AS7341 ----------------------
  initAS7341Sensor();


  // ------------------- Startup sequence -------------------
  showColor(strip.Color(127, 0, 0));  // Red
  waitForInput();
  showColor(strip.Color(0, 50, 0));  // Green
  Serial.println("start");

  // ------------------- MPU -------------------
  beginMpu();
  mpu.calcOffsets(true, true);
  mpu.resetAngleZ();

  // Warm-up sensors
  for (int i = 0; i < 10; i++) {
    mpu.update();
    getDistanceOnce(TRIG_LEFT, ECHO_LEFT);
    getDistanceOnce(TRIG_FRONT, ECHO_FRONT);
    delay(50);
  }

  Serial.println("calibrated...");
  showColor(strip.Color(0, 127, 0));  // Bright Green
  waitForInput();
}

// =============================================================
// =============== MAIN LOOP ===================================
// =============================================================

void loop() {
  mpu.update();

  int distLeft = getDistanceOnce(TRIG_LEFT, ECHO_LEFT);
  int distFront = getDistanceOnce(TRIG_FRONT, ECHO_FRONT);

  // Simple filter (smoothing)
  distLeft = (distLeftLAST * 4 + distLeft * 6) / 10;
  distFront = (distFrontLAST * 5 + distFront * 5) / 10;
  Serial.println(distLeft);
  distLeftLAST = distLeft;
  distFrontLAST = distFront;


  int color = readColor();
  Serial.println(color);

  // ------------------- Obstacle handling -------------------   
  if (color == 0) {
    turnBot(90, RIGHT);
    turnBot(90, RIGHT);
  }
    else if(color == 1)
  {
    stopMotors();
    delay(5000);
    showColor(strip.Color(100, 100,255));
  }
  // if(color == 1)
  // {
  //   stopMotors();
  //   delay(5000);
  //   showColor(strip.Color(100, 100,255));
  //   if(color == 0)
  //   {
  //   stopMotors();
  //   delay(5000);
  //   showColor(strip.Color(100, 100,255));
  //   }

  // }

  if (distFront < 10) {
    stopMotors();
    delay(200);
    distLeft = getDistanceOnce(TRIG_LEFT, ECHO_LEFT);
    if (distLeft < 15)
      turnBot(90, RIGHT);
    else
      turnBot(90, LEFT);
  }

  if (distLeft > 30) {
    showColor(strip.Color(0, 0, 127));
    delay(700);
    stopMotors();
    turnBot(90, LEFT);
    long curr = millis();
    delay(1);
    showColor(strip.Color(127, 0, 127));
    while (millis() - curr < 700) {
      distLeft = getDistanceOnce(TRIG_LEFT, ECHO_LEFT);
      distLeft = (distLeftLAST * 4 + distLeft * 6) / 10;
      distLeftLAST = distLeft;
      moveStraightAligned(distLeft);
      delay(30);
    }
  }


  // ------------------- Default: move forward -------------------
  moveStraightAligned(distLeft);
  delay(100);
}

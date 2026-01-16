#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LedControl.h>

const int DIN = 11;
const int CLK = 13;
const int CS = 12;

const int BUZZER = 7;

const int LED_G = 9;
const int LED_R = 8;

const int start = 5;
const int finish = 4;

LedControl lc = LedControl(DIN, CLK, CS, 1);
Adafruit_MPU6050 mpu;

const byte Dote_UP[8] = {
  B00011000, 
  B00111100, 
  B01111110, 
  B00011000,
  B00011000, 
  B00011000, 
  B00011000, 
  B00011000
};

const byte Dote_DOWN[8] = {
  B00011000, 
  B00011000, 
  B00011000, 
  B00011000,
  B00011000, 
  B01111110, 
  B00111100, 
  B00011000
};

const byte Dote_LEFT[8] = {
  B00011000,
  B00111000,
  B01111000,
  B11111111,
  B11111111,
  B01111000,
  B00111000,
  B00011000
};

const byte Dote_RIGHT[8] = {
  B00011000,
  B00011100,
  B00011110,
  B11111111,
  B11111111,
  B00011110,
  B00011100,
  B00011000
};

enum Direction { UP, DOWN, LEFT, RIGHT };
Direction targetDir = UP;

enum GameState { IDLE, WAIT_INPUT, SHOW_RESULT };
GameState state = IDLE;

unsigned long stateStartMs = 0;

const unsigned long TIME_LIMIT_MS = 2000;
const unsigned long RESULT_SHOW_MS = 300;

// 기울기 판정 임계값
const float TILT_THRESHOLD = 4.0; //약 20-25°

// 재입력 방지
const float NEUTRAL_THRESHOLD = 2.0;
bool neutralReady = true;

void clearMatrix() {
  for (int r = 0; r < 8; r++) {
    lc.setRow(0, r, 0);
  } 
}

void drawBitmap(const byte bmp[8]) {
  for (int r = 0; r < 8; r++) {
    lc.setRow(0, r, bmp[r]); 
  }
}

void showArrow(Direction d) {
  switch (d) {
    case UP: drawBitmap(Dote_UP); break;
    case DOWN: drawBitmap(Dote_DOWN); break;
    case LEFT: drawBitmap(Dote_LEFT); break;
    case RIGHT: drawBitmap(Dote_RIGHT); break;
  }
}

void ledsOff() {
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_R, LOW);
}

void beepOK() {
  tone(BUZZER, 1200, 90);
  delay(110);
  tone(BUZZER, 1600, 90);
  delay(110);
  noTone(BUZZER);
}

void beepBad() {
  tone(BUZZER, 250, 160);
  delay(180);
  noTone(BUZZER);
}

Direction randomDirection() {
  return (Direction)random(0, 4);
}

// 현재 가속도 읽기
void readAccel(float &ax, float &ay) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  ax = a.acceleration.x;
  ay = a.acceleration.y;
}

bool readTiltDirection(Direction &outDir) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;

  if (ay > TILT_THRESHOLD) {outDir = UP; return true;}
  if (ay < -TILT_THRESHOLD) {outDir = DOWN; return true;}
  if (ax > TILT_THRESHOLD) {outDir = RIGHT; return true;}
  if (ax < -TILT_THRESHOLD) {outDir = LEFT; return true;}

  return false;
}

bool pressedEdge(int pin) {
  static uint8_t lastStart = HIGH;
  static uint8_t lastFinish = HIGH;

  uint8_t cur = digitalRead(pin);

  delay(10);
  uint8_t cur2 = digitalRead(pin);
  if (cur != cur2) return false;

  bool edge = false;

  if (pin == start) {
    if (lastStart == HIGH && cur2 == LOW) edge = true;
    lastStart = cur2;
  } else if (pin == finish) {
    if (lastFinish == HIGH && cur2 == LOW) edge = true;
    lastFinish = cur2;
  }

  return edge;
}

void showIdleScreen() {
  clearMatrix();
  ledsOff();
  state = IDLE;
}

void startRound() {
  ledsOff();
  targetDir = randomDirection();
  showArrow(targetDir);
  state = WAIT_INPUT;
  stateStartMs = millis();
  neutralReady = false;
}

void nextRound() {
  state = SHOW_RESULT;
  stateStartMs = millis();
}

void setup() {
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_R, OUTPUT);
  ledsOff();

  pinMode(start, INPUT_PULLUP);
  pinMode(finish, INPUT_PULLUP);

  Serial.begin(115200);
  randomSeed(analogRead(A0));

  lc.shutdown(0, false);
  lc.setIntensity(0, 8);
  lc.clearDisplay(0);

  // MPU6050 초기화
  Wire.begin();
  if (!mpu.begin()) {
    while (1) {
      digitalWrite(LED_R, HIGH); delay(200);
      digitalWrite(LED_R, LOW);  delay(200);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  showIdleScreen();
}

void loop() {
  unsigned long now = millis();

  if (pressedEdge(finish)) {
    showIdleScreen();
    return;
  }

  if (state == IDLE) {
    if (pressedEdge(start)) {
      clearMatrix();
      startRound();
    }
    return;
  }

  // 중립 상태 확인
  if (!neutralReady) {
    float ax, ay;
    readAccel(ax, ay);
    if (fabs(ax) < NEUTRAL_THRESHOLD && fabs(ay) < NEUTRAL_THRESHOLD) {
      neutralReady = true;
    }
  }

  switch (state) {
    case WAIT_INPUT: {
      // 시간 초과
      if (now - stateStartMs > TIME_LIMIT_MS) {
        digitalWrite(LED_R, HIGH);
        beepBad();
        nextRound();
        break;
      }

      // 입력 감지
      if (neutralReady) {
        Direction sensed;
        if (readTiltDirection(sensed)) {
          if (sensed == targetDir) {
            digitalWrite(LED_G, HIGH);
            beepOK();
          } else {
            digitalWrite(LED_R, HIGH);
            beepBad();
          }
          nextRound();
        }
      }
      break;
    }

    case SHOW_RESULT: {
      if (now - stateStartMs > RESULT_SHOW_MS) {
        neutralReady = false;
        startRound();
      }
      break;
    }

    default:
      break;
  }
}
